/*
Package ar3 is a Golang library implementing serial commands for the AR3
robotic arm.

Basics

The AR3 robotic arm is a low cost + open source 6 axis robotic arm designed
and created by Chris Annin of Annin Robotics. Annin Robotics provides software
for operating the robotic arm, but this software is designed for windows and is
not useful for devices for like raspberry pis.

This Golang library takes over for the local robotic control software by
implementing the functions needed to communicate with the arduino on the AR3 or
AR2 robot. Specifically, we implement the functions for:

 - Echo
 - MoveSteppers
 - Calibrate
 - MoveAngles

We do not yet support encoders in the AR3, nor any other commands. All other
rountines can be reproduced in code and not directly on the robot.

Testing

We do not test any of the code for connecting to the AR3 robot. We do not test
it because there is no easy way for us to validate that the code is running as
expected (because in order to do that, we have to connect to an AR3 robotic
arm).

There would be an option to simulate the code if we first simulated the AR3
arduino on local development machines and then connected the simulation to a
simulated serial port (probably using something like socat). However, this does
not actually guarantee proper connection in real situations.

Instead, we depend on the AR3 Echo command, which runs in the Connect command,
to validate that the robot has been connected properly over serial.
*/
package ar3

import (
	"fmt"
	"golang.org/x/sys/unix"
	"os"
	"unsafe"
)

// stepper is a representation of a stepper motor on the robot.
type stepper struct {
	currentStep int
	stepLim     int
}

// AR3 struct represents an AR3 robotic arm connected to a serial port.
type AR3 struct {
	serial *os.File
	j1     stepper
	j2     stepper
	j3     stepper
	j4     stepper
	j5     stepper
	j6     stepper
}

// The following StepLims are hard-coded in the ARbot.cal file for the stepper
// motors. These should not change.
var j1stepLim int = 15200
var j2stepLim int = 7300
var j3stepLim int = 7850
var j4stepLim int = 15200
var j5stepLim int = 4575
var j6stepLim int = 6625

// Connect connects to the AR3 over serial.
func Connect(serialConnectionStr string) (AR3, error) {
	// Set up connection to the serial port
	f, err := os.OpenFile(serialConnectionStr, unix.O_RDWR|unix.O_NOCTTY|unix.O_NONBLOCK, 0666)
	if err != nil {
		return AR3{}, err
	}
	rate := uint32(unix.B115200) // 115200 is the default Baud rate of the AR3 arm
	cflagToUse := unix.CREAD | unix.CLOCAL | rate
	// We use rational defaults from https://github.com/tarm/serial/blob/master/serial_linux.go
	cflagToUse |= unix.CS8
	// Get Unix file descriptor
	fd := f.Fd()
	t := unix.Termios{
		Iflag:  unix.IGNPAR,
		Cflag:  cflagToUse,
		Ispeed: rate,
		Ospeed: rate,
	}
	t.Cc[unix.VMIN] = uint8(1)
	t.Cc[unix.VTIME] = uint8(10) // Default timeout is 1s

	_, _, errno := unix.Syscall6(
		unix.SYS_IOCTL,
		uintptr(fd),
		uintptr(unix.TCSETS),
		uintptr(unsafe.Pointer(&t)),
		0,
		0,
		0,
	)
	if errno != 0 {
		return AR3{}, err
	}

	// Instantiate a new AR3 object that holds our serial port. Additionally, set default stepLims, which are hard-coded in the AR3 software
	newAR3 := AR3{serial: f, j1: stepper{0, j1stepLim}, j2: stepper{0, j2stepLim}, j3: stepper{0, j3stepLim}, j4: stepper{0, j4stepLim}, j5: stepper{0, j5stepLim}, j6: stepper{0, j6stepLim}}

	// Test to see if we can connect to the newAR3
	err = newAR3.Echo("Test")
	if err != nil {
		return newAR3, err
	}

	// If we can echo, return newAR3 object
	return newAR3, nil
}

// Echo echos back a string sent to the AR3. This uses the "Echo" function on
// the AR3 arduino. Useful for testing connectivity to the AR3.
func (ar3 *AR3) Echo(str string) error {
	// Send echo to the device
	stringToSend := fmt.Sprintf("TM%s\n", str)
	_, err := ar3.serial.Write([]byte(stringToSend))
	if err != nil {
		return err
	}

	// Read output of echo
	buf := make([]byte, 128)
	n, err := ar3.serial.Read(buf)
	if err != nil {
		return err
	}

	// Double check to make sure the length is > 1
	if n < 2 {
		return fmt.Errorf("Return from echo is empty. Is the serial port responding properly?")
	}

	// See if we had the same bytes returned
	// Note: the serial returns with your string with \n\r\n between two double-quotes ("), so we remove these characters
	stringOutput := fmt.Sprintf("%q", buf[:n])
	stringOutput = stringOutput[1 : len(stringOutput)-7]
	if stringOutput != str {
		return fmt.Errorf("Failed echo to AR3. Expected %s but got %s", str, stringOutput)
	}

	// If we got the same string back, success
	return nil
}

// moveCommandGenerator generates a move command string.
func moveCommandGenerator(speed, accdur, accspd, dccdur, dccspd, j1, j2, j3, j4, j5, j6, tr int) string {
	command := "MJ"
	// First, compute direction. If the stepper is negative, that means that direction is set to 1.
	// We are going to compute these as a list, and then append them to a growing string
	var jdirection int
	// The move string is assembled with the beginning of an alphabetical character for each axis.
	// These were derived from line 4493 in the ARCS source file under the variable "commandCalc".
	alphabetForCommands := []string{"A", "B", "C", "D", "E", "F", "T"}
	for i, j := range []int{j1, j2, j3, j4, j5, j6, tr} {
		jdirection = 0
		if j < 0 {
			jdirection = 1
			j = -1 * j
		}
		command = command + fmt.Sprintf("%s%d%d", alphabetForCommands[i], jdirection, j)
	}

	// We now have the axis commands, so we need to add the speed, accspd, accdur, dccdur, and dccspd.
	// These are also derived from the above commandCalc.
	command = command + fmt.Sprintf("S%dG%dH%dI%dK%d", speed, accspd, accdur, dccdur, dccspd)

	return command
}

// MoveSteppers moves each of the AR3's stepper motors by a certain amount of steps.
// In addition to the j1,j2,j3,j4,j5,j6 positions, you can also define 5 other
// variables: ACCdur, ACCspd, DCCdur, and DCCspd (these are named DEC on ARCS
// but DCC on the arduino controller), which define the acceleration duration
// and speed of the stepper motors. Good defaults are:
//  speed: 25 (line 7941 on ARCS)
//  accdur: 15 (line 7942 on ARCS)
//  accspd: 10 (line 7943 on ARCS)
//  dccdur: 20 (line 7944 on ARCS)
//  dccspd: 5 (line 7945 on ARCS)
//
// Tr is also an active variable that can be changed. It is for controlling
// the AR3 arm on a track, but it would appear that has not been implemented.
// Unless you know what you're doing, please keep this variable at 0.
//
// MoveSteppers does not have ANY checks. Please double check the values getting fed to
// MoveSteppers or else the robot WILL self destruct.
func (ar3 *AR3) MoveSteppers(speed, accdur, accspd, dccdur, dccspd, j1, j2, j3, j4, j5, j6, tr int) error {
	command := moveCommandGenerator(speed, accdur, accspd, dccdur, dccspd, j1, j2, j3, j4, j5, j6, tr)

	// Send command
	_, err := ar3.serial.Write([]byte(command))
	if err != nil {
		return err
	}

	// Normally, we would check here for successful completion. However, there IS no way to check for
	// successful completion implemented in the AR3 code. So we do not check for this.
	return nil
}

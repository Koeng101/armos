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

 - CurrentPosition
 - Echo
 - Calibrate
 - MoveSteppers
 - SetDirections

We do not yet support encoders in the AR3, nor any other commands. All other
rountines can be reproduced in code and not directly on the robot.

Testing

Testing can be done with the AR3simulate struct, which satisfies all of the
interfaces of AR3. For real connection to a robot, use connect to the robot
using `Connect` instead of `ConnectMock`.

Compatibility

The code here is only designed to function on linux machines directly connected
to the AR3 robotic arm. It uses a couple of serial port configuration variables
specific to linux systems. We use as few packages as possible, with the only
non-standard package being golang.org/x/sys/unix, which carries unix file
variables that help us open the serial port.

*/
package ar3

import (
	"fmt"
	"math"
	"os"
	"time"
	"unsafe"

	"golang.org/x/sys/unix"
)

// Converts degrees to radians
var DEG float64 = math.Pi / 180

// AR3 is the generic interface for interacting with an AR3 robotic arm.
type AR3 interface {
	CurrentPosition() (int, int, int, int, int, int, int)
	Echo() error
	Calibrate(speed int, j1, j2, j3, j4, j5, j6, tr bool) error
	MoveSteppers(speed, accdur, accspd, dccdur, dccspd, j1, j2, j3,
		j4, j5, j6, tr int) error
	SetDirections(bool, bool, bool, bool, bool, bool, bool)
	GetDirections() (bool, bool, bool, bool, bool, bool, bool)
}

// The following StepLims are hard-coded in the ARbot.cal file for the stepper
// motors. These should not change.
var j1stepLim int = 15200
var j2stepLim int = 7300
var j3stepLim int = 7850
var j4stepLim int = 15200
var j5stepLim int = 4575
var j6stepLim int = 6625

// The following RadSteps (radians per step) are calculated from the AR3 stepper
// motors and gearing to be exact values for converting steps to joint angles
var j1RadStep float64 = 0.0225 * DEG
var j2RadStep float64 = 0.018 * DEG
var j3RadStep float64 = 0.018 * DEG
var j4RadStep float64 = 0.01092214664 * DEG
var j5RadStep float64 = 0.04723477289 * DEG
var j6RadStep float64 = 0.02343358396 * DEG
var trMmStep float64 = 0.0 // This is for a linear rail (mm/step)

// AR3exec struct represents an AR3 robotic arm connected to a serial port.
type AR3exec struct {
	serial *os.File

	jointVals [7]int
	calibDirs [7]bool
	jointDirs [7]bool
}

// Read values off serial into a buffer
func (ar3 *AR3exec) ReadBuffer() error {
	// Read output of echo
	buf := make([]byte, 128)
	_, err := ar3.serial.Read(buf)
	if err != nil {
		return err
	}
	return nil
}

// Discards data written to the port but not transmitted,
// or data received but not read
func (ar3 *AR3exec) ClearBuffer() error {
	const TCFLSH = 0x540B
	_, _, errno := unix.Syscall(
		unix.SYS_IOCTL,
		uintptr(ar3.serial.Fd()),
		uintptr(TCFLSH),
		uintptr(unix.TCIOFLUSH),
	)

	if errno == 0 {
		return nil
	}
	return errno
}

// Connect connects to the AR3 over serial.
func Connect(serialConnectionStr string,
	j1dir, j2dir, j3dir, j4dir, j5dir, j6dir, trdir,
	j1calibdir, j2calibdir, j3calibdir, j4calibdir, j5calibdir, j6calibdir,
	trcalibdir bool) (*AR3exec, error) {

	// Set up connection to the serial port
	f, err := os.OpenFile(serialConnectionStr, unix.O_RDWR|unix.O_NOCTTY|unix.O_NONBLOCK, 0666)
	if err != nil {
		return &AR3exec{}, err
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
		return &AR3exec{}, err
	}
	time.Sleep(time.Millisecond * 1000)

	newCalibDirs := [7]bool{j1calibdir, j2calibdir, j3calibdir, j4calibdir, j5calibdir, j6calibdir,
		trcalibdir}
	newJointDirs := [7]bool{j1dir, j2dir, j3dir, j4dir, j5dir, j6dir, trdir}
	// Instantiate a new AR3 object that holds our serial port. Additionally,
	// set default stepLims, which are hard-coded in the AR3 software
	newAR3 := AR3exec{serial: f, jointDirs: newJointDirs, calibDirs: newCalibDirs}

	err = newAR3.ClearBuffer()
	if err != nil {
		return &newAR3, err
	}

	// Test to see if we can connect to the newAR3
	err = newAR3.Echo()
	if err != nil {
		return &newAR3, err
	}

	// If we can echo, return newAR3 object
	return &newAR3, nil
}

// Echo tests an echo command on the AR3. Useful for testing connectivity to
// the AR3.
func (ar3 *AR3exec) Echo() error {
	// Send echo to the device
	str := "Test"
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
		return fmt.Errorf("return from echo is empty")
	}

	// See if we had the same bytes returned
	// Note: the serial returns with your string with \n\r\n between two double-quotes ("), so we remove these characters
	stringOutput := fmt.Sprintf("%q", buf[:n])
	stringOutput = stringOutput[1 : len(stringOutput)-7]
	if stringOutput != str {
		return fmt.Errorf("failed echo to AR3. Expected %s but got %s", str, stringOutput)
	}

	// If we got the same string back, success
	return nil
}

// MoveSteppersRelative moves each of the AR3's stepper motors by a certain
// amount of steps. In addition to the j1,j2,j3,j4,j5,j6 positions, you can also
// define 5 other variables: ACCdur, ACCspd, DCCdur, and DCCspd (these are named
// DEC on ARCS but DCC on the arduino controller), which define the acceleration
// duration and speed of the stepper motors. Good defaults are:
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
// MoveSteppersRelative does not have ANY checks. Please double check the values
// getting fed to MoveSteppersRelative or else the robot WILL self destruct.
func (ar3 *AR3exec) MoveSteppersRelative(speed, accdur, accspd, dccdur, dccspd,
	j1, j2, j3, j4, j5, j6, tr int) error {
	// First, check if the move can be made
	to := []int{j1, j2, j3, j4, j5, j6}
	from := []int{ar3.jointVals[0], ar3.jointVals[1], ar3.jointVals[2],
		ar3.jointVals[3], ar3.jointVals[4], ar3.jointVals[5]}

	limits := []int{j1stepLim, j2stepLim, j3stepLim,
		j4stepLim, j5stepLim, j6stepLim}

	motor := []string{"J1", "J2", "J3", "J4", "J5", "J6"}
	var newPositions [7]int
	for i := 0; i < 6; i++ {
		newJ := to[i] + from[i]
		lowerLimit := 0
		upperLimit := limits[i]
		if !ar3.calibDirs[i] {
			lowerLimit = -upperLimit
			upperLimit = 0
		}
		if newJ < lowerLimit || newJ > upperLimit {
			return fmt.Errorf("%s out of range. Must be between %d and %d."+
				" Got %d", motor[i], lowerLimit, upperLimit, newJ)
		}
		newPositions[i] = newJ
	}
	// If all the limits check out, apply them.
	ar3.jointVals = newPositions

	// command string for movement is MJ
	command := "MJ"
	// First, compute direction. If the stepper is negative, that means that
	// direction is set to 1. We are going to compute these as a list, and then
	// append them to a growing string
	var jdirection int
	// The move string is assembled with the beginning of an alphabetical
	// character for each axis. These were derived from line 4493 in the ARCS
	// source file under the variable "commandCalc".
	alphabetForCommands := []string{"A", "B", "C", "D", "E", "F", "T"}

	// directions need to be set as well
	for i, j := range []int{j1, j2, j3, j4, j5, j6, tr} {
		jdirection = 0

		if j < 0 {
			jdirection = 1
			j = -1 * j
		}

		// We also have to compensate for the direction coded when initializing
		// the AR3 (as oftentimes, this can be off)
		if ar3.jointDirs[i] {
			tempDir := 0
			switch jdirection {
			case 1:
				tempDir = 0
			case 0:
				tempDir = 1
			}
			jdirection = tempDir
		}
		command = command + fmt.Sprintf("%s%d%d", alphabetForCommands[i], jdirection, j)
	}

	// We now have the axis commands, so we need to add the speed, accspd, accdur, dccdur, and dccspd.
	// These are also derived from the above commandCalc.
	command = command + fmt.Sprintf("S%dG%dH%dI%dK%d\n", speed, accspd, accdur, dccdur, dccspd)

	// Send command to AR3
	_, err := ar3.serial.Write([]byte(command))
	if err != nil {
		return err
	}

	err = ar3.ReadBuffer()
	if err != nil {
		return err
	}

	// This has to send and get a response to indicate the move is complete
	ar3.Echo()

	// Normally, we would check here for successful completion. However, there
	// IS no way to check for successful completion implemented in the AR3 code.
	// So we do not check for this.
	return nil
}

// MoveSteppersAbsolute moves each of the AR3's stepper motors to an absolute
// step position between 0 and the step limit for each joint. See
// MoveSteppersRelative for full documentation of arguments.
func (ar3 *AR3exec) MoveSteppersAbsolute(speed, accdur, accspd, dccdur, dccspd,
	j1, j2, j3, j4, j5, j6, tr int) error {
	js := ar3.jointVals
	return ar3.MoveSteppersRelative(speed, accdur, accspd, dccdur, dccspd,
		j1-js[0], j2-js[1], j3-js[2], j4-js[3], j5-js[4], j6-js[5],
		tr-js[6])
}

// MoveJointsAbsolute moves each of the AR3's joints to an absolute angle
// defined relative to the calibration position for each joint. Angles are
// definied in radians, unless deg is true (in which angles are degrees).
func (ar3 *AR3exec) MoveJointsAbsolute(speed, accdur, accspd, dccdur,
	dccspd int, j1, j2, j3, j4, j5, j6, tr float64, deg bool) error {

	var conv float64

	if deg {
		conv = DEG
	} else {
		conv = 1
	}

	jointSteps := []int{
		int(math.Round(conv * j1 / j1RadStep)),
		int(math.Round(conv * j2 / j2RadStep)),
		int(math.Round(conv * j3 / j3RadStep)),
		int(math.Round(conv * j4 / j4RadStep)),
		int(math.Round(conv * j5 / j5RadStep)),
		int(math.Round(conv * j6 / j6RadStep)),
		int(math.Round(conv * tr / trMmStep))}

	return ar3.MoveSteppersAbsolute(speed, accdur, accspd, dccdur, dccspd,
		jointSteps[0], jointSteps[1], jointSteps[2], jointSteps[3],
		jointSteps[4], jointSteps[5], jointSteps[6])
}

// Calibrate moves each of the AR3's stepper motors to their respective limit
// switch. A good default speed for this action is 50 (line 4659 on ARCS). Set
// the j1 -> j6 booleans "true" if that joint should be homed. Set the
// j1calibdir -> j6calibdir booleans "true" if the calibration direction should
// be in the negative axis direction.
func (ar3 *AR3exec) Calibrate(speed int, j1, j2, j3, j4, j5, j6, tr bool) error {
	// command string for home is LL
	command := "LL"

	// The home string is assembled with the beginning of an alphabetical
	// character for each axis. These were derived from line 4493 in the ARCS
	// source file under the variable "commandCalc".
	alphabetForCommands := []string{"A", "B", "C", "D", "E", "F", "T"}
	jmotors := []int{j1stepLim, j2stepLim, j3stepLim, j4stepLim, j5stepLim, j6stepLim, 0}

	homeMotor := []bool{j1, j2, j3, j4, j5, j6, tr}
	for i := range ar3.jointDirs {
		// First, we check if we need to home the motor. If we do not (false),
		// do not home the motor.
		if homeMotor[i] {
			// Each direction is set by the boolean and appended into the
			// calibrate string. The number of steps taken is equivalent to the
			// step limits, which are hardcoded into the AR3 arm.
			dirBit := !(ar3.jointDirs[i] != ar3.calibDirs[i])
			if dirBit {
				command = command + fmt.Sprintf("%s%d%d", alphabetForCommands[i], 0, jmotors[i])
			} else {
				command = command + fmt.Sprintf("%s%d%d", alphabetForCommands[i], 1, jmotors[i])
			}

		} else {
			command = command + fmt.Sprintf("%s%d%d", alphabetForCommands[i], 0, 0)
		}
	}
	// Finally, we append the speed.
	command = command + fmt.Sprintf("S%d\n", speed)

	// Send command to AR3
	_, err := ar3.serial.Write([]byte(command))

	if err != nil {
		return err
	}

	err = ar3.ReadBuffer()
	if err != nil {
		return err
	}

	// Normally, we would check here for successful completion. However, there IS no way to check for
	// successful completion implemented in the AR3 code. So we do not check for this.
	return nil
}

// CurrentStepperPosition returns the current position of the AR3 arm as stepper
// motor steps from 0 for each axis.
func (ar3 *AR3exec) CurrentStepperPosition() (int, int, int, int, int, int, int) {
	vals := ar3.jointVals
	return vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6]
}

// SetDirections sets the directions of the AR3 arm.
func (ar3 *AR3exec) SetDirections(j1dir, j2dir, j3dir, j4dir, j5dir, j6dir, trdir bool) {
	ar3.jointDirs = [7]bool{j1dir, j2dir, j3dir, j4dir, j5dir, j6dir, trdir}
}

// GetDirections gets the directions of the AR3 arm.
func (ar3 *AR3exec) GetDirections() (bool, bool, bool, bool, bool, bool, bool) {
	dirs := ar3.jointDirs
	return dirs[0], dirs[1], dirs[2], dirs[3], dirs[4], dirs[5], dirs[6]
}

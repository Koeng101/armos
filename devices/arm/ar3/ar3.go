package ar3

import (
	"fmt"

	"github.com/tarm/serial"
)

/******************************************************************************

				AR3 robotic control

This is a Golang library implementing serial commands for the AR3 robotic arm.

The AR3 robotic arm is a low cost + open source 6 axis robotic arm designed
and created by Chris Annin of Annin Robotics. Annin Robotics provides software
[1] for operating the robotic arm, but this software is designed for windows
and not useful for devices for like raspberry pis.

This Golang library takes over for the local robotic control software by
implementing the functions needed to communicate with the arduino on the AR3 or
AR2 robot. Specifically, we implement the functions for:

- Echo
- MoveJ

We do not yet support encoders in the AR3, nor any other commands. All routines
directly on the robot, such as waypoints or calibrate, can be reproduced with
MoveJ and Echo.

1. https://www.anninrobotics.com/downloads

******************************************************************************/

// Stepper is a representation of a stepper motor on the robot.
type Stepper struct {
	currentStep          int
	stepLim              int
	ClockwiseCalibration bool
}

// AR3 struct represents an AR3 robotic arm connected to a serial port.
type AR3 struct {
	serialPort *serial.Port
	j1         Stepper
	j2         Stepper
	j3         Stepper
	j4         Stepper
	j5         Stepper
	j6         Stepper
}

// The following StepLims are hard coded in the ARbot.cal file for the stepper
// motors. They are used in calibration protocols.
var j1stepLim int = 15200
var j2stepLim int = 7300
var j3stepLim int = 7850
var j4stepLim int = 15200
var j5stepLim int = 4575
var j6stepLim int = 6625

// The following ClockwiseCalibrations are hard coded in the ARbot.cal file.
// The format is "001001" which are then translated into j1,j2,j3,j4,j5,j6, but
// we will simply hard code them here. ClockwiseCalibration is used within the
// calibration protocols.
var j1ClockwiseCalibration bool = false
var j2ClockwiseCalibration bool = false
var j3ClockwiseCalibration bool = true
var j4ClockwiseCalibration bool = false
var j5ClockwiseCalibration bool = false
var j6ClockwiseCalibration bool = true

// Connect connects to the AR3 over serial. Baud is set to 115200 as this is
// the default used by Annin Robotics.
func Connect(serialConnectionStr string) (AR3, error) {
	// Set up connection to the serial port
	c := &serial.Config{Name: serialConnectionStr, Baud: 115200}
	serialPort, err := serial.OpenPort(c)
	if err != nil {
		return AR3{}, err
	}

	// Instantiate a new AR3 object that holds our serial port
	newAR3 := AR3{serialPort: serialPort}

	// Set default stepLims. These are hard coded in the AR3 software.
	AR3.j1.stepLim = j1stepLim
	AR3.j2.stepLim = j2stepLim
	AR3.j3.stepLim = j3stepLim
	AR3.j4.stepLim = j4stepLim
	AR3.j5.stepLim = j5stepLim
	AR3.j6.stepLim = j6stepLim

	// Set default ClockwiseCalibration. These can be changed by the user
	// if necessary.
	AR3.j1.ClockwiseCalibration = j1ClockwiseCalibration
	AR3.j2.ClockwiseCalibration = j2ClockwiseCalibration
	AR3.j3.ClockwiseCalibration = j3ClockwiseCalibration
	AR3.j4.ClockwiseCalibration = j4ClockwiseCalibration
	AR3.j5.ClockwiseCalibration = j5ClockwiseCalibration
	AR3.j6.ClockwiseCalibration = j6ClockwiseCalibration

	// Test to see if we can connect to the newAR3
	err = newAR3.Echo("Test")
	if err != nil {
		return newAR3, err
	}

	// If we can echo, return newAR3 object
	return newAR3, nil
}

// Echo echos back a string sent to the AR3. This uses the "Echo" function on
// the AR3 arduino.
func (ar3 *AR3) Echo(str string) error {
	// Send echo to the device
	stringToSend := fmt.Sprintf("TM%s\n", str)
	_, err := ar3.serialPort.Write([]byte(stringToSend))
	if err != nil {
		return err
	}

	// Read output of echo
	buf := make([]byte, 128)
	n, err := ar3.serialPort.Read(buf)
	if err != nil {
		return err
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

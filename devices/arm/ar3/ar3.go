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
- Calibrate
- MoveJ

We do not yet support encoders in the AR3.

1. https://www.anninrobotics.com/downloads

******************************************************************************/

// AR3 struct represents an AR3 robotic arm connected to a serial port.
type AR3 struct {
	serialPort *serial.Port
	j1cur      int
	j2cur      int
	j3cur      int
	j4cur      int
	j5cur      int
	j6cur      int
}

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

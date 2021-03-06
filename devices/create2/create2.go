/*
Package create2 is a Golang library implementing serial commands for the iRobot
Create2 robot.

Basics

The iRobot Create2 is a low cost mobile robot based on the Roomba 600 series
robot repurposed for education and hacking. iRobot provides a well documented
serial interface for controlling the Create2.

*/
package create2

import (
	"fmt"
	"golang.org/x/sys/unix"
	"os"
	"time"
	"unsafe"
)

type Create2 interface {
	Reset() error
	Safe() error
	Full() error
	SeekDock() error
	DrivePwn(int, int) error
	GetSensorData() (SensorData, error)
}

type Create2exec struct {
	serial *os.File
	S      *os.File
}

func Connect(serialConnectionStr string) (*Create2exec, error) {
	// Set up connection to the serial port
	f, err := os.OpenFile(serialConnectionStr, unix.O_RDWR|unix.O_NOCTTY|unix.O_NONBLOCK, 0666)
	if err != nil {
		return &Create2exec{}, err
	}
	rate := uint32(unix.B115200) // 115200 is the default Baud rate of the Create 2 arm
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
		return &Create2exec{}, err
	}

	newCreate2 := Create2exec{serial: f, S: f}

	// Start the create2
	var start byte = 128
	_, err = newCreate2.serial.Write([]byte{start})
	if err != nil {
		return &newCreate2, err
	}

	// Return the Create2 object
	return &newCreate2, nil
}

func (create2 *Create2exec) Reset() error {
	// First we reset the bot
	var command byte = 7
	_, err := create2.serial.Write([]byte{command})
	if err != nil {
		return err
	}
	// After we reset, we have to start the interface again
	var start byte = 128
	_, err = create2.serial.Write([]byte{start})
	if err != nil {
		return err
	}
	return nil
}

func (create2 *Create2exec) Safe() error {
	var command byte = 131
	_, err := create2.serial.Write([]byte{command})
	if err != nil {
		return err
	}
	// Requires a sleep before commands are issued
	time.Sleep(1 * time.Second)
	return nil
}

func (create2 *Create2exec) Full() error {
	var command byte = 132
	_, err := create2.serial.Write([]byte{command})
	if err != nil {
		return err
	}
	// Requires a sleep before commands are issued
	time.Sleep(1 * time.Second)
	return nil
}

func (create2 *Create2exec) SeekDock() error {
	var command byte = 143
	_, err := create2.serial.Write([]byte{command})
	if err != nil {
		return err
	}
	return nil
}

func (create2 *Create2exec) DrivePwm(right, left int) error {
	// First, check if the values are within tolerable range.
	if right > 255 || right < -255 {
		return fmt.Errorf("Right drive values must be between 500 and -500. Got: %d", right)
	}
	if left > 255 || left < -255 {
		return fmt.Errorf("Left drive values must be between 500 and -500. Got: %d", left)
	}

	// Append into a command
	var opcode byte = 146
	command := []byte{opcode}

	var rightInt16 int16 = int16(right)
	var rightH, rightL uint8 = uint8(rightInt16 >> 8), uint8(rightInt16 & 0xff)
	command = append(command, rightH)
	command = append(command, rightL)

	var lefInt16 int16 = int16(left)
	var leftH, leftL uint8 = uint8(lefInt16 >> 8), uint8(lefInt16 & 0xff)
	command = append(command, leftH)
	command = append(command, leftL)

	_, err := create2.serial.Write(command)
	if err != nil {
		return err
	}
	return nil
}

func (create2 *Create2exec) GetSensors() (SensorData, error) {
	var command byte = 142

	_, err := create2.serial.Write([]byte{command, 100})
	if err != nil {
		return SensorData{}, err
	}
	// wait 15ms before update
	var sensorBytes = make([]byte, 80)
	time.Sleep(15 * time.Millisecond)
	_, err = create2.serial.Read(sensorBytes)
	if err != nil {
		return SensorData{}, err
	}
	sensorData, err := decodeFullPacket(sensorBytes)
	if err != nil {
		return sensorData, err
	}
	return sensorData, nil
}

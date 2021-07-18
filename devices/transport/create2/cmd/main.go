package main

import (
	"github.com/koeng101/armos/devices/transport/create2"
	"time"
)

func main() {
	var robot create2.Create2
	robot, _ = create2.Connect("/dev/ttyUSB0")

	// Set robot into safe mode
	_ = robot.Safe()

	// Drive for 5 seconds backwards
	_ = robot.DriveDirect(-100, -100)
	time.Sleep(5 * time.Second)

	// Seek dock
	_ = robot.SeekDock()
}

package main

import (
	"time"
	"github.com/koeng101/armos/devices/transport/create2"
)

func main() {
	//var robot create2.Create2
	robot, _ := create2.Connect("/dev/serial0")

	// Set robot into safe mode
	time.Sleep(1 * time.Second)
	_ = robot.Safe()
	time.Sleep(1 * time.Second)

	// Drive for 5 seconds backwards
	_ = robot.DrivePwm(-100, -100)
	time.Sleep(6 * time.Second)

	// Spin around
	_ = robot.DrivePwm(-100, 100)
	time.Sleep(950 *time.Millisecond)

	_ = robot.DrivePwm(100,100)
	time.Sleep(7 * time.Second)

	_ = robot.DrivePwm(100, -100)
	time.Sleep(950 *time.Millisecond)

	_ = robot.DrivePwm(100,100)
	time.Sleep(1200 *time.Millisecond)

	// REVERSE
	_ = robot.DrivePwm(0,0)
	time.Sleep(20 * time.Second)

	_ = robot.DrivePwm(-100,-100)
	time.Sleep(1200*time.Millisecond)

	_ = robot.DrivePwm(-100, 100)
	time.Sleep(950 *time.Millisecond)

	_ = robot.DrivePwm(-100,-100)
	time.Sleep(7500 * time.Millisecond)

	_ = robot.DrivePwm(100, -100)
        time.Sleep(950 *time.Millisecond)

	_ = robot.DrivePwm(100, 100)
        time.Sleep(4000 * time.Millisecond)
	_ = robot.DrivePwm(0,0)
	_ = robot.SeekDock()

}

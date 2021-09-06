package main

import (
	"fmt"

	"github.com/koeng101/armos/devices/ar3"
)

func main() {
	robot, err := ar3.Connect("/dev/ttyUSB0", false, false, true, false, false, true, false)
	if err != nil {
		fmt.Printf("%s\n", err)
	}

	robot.MoveSteppersRelative(5, 10, 10, 10, 10, 100, 100, 100, 100, 100, 100, 0)
	robot.Calibrate(25, true, true, true, true, true, true, false)
	fmt.Println("Calibration Complete")

	err = robot.MoveJointsAbsolute(5, 10, 10, 10, 10, 170, 85, 60, 85, 90, 80, 0, true)
	fmt.Println("Move 1 Complete")
	if err != nil {
		fmt.Printf("%s\n", err)
	}
	// ar3.Connect("/dev/ttyUSB0", false, false, true, false, false, true, false)

	// err = robot.MoveSteppersAbsolute(5, 10, 10, 10, 10, 500, 1000, 1000, 1000, 1000, 1000, 0)
	// fmt.Println("Move 2 Complete")
	// if err != nil {
	// 	fmt.Printf("%s\n", err)
	// }
	// robot.MoveSteppersRelative(5, 0, 10, 0, 10, -100, -200, 0, 0, 0, 0, 0)

	// _ = robot.MoveSteppers(10, 15, 10, 20, 5, 100, 100, 0, 0, 0, 0, 0)

	// _ = robot.MoveSteppers(10, 15, 10, 20, 5, -100, -100, 0, 0, 0, 0, 0)

	// _ = robot.MoveSteppers(25,15,10,20,5,800,4500,0,0,1000,0,0)

	// _ = robot.MoveSteppers(25,15,10,20,5,0,0,0,0,0,0,0)

	// _ = robot.MoveSteppers(25,15,10,20,5,-800,-4500,0,0,-1000,0,0)
}

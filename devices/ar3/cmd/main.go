package main

import (
	"fmt"

	"github.com/koeng101/armos/devices/ar3"
)

func main() {
	jointDirs := [7]bool{true, false, false, true, false, true, false}
	calibDirs := [7]bool{false, true, false, false, true, true, false}
	limitSwitchSteps := ar3.AnglesToSteps([7]float64{-170, 85, -60, -85, 90, 80, 0}, true)
	robot, err := ar3.Connect("/dev/ttyUSB0", jointDirs, calibDirs, limitSwitchSteps)
	if err != nil {
		fmt.Printf("%s\n", err)
	}

	robot.Calibrate(25, true, true, true, true, true, true, false)

	err = robot.MoveJointsAbsolute(5, 10, 10, 10, 10, 0, 0, 0, 0, 0, 0, 0, true)
	if err != nil {
		fmt.Printf("%s\n", err)
	}
	currPose := robot.CurrentPose()
	err = robot.MovePose(5, 10, 10, 10, 10, currPose)
	if err != nil {
		fmt.Printf("%s\n", err)
	}
	// Move down 50 mm to avoid singularities during square
	targPose := currPose
	targPose.Pos.Z -= 50
	err = robot.MovePose(5, 10, 10, 10, 10, targPose)
	if err != nil {
		fmt.Printf("%s\n", err)
	}

	// Now make a 100 mm square
	sideLength := 100.0
	targPose.Pos.Z -= sideLength
	err = robot.MovePose(5, 10, 10, 10, 10, targPose)
	if err != nil {
		fmt.Printf("%s\n", err)
	}
	targPose.Pos.Y -= sideLength
	err = robot.MovePose(5, 10, 10, 10, 10, targPose)
	if err != nil {
		fmt.Printf("%s\n", err)
	}
	targPose.Pos.Z += sideLength
	err = robot.MovePose(5, 10, 10, 10, 10, targPose)
	if err != nil {
		fmt.Printf("%s\n", err)
	}
	targPose.Pos.Y += sideLength
	err = robot.MovePose(5, 10, 10, 10, 10, targPose)
	if err != nil {
		fmt.Printf("%s\n", err)
	}
}

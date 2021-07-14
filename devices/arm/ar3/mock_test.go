package ar3

import (
	"fmt"
)

func ExampleConnectMock() {
	var arm AR3
	arm = ConnectMock()
	if arm.Echo() == nil {
		fmt.Println("Connected")
	}
	// Output: Connected
}

func ExampleAR3simulate_Echo() {
	var arm AR3
	arm = ConnectMock()
	err := arm.Echo()
	if err == nil {
		fmt.Print("Connected")
	}
	// Output: Connected
}

func ExampleAR3simulate_MoveSteppers() {
	var arm AR3
	arm = ConnectMock()
	// Move the arm. First 5 numbers are rational defaults, and each motor gets moved 500 steps
	err := arm.MoveSteppers(25, 15, 10, 20, 5, 500, 500, 500, 500, 500, 500, 0)
	if err == nil {
		fmt.Println("Moved")
	}
	// Output: Moved
}

func ExampleAR3simulate_Home() {
	var arm AR3
	arm = ConnectMock()
	// Home the arm. 50 is a good default speed.
	err := arm.Home(50)
	if err == nil {
		fmt.Println("Homed")
	}
	// Output: Homed
}

func ExampleAR3simulate_CurrentPosition() {
	var arm AR3
	arm = ConnectMock()
	// Current position. By default, the arm is assumed to be homed at 0
	j1, _, _, _, _, _, _ := arm.CurrentPosition()
	if j1 == 0 {
		fmt.Println("At 0")
	}
	// Output: At 0
}

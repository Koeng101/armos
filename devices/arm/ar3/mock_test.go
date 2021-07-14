package ar3

import (
	"fmt"
)

func ExampleConnectMock() {
	AR3 := ConnectMock()
	if AR3.Echo() == nil {
		fmt.Println("Connected")
	}
	// Output: Connected
}

func ExampleAR3simulate_Echo() {
	AR3 := ConnectMock()
	err := AR3.Echo()
	if err == nil {
		fmt.Print("Connected")
	}
	// Output: Connected
}

func ExampleAR3simulate_MoveSteppers() {
	AR3 := ConnectMock()
	// Move the arm. First 5 numbers are rational defaults, and each motor gets moved 500 steps
	err := AR3.MoveSteppers(25, 15, 10, 20, 5, 500, 500, 500, 500, 500, 500, 0)
	if err == nil {
		fmt.Println("Moved")
	}
	// Output: Moved
}

func ExampleAR3simulate_Home() {
	AR3 := ConnectMock()
	// Home the arm. 50 is a good default speed.
	err := AR3.Home(50)
	if err == nil {
		fmt.Println("Homed")
	}
	// Output: Homed
}

func ExampleAR3simulate_CurrentPosition() {
	AR3 := ConnectMock()
	// Current position. By default, the arm is assumed to be homed at 0
	j1, _, _, _, _, _, _ := AR3.CurrentPosition()
	if j1 == 0 {
		fmt.Println("At 0")
	}
	// Output: At 0
}

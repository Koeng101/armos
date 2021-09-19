package ar3

import (
	"fmt"
	"testing"
)

// TestMockArmInterface checks that the AR3simulate struct implements the Arm
// interface
func TestMockArmInterface(t *testing.T) {
	val := AR3simulate{}
	_, ok := interface{}(&val).(Arm)
	if !ok {
		t.Errorf("Failed")
	}
}

func TestAR3simulate_MoveSteppers(t *testing.T) {
	// The following line establishes that mock DOES implement the AR3 interface.
	var arm Arm //nolint
	arm = ConnectMock()
	err := arm.MoveSteppers(25, 15, 10, 20, 5, 500, 500, 500, 500, 500, 500000000, 0)
	if err == nil {
		t.Errorf("Arm should have failed with large j6 value")
	}
}

func ExampleConnectMock() {
	arm := ConnectMock()
	if arm.Echo() == nil {
		fmt.Println("Connected")
	}
	// Output: Connected
}

func ExampleAR3simulate_Echo() {
	arm := ConnectMock()
	err := arm.Echo()
	if err == nil {
		fmt.Print("Connected")
	}
	// Output: Connected
}

func ExampleAR3simulate_MoveSteppers() {
	arm := ConnectMock()
	// Move the arm. First 5 numbers are rational defaults, and each motor gets moved 500 steps
	err := arm.MoveSteppers(25, 15, 10, 20, 5, 500, 500, 500, 500, 500, 500, 0)
	if err == nil {
		fmt.Println("Moved")
	}
	// Output: Moved
}

func ExampleAR3simulate_Calibrate() {
	arm := ConnectMock()
	// Calibrate the arm. 50 is a good default speed.
	err := arm.Calibrate(50, true, true, true, true, true, true, true)
	if err == nil {
		fmt.Println("Calibrated")
	}
	// Output: Calibrated
}

func ExampleAR3simulate_CurrentPosition() {
	arm := ConnectMock()
	// Current position. By default, the arm is assumed to be homed at 0
	j1, _, _, _, _, _, _ := arm.CurrentPosition()
	if j1 == 0 {
		fmt.Println("At 0")
	}
	// Output: At 0
}

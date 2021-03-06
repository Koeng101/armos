package ar3

import (
	"fmt"
)

// AR3simulate struct represents an AR3 robotic arm interface for testing purposes.
type AR3simulate struct {
	j1    int
	j2    int
	j3    int
	j4    int
	j5    int
	j6    int
	tr    int
	j1dir bool
	j2dir bool
	j3dir bool
	j4dir bool
	j5dir bool
	j6dir bool
	trdir bool
}

// ConnectMock connects to a mock AR3simulate interface.
func ConnectMock() *AR3simulate {
	return &AR3simulate{}
}

// Echo simulates AR3exec.Echo().
func (ar3 *AR3simulate) Echo() error {
	return nil
}

// MoveSteppers simulates AR3exec.MoveSteppers().
func (ar3 *AR3simulate) MoveSteppers(speed, accdur, accspd, dccdur, dccspd, j1, j2, j3, j4, j5, j6, tr int) error {
	// First, check if the move can be made
	to := []int{j1, j2, j3, j4, j5, j6}
	from := []int{ar3.j1, ar3.j2, ar3.j3, ar3.j4, ar3.j5, ar3.j6}
	limits := []int{j1stepLim, j2stepLim, j3stepLim, j4stepLim, j5stepLim, j6stepLim}
	motor := []string{"J1", "J2", "J3", "J4", "J5", "J6"}
	var newPositions []int
	for i := 0; i < 6; i++ {
		newJ := to[i] + from[i]
		if newJ < 0 || newJ > limits[i] {
			return fmt.Errorf("%s out of range. Must be between 0 and %d. Got %d", motor[i], limits[i], newJ)
		}
		newPositions = append(newPositions, newJ)
	}
	// If all the limits check out, apply them.
	ar3.j1 = newPositions[0]
	ar3.j2 = newPositions[1]
	ar3.j3 = newPositions[2]
	ar3.j4 = newPositions[3]
	ar3.j5 = newPositions[4]
	ar3.j6 = newPositions[5]

	// Since we are simulating, simply update and assume that there is no error.
	return nil
}

// Calibrate simulates AR3exec.Calibrate()
func (ar3 *AR3simulate) Calibrate(speed int, j1, j2, j3, j4, j5, j6, tr bool) error {
	if j1 {
		ar3.j1 = 0
	}
	if j2 {
		ar3.j2 = 0
	}
	if j3 {
		ar3.j3 = 0
	}
	if j4 {
		ar3.j4 = 0
	}
	if j5 {
		ar3.j5 = 0
	}
	if j6 {
		ar3.j6 = 0
	}
	if tr {
		ar3.tr = 0
	}
	return nil
}

// CurrentPosition simulates AR3exec.CurrentPosition().
func (ar3 *AR3simulate) CurrentPosition() (int, int, int, int, int, int, int) {
	return ar3.j1, ar3.j2, ar3.j3, ar3.j4, ar3.j5, ar3.j6, ar3.tr
}

// SetDirections simulates AR3exec.SetDirections().
func (ar3 *AR3simulate) SetDirections(j1dir, j2dir, j3dir, j4dir, j5dir, j6dir, trdir bool) {
	ar3.j1dir = j1dir
	ar3.j2dir = j2dir
	ar3.j3dir = j3dir
	ar3.j4dir = j4dir
	ar3.j5dir = j5dir
	ar3.j6dir = j6dir
	ar3.trdir = trdir
}

// GetDirections simulates AR3exec.GetDirections().
func (ar3 *AR3simulate) GetDirections() (bool, bool, bool, bool, bool, bool, bool) {
	return ar3.j1dir, ar3.j2dir, ar3.j3dir, ar3.j4dir, ar3.j5dir, ar3.j6dir, ar3.trdir
}

package ar3

import (
	"fmt"
	//"gonum.org/v1/gonum/mat"
	"testing"
)

func TestGetTMats(t *testing.T) {
	testThetas := StepperTheta{10, 1, 1, 0, 0, 0}
	xyzwxyz := ForwardKinematics(testThetas, AR3DhParameters)
}

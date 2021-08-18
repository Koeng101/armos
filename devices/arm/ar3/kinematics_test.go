package ar3

import (
	"gonum.org/v1/gonum/mat"
	"testing"
)

func TestGetTMats(t *testing.T) {
	testThetas := StepperTheta{10, 1, 1, 0, 0, 0}
	_ = ForwardKinematics(testThetas, AR3DhParameters)
}

func TestMatrixToQuaterian(t *testing.T) {
	var qw float64
	var qx float64
	var qy float64
	var qz float64

	// Test tr > 0
	accumulatortMat1 := mat.NewDense(4, 4, []float64{1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1,
	})
	qw, qx, qy, qz = matrixToQuaterian(accumulatortMat1)
	switch {
	case qw != 1:
		t.Errorf("Failed mat1 with qw = %f", qw)
	case qx != 0:
		t.Errorf("Failed mat1 with qx = %f", qx)
	case qy != 0:
		t.Errorf("Failed mat1 with qy = %f", qy)
	case qz != 0:
		t.Errorf("Failed mat1 with qz = %f", qz)
	}

	// Test (accumulatortMat.At(0, 0) > accumulatortMat.At(1, 1)) && (accumulatortMat.At(0, 0) > accumulatortMat.At(2, 2))
	accumulatortMat2 := mat.NewDense(4, 4, []float64{1, 0, 0, 0,
		0, -1, 0, 0,
		0, 0, -1, 0,
		0, 0, 0, 0,
	})
	qw, qx, qy, qz = matrixToQuaterian(accumulatortMat2)
	switch {
	case qw != 0:
		t.Errorf("Failed mat2 with qw = %f", qw)
	case qx != 1:
		t.Errorf("Failed mat2 with qx = %f", qx)
	case qy != 0:
		t.Errorf("Failed mat2 with qy = %f", qy)
	case qz != 0:
		t.Errorf("Failed mat2 with qz = %f", qz)
	}

	// Test accumulatortMat.At(1, 1) > accumulatortMat.At(2, 2)
	accumulatortMat3 := mat.NewDense(4, 4, []float64{-1, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, -2, 0,
		0, 0, 0, 1,
	})
	qw, qx, qy, qz = matrixToQuaterian(accumulatortMat3)
	switch {
	case qw != 0:
		t.Errorf("Failed mat3 with qw = %f", qw)
	case qx != 0:
		t.Errorf("Failed mat3 with qx = %f", qx)
	case qy != 1:
		t.Errorf("Failed mat3 with qy = %f", qy)
	case qz != 0:
		t.Errorf("Failed mat3 with qz = %f", qz)
	}

	// Test default
	accumulatortMat4 := mat.NewDense(4, 4, []float64{-1, 0, 0, 0,
		0, -2, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 1,
	})
	qw, qx, qy, qz = matrixToQuaterian(accumulatortMat4)
	switch {
	case qw != 0:
		t.Errorf("Failed mat4 with qw = %f", qw)
	case qx != 0:
		t.Errorf("Failed mat4 with qx = %f", qx)
	case qy != 0:
		t.Errorf("Failed mat4 with qy = %f", qy)
	case qz != 1:
		t.Errorf("Failed mat4 with qz = %f", qz)
	}
}

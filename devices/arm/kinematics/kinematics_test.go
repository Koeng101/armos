package kinematics

import (
	"gonum.org/v1/gonum/mat"
	"testing"
)

func TestForwardKinematics(t *testing.T) {
	testThetas := StepperTheta{10, 1, 1, 0, 0, 0}
	f := ForwardKinematics(testThetas, AR3DhParameters)
	switch {
	case f.X != -101.74590611879692:
		t.Errorf("Forward kinematics failed on f.X = %f", f.X)
	case f.Y != -65.96805988175777:
		t.Errorf("Forward kinematics failed on f.Y = %f", f.Y)
	case f.Z != -322.27756822304093:
		t.Errorf("Forward kinematics failed on f.Z = %f", f.Z)
	case f.Qx != 0.06040824945687102:
		t.Errorf("Forward kinematics failed on f.Qx = %f", f.Qx)
	case f.Qy != -0.20421099379003957:
		t.Errorf("Forward kinematics failed on f.Qy = %f", f.Qy)
	case f.Qz != 0.2771553334491873:
		t.Errorf("Forward kinematics failed on f.Qz = %f", f.Qz)
	case f.Qw != 0.9369277637862541:
		t.Errorf("Forward kinematics failed on f.Qw = %f", f.Qw)
	}
}

func TestInverseKinematics(t *testing.T) {
	thetasInit := StepperTheta{0, 0, 0, 0, 0, 0}
	desiredEndEffector := XyzXyzw{-101.74590611879692, -65.96805988175777, -322.27756822304093, 0.06040824945687102, -0.20421099379003957, 0.2771553334491873, 0.9369277637862541}
	_, f, err := InverseKinematics(thetasInit, desiredEndEffector, AR3DhParameters)
	if err != nil {
		t.Errorf("Inverse Kinematics failed with error: %s", err)
	}
	if f > 0.000001 {
		t.Errorf("Inverse kinematics should have a score < 1e-6. Got %f", f)
	}
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

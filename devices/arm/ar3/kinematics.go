package ar3

import (
	"gonum.org/v1/gonum/mat"
	"math"
)

// Denavit-Hartenberg Parameters
type DhParameters struct {
	ThetaOffsets [6]float64
	AlphaValues  [6]float64
	AValues      [6]float64
	DValues      [6]float64
}

// Denavit-Hartenberg Parameters of AR3 provided by AR2 Version 2.0 software
// executable files from https://www.anninrobotics.com/downloads
// parameters are the same between the AR2 and AR3
var AR3DhParameters DhParameters = DhParameters{
	ThetaOffsets: [...]float64{0, 0, -math.Pi / 2, 0, 0, math.Pi},
	AlphaValues:  [...]float64{-(math.Pi / 2), 0, math.Pi / 2, -(math.Pi / 2), math.Pi / 2, 0},
	AValues:      [...]float64{64.2, 305, 0, 0, 0, 0},
	DValues:      [...]float64{169.77, 0, 0, -222.63, 0, -36.25},
}

type StepperTheta struct {
	J1 float64
	J2 float64
	J3 float64
	J4 float64
	J5 float64
	J6 float64
}

type XyzXyzw struct {
	X  float64
	Y  float64
	Z  float64
	Qx float64
	Qy float64
	Qz float64
	Qw float64
}

func ForwardKinematics(thetas StepperTheta, dhParameters DhParameters) XyzXyzw {
	// First, setup variables. We use 4 variables - theta, alpha, a and d to calculate a matrix
	// which is then multiplied to an accumulator matrix.
	thetaArray := []float64{thetas.J1, thetas.J2, thetas.J3, thetas.J4, thetas.J5, thetas.J6}
	var theta float64
	var alpha float64
	var a float64
	var d float64
	// Setup accumulator matrix - an identity matrix.
	accumulatortMat := mat.NewDense(4, 4, []float64{1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1,
	})
	// Iterate through each joint and built a new
	// matrix, multiplying it against the accumulator.
	for jointIdx := 0; jointIdx < 6; jointIdx++ {
		theta = thetaArray[jointIdx]
		theta = theta + dhParameters.ThetaOffsets[jointIdx]
		alpha = dhParameters.AlphaValues[jointIdx]
		a = dhParameters.AValues[jointIdx]
		d = dhParameters.DValues[jointIdx]
		tMat := mat.NewDense(4, 4, []float64{
			// First row
			math.Cos(theta),
			-math.Sin(theta) * math.Cos(alpha),
			math.Sin(theta) * math.Sin(alpha),
			a * math.Cos(theta),
			// Second row
			math.Sin(theta),
			math.Cos(theta) * math.Cos(alpha),
			-math.Cos(theta) * math.Sin(alpha),
			a * math.Sin(theta),
			// Third row
			0,
			math.Sin(alpha),
			math.Cos(alpha),
			d,
			// Forth row
			0,
			0,
			0,
			1,
		})
		// Multiply tMat against accumulatortMat
		x := mat.NewDense(4, 4, nil)
		x.Mul(accumulatortMat, tMat)
		accumulatortMat = x
	}

	// Now that we have the final accumulatorMatrix, lets figure out the euler angles.
	var output XyzXyzw
	output.X = accumulatortMat.At(0, 3)
	output.Y = accumulatortMat.At(1, 3)
	output.Z = accumulatortMat.At(2, 3)

	// http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
	var tr float64
	var s float64
	tr = accumulatortMat.At(0, 0) + accumulatortMat.At(1, 1) + accumulatortMat.At(2, 2)
	switch {
	case tr > 0:
		s = math.Sqrt(tr+1.0) * 2
		output.Qw = 0.25 * s
		output.Qx = (accumulatortMat.At(2, 1) - accumulatortMat.At(1, 2)) / s
		output.Qy = (accumulatortMat.At(0, 2) - accumulatortMat.At(2, 0)) / s
		output.Qz = (accumulatortMat.At(1, 0) - accumulatortMat.At(0, 1)) / s
	case (accumulatortMat.At(0, 0) > accumulatortMat.At(1, 1)) && (accumulatortMat.At(0, 0) > accumulatortMat.At(2, 2)):
		s = math.Sqrt(1.0+accumulatortMat.At(0, 0)-accumulatortMat.At(1, 1)-accumulatortMat.At(2, 2)) * 2
		output.Qw = (accumulatortMat.At(2, 1) - accumulatortMat.At(1, 2)) / s
		output.Qx = 0.25 * s
		output.Qy = (accumulatortMat.At(0, 1) + accumulatortMat.At(1, 0)) / s
		output.Qz = (accumulatortMat.At(0, 2) + accumulatortMat.At(2, 0)) / s
	case accumulatortMat.At(1, 1) > accumulatortMat.At(2, 2):
		s = math.Sqrt(1.0+accumulatortMat.At(1, 1)-accumulatortMat.At(0, 0)-accumulatortMat.At(2, 2)) * 2
		output.Qw = (accumulatortMat.At(0, 2) - accumulatortMat.At(2, 0)) / s
		output.Qx = (accumulatortMat.At(0, 1) + accumulatortMat.At(1, 0)) / s
		output.Qy = 0.25 * s
		output.Qz = (accumulatortMat.At(2, 1) + accumulatortMat.At(1, 2)) / s
	default:
		s = math.Sqrt(1.0+accumulatortMat.At(2, 2)-accumulatortMat.At(0, 0)-accumulatortMat.At(1, 1)) * 2
		output.Qw = (accumulatortMat.At(0, 1) - accumulatortMat.At(1, 0))
		output.Qx = (accumulatortMat.At(0, 2) + accumulatortMat.At(2, 0)) / s
		output.Qy = (accumulatortMat.At(2, 1) + accumulatortMat.At(1, 2)) / s
		output.Qz = 0.25 * s
	}
	return output
}

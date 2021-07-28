package ar3

import (
	"gonum.org/v1/gonum/mat"
	"math"
)

// Denavit-Hartenberg Parameters of AR3 provided by AR2 Version 2.0 software
// executable files from https://www.anninrobotics.com/downloads
// parameters are the same between the AR2 and AR3
var alphas = [...]float64{-(math.Pi / 2), 0, math.Pi / 2, -(math.Pi / 2), math.Pi / 2, 0}
var aVals = [...]float64{64.2, 305, 0, 0, 0, 0}
var dVals = [...]float64{169.77, 0, 0, -222.63, 0, -36.25}
var thetaOffsets = [...]float64{0, 0, -math.Pi / 2, 0, 0, math.Pi}

type StepperTheta struct {
	J1 float64
	J2 float64
	J3 float64
	J4 float64
	J5 float64
	J6 float64
}

type Xyzabc struct {
	X float64
	Y float64
	Z float64
	A float64
	B float64
	C float64
}

func ForwardKinematics(thetas StepperTheta) Xyzabc {
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
		theta = theta + thetaOffsets[jointIdx]
		alpha = alphas[jointIdx]
		a = aVals[jointIdx]
		d = dVals[jointIdx]
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
	var output Xyzabc
	output.X = accumulatortMat.At(0, 3)
	output.Y = accumulatortMat.At(1, 3)
	output.Z = accumulatortMat.At(2, 3)
	output.A = math.Atan2(math.Sqrt(math.Pow(accumulatortMat.At(0, 2), 2)+math.Pow(accumulatortMat.At(1, 2), 2)), -accumulatortMat.At(2, 2))
	output.B = math.Atan2(accumulatortMat.At(2, 0)/output.A, accumulatortMat.At(2, 1)/output.A)
	output.C = math.Atan2(accumulatortMat.At(0, 2)/output.A, accumulatortMat.At(1, 2)/output.A)
	return output
}

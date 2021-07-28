package ar3

import (
	"math"
)

// Denavit-Hartenberg Parameters of AR3 provided by AR2 Version 2.0 software
// executable files from https://www.anninrobotics.com/downloads 
// parameters are the same between the AR2 and AR3
var alphas = [...]float64{-(math.Pi/2), 0, math.Pi/2, -(math.Pi/2), math.Pi/2, 0}
var aVals = [...]float64{64.2, 305, 0, 0, 0, 0}
var dVals = [...]float64{169.77, 0, 0, -222.63, 0, -36.25}
var thetaOffsets = [...]float64{0, 0, -math.Pi/2, 0, 0, math.Pi}





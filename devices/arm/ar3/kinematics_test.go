package ar3

import (
	"testing"
)

func TestGetTMats(t *testing.T) {
	testThetas := StepperTheta{10, 1, 1, 0, 0, 0}
	_ = ForwardKinematics(testThetas)
	//fmt.Println(xyzabc)
	//fmt.Printf("%T\n", outputMatrix[0])
	//for i := 0; i < 6; i++ {
	//	fa := mat.Formatted(&outputMatrix[i], mat.Prefix("    "), mat.Squeeze())
	//	fmt.Printf("%d = %v\n\n", i, fa)
	//}
}

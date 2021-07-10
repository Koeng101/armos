package main

import (
	"fmt"

	"github.com/koeng101/armos/devices/arm/ar3"
)

func main() {
	fmt.Println("test")
	_, err := ar3.Connect("/dev/ttyUSB0")
	if err != nil {
		fmt.Println(err)
	}
	fmt.Println("Finished")
}

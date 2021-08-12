package main

import (
	"fmt"
	"github.com/koeng101/armos/devices/arm/ar3"
)

func main() {
	_, err := ar3.Connect("/dev/tty.usbserial110", false, false, false, false, false, false, false)
	if err != nil {
		fmt.Println("FAILED")
	}
}

package create2

import (
	"encoding/binary"
	"fmt"
)

type SensorData struct {
	BumpsAndWheelDrops       int // range: 0, 15
	Wall                     int // range: 0, 1
	CliffLeft                int // range: 0, 1
	CliffFrontLeft           int // range: 0, 1
	CliffFrontRight          int // range: 0, 1
	CliffRight               int // range: 0, 1
	VirtualWall              int // range: 0, 1
	WheelOvercurrents        int // range: 0, 31
	DirtDetect               int // range: 0, 255
	InfraredOmni             int // range: 0, 255
	Buttons                  int // range: 0, 255
	Distance                 int // range: -32768, 32767 mm
	Angle                    int // range: -32768, 32767
	ChargingState            int // range: 0, 5
	Voltage                  int // range: 0, 65535 mV
	Current                  int // range: -32768, 32767 mA
	Temperature              int // range: -128, 127 C
	BatteryCharge            int // range: 0, 65535 mAh
	BatteryCapacity          int // range: 0, 65535 mAh
	WallSignal               int // range: 0, 1023
	CliffLeftSignal          int // range: 0, 4095
	CliffFrontLeftSignal     int // range: 0, 4095
	CliffFrontRightSignal    int // range: 0, 4095
	CliffRightSignal         int // range: 0, 4095
	ChargingSourcesAvailable int // range: 0, 3
	OIMode                   int // range: 0, 3
	SongNumber               int // range: 0, 15
	SongPlaying              int // range: 0, 1
	NumberOfStreamPackets    int // range: 0, 108
	RequestedVelocity        int // range: -500, 500 mm
	RequestedRadius          int // range: -32768, 32767 mm
	RequestedRightVelocity   int // range: -500, 500 mm
	RequestedLeftVelocity    int // range: -500, 500 mm
	LeftEncoderCounts        int // range: -32768, 32767
	RightEncoderCounts       int // range: -32768, 32767
	LightBumper              int // 0, 127
	LightBumpLeft            int // range: 0, 4095
	LightBumpFrontLeft       int // range: 0, 4095
	LightBumpCenterLeft      int // range: 0, 4095
	LightBumpCenterRight     int // range: 0, 4095
	LightBumpFrontRight      int // range: 0, 4095
	LightBumpRight           int // range: 0, 4095
	InfraredCharacterLeft    int // range: 0, 255
	InfraredCharacterRight   int // range: 0, 255
	LeftMotorCurrent         int // range: -32768, 32767 mA
	RightMotorCurrent        int // range: -32768, 32767 mA
	MainBrushMotorCurrent    int // range: -32768, 32767 mA
	SideBrushMotorCurrent    int // range: -32768, 32767 mA
	Stasis                   int // range: 0, 3
}

func bytesToInt(buf []byte, signed bool) int {
	var i int
	if signed {
		num, _ := binary.Varint(buf)
		i = int(num)
	} else {
		num, _ := binary.Uvarint(buf)
		i = int(num)
	}
	return i
}

func byteToInt(b byte, signed bool) int {
	return bytesToInt([]byte{b}, signed)
}

func DecodeFullPacket(data []byte) (SensorData, error) {
	if len(data) != 80 {
		return SensorData{}, fmt.Errorf("DecodeFullPacket requires a packet of 80 bytes")
	}

	// Initialize a raw sensor data struct
	var s SensorData
	s.BumpsAndWheelDrops = byteToInt(data[0], false)
	s.Wall = byteToInt(data[1], false)
	s.CliffLeft = byteToInt(data[2], false)
	s.CliffFrontLeft = byteToInt(data[3], false)
	s.CliffFrontRight = byteToInt(data[4], false)
	s.CliffRight = byteToInt(data[5], false)
	s.VirtualWall = byteToInt(data[6], false)
	s.WheelOvercurrents = byteToInt(data[7], false)
	s.DirtDetect = byteToInt(data[8], false)
	// data[9] is unused
	s.InfraredOmni = byteToInt(data[10], false)
	s.Buttons = byteToInt(data[11], false)
	s.Distance = bytesToInt(data[12:14], true)
	s.Angle = bytesToInt(data[14:16], true)
	s.ChargingState = byteToInt(data[16], false)
	s.Voltage = bytesToInt(data[17:19], false)
	s.Current = bytesToInt(data[19:21], true)
	s.Temperature = byteToInt(data[21], true)
	s.BatteryCharge = bytesToInt(data[22:24], false)
	s.BatteryCapacity = bytesToInt(data[24:26], false)
	s.WallSignal = bytesToInt(data[26:28], false)
	s.CliffLeftSignal = bytesToInt(data[28:30], false)
	s.CliffFrontLeftSignal = bytesToInt(data[30:32], false)
	s.CliffFrontRightSignal = bytesToInt(data[32:34], false)
	s.CliffRightSignal = bytesToInt(data[34:36], false)
	// data[36:38] is unused
	s.ChargingSourcesAvailable = byteToInt(data[39], false)
	s.OIMode = byteToInt(data[40], false)
	s.SongNumber = byteToInt(data[41], false)
	s.SongPlaying = byteToInt(data[42], false)
	s.NumberOfStreamPackets = byteToInt(data[43], false)
	s.RequestedVelocity = bytesToInt(data[44:46], true)
	s.RequestedRadius = bytesToInt(data[46:48], true)
	s.RequestedRightVelocity = bytesToInt(data[48:50], true)
	s.RequestedLeftVelocity = bytesToInt(data[50:52], true)
	s.LeftEncoderCounts = bytesToInt(data[52:54], true)
	s.RightEncoderCounts = bytesToInt(data[54:56], true)
	s.LightBumper = byteToInt(data[56], false)
	s.LightBumpLeft = bytesToInt(data[57:59], false)
	s.LightBumpFrontLeft = bytesToInt(data[59:61], false)
	s.LightBumpCenterLeft = bytesToInt(data[61:63], false)
	s.LightBumpCenterRight = bytesToInt(data[63:65], false)
	s.LightBumpFrontRight = bytesToInt(data[65:67], false)
	s.LightBumpRight = bytesToInt(data[67:69], false)
	s.InfraredCharacterLeft = byteToInt(data[69], false)
	s.InfraredCharacterRight = byteToInt(data[70], false)
	s.LeftMotorCurrent = bytesToInt(data[71:73], true)
	s.RightMotorCurrent = bytesToInt(data[73:75], true)
	s.MainBrushMotorCurrent = bytesToInt(data[75:77], true)
	s.SideBrushMotorCurrent = bytesToInt(data[77:79], true)
	s.Stasis = byteToInt(data[79], false)

	return s, nil
}

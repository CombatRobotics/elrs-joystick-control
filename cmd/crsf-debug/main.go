package main

import (
	"flag"
	"fmt"
	"os"
	"time"

	"go.bug.st/serial"
)

func main() {
	portName := flag.String("port", "/dev/ttyUSB0", "serial port")
	baudRate := flag.Int("baud", 400000, "baud rate")
	duration := flag.Int("duration", 5, "test duration in seconds")
	flag.Parse()

	fmt.Printf("=== CRSF Debug Tool ===\n")
	fmt.Printf("Port: %s, Baud: %d, Duration: %ds\n\n", *portName, *baudRate, *duration)

	port, err := serial.Open(*portName, &serial.Mode{
		BaudRate: *baudRate,
		Parity:   serial.NoParity,
		DataBits: 8,
		StopBits: serial.OneStopBit,
	})
	if err != nil {
		fmt.Printf("ERROR opening port: %s\n", err)
		os.Exit(1)
	}
	defer port.Close()

	if err := port.SetReadTimeout(100 * time.Millisecond); err != nil {
		fmt.Printf("ERROR setting read timeout: %s\n", err)
		os.Exit(1)
	}

	fmt.Printf("Port opened successfully.\n\n")

	// CRSF channel frame: 16 channels all at center (992)
	channelFrame := buildChannelFrame()
	// CRSF ping devices frame
	pingFrame := []byte{0xC8, 0x04, 0x28, 0x00, 0xEA, 0x54}

	// Send a ping first
	fmt.Printf("[TX] Sending ping devices frame: %X\n", pingFrame)
	if _, err := port.Write(pingFrame); err != nil {
		fmt.Printf("ERROR writing ping: %s\n", err)
	}

	startTime := time.Now()
	ticker := time.NewTicker(4 * time.Millisecond) // ~250Hz like the Pocket
	defer ticker.Stop()

	sentCount := 0
	rxBytes := 0
	readBuf := make([]byte, 256)

	fmt.Printf("[RX] Listening for responses...\n\n")

	for time.Since(startTime) < time.Duration(*duration)*time.Second {
		select {
		case <-ticker.C:
			// Send channel frame
			if _, err := port.Write(channelFrame); err != nil {
				fmt.Printf("ERROR writing channels: %s\n", err)
				return
			}
			sentCount++

			// Every 500 frames (~2s), send a ping too
			if sentCount%500 == 0 {
				port.Write(pingFrame)
				fmt.Printf("[TX] Sent %d channel frames so far, sending ping...\n", sentCount)
			}
		default:
			// Try to read
			n, err := port.Read(readBuf)
			if err != nil {
				continue
			}
			if n > 0 {
				rxBytes += n
				fmt.Printf("[RX] Got %d bytes: ", n)
				for i := 0; i < n; i++ {
					fmt.Printf("%02X ", readBuf[i])
				}
				fmt.Printf("\n")

				// Try to identify CRSF frames
				identifyFrames(readBuf[:n])
			}
		}
	}

	fmt.Printf("\n=== Summary ===\n")
	fmt.Printf("Sent: %d channel frames\n", sentCount)
	fmt.Printf("Received: %d bytes total\n", rxBytes)
	if rxBytes == 0 {
		fmt.Printf("RESULT: No response from TX module at baud %d\n", *baudRate)
		fmt.Printf("  -> Try a different baud rate, or check if backpack is enabled\n")
	} else {
		fmt.Printf("RESULT: TX module is responding at baud %d!\n", *baudRate)
	}
}

func buildChannelFrame() []byte {
	// Build a CRSF channels frame with all 16 channels at center (992)
	var buf [26]byte
	buf[0] = 0xEE // sync/address
	buf[1] = 24   // length: 1(type) + 22(payload) + 1(crc)
	buf[2] = 0x16 // RC channels packed

	// Pack 16 channels of value 992 (center) into 22 bytes
	// Each channel is 11 bits
	channels := [16]uint16{992, 992, 992, 992, 992, 992, 992, 992,
		992, 992, 992, 992, 992, 992, 992, 992}

	var offset uint8 = 3
	var bits uint32 = 0
	var bitsAvailable uint8 = 0

	for i := 0; i < 16; i++ {
		bits |= uint32(channels[i]) << bitsAvailable
		bitsAvailable += 11
		for bitsAvailable >= 8 {
			buf[offset] = byte(bits & 0xFF)
			offset++
			bits >>= 8
			bitsAvailable -= 8
		}
	}

	// CRC8 DVB-S2 (poly 0xD5) over bytes 2..24
	buf[25] = crc8d5(buf[2:25])
	return buf[:]
}

func crc8d5(data []byte) byte {
	var crc byte = 0
	for _, b := range data {
		crc ^= b
		for i := 0; i < 8; i++ {
			if crc&0x80 != 0 {
				crc = (crc << 1) ^ 0xD5
			} else {
				crc = crc << 1
			}
		}
	}
	return crc
}

func identifyFrames(data []byte) {
	frameTypes := map[byte]string{
		0x02: "GPS",
		0x08: "Battery",
		0x14: "LinkStats",
		0x16: "RCChannels",
		0x21: "FlightMode",
		0x28: "PingDevices",
		0x29: "DeviceInfo",
		0x2B: "SettingsEntry",
		0x2E: "Status",
		0x32: "Command",
		0xC8: "UartSync",
	}

	for i, b := range data {
		if name, ok := frameTypes[b]; ok {
			fmt.Printf("       ^ Possible frame type 0x%02X (%s) at offset %d\n", b, name, i)
		}
	}
}

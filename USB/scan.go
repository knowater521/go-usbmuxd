package USB

import (
	"time"

	"github.com/yichengchen/go-usbmuxd/frames"
	"github.com/yichengchen/go-usbmuxd/transmission"
)

type (
	Scan struct {
		IsScanning bool
	}
)

func (scan *Scan) Start(device *ConnectedDevices, frame frames.USBDeviceAttachedDetachedFrame, toPort int) {
	if false == scan.IsScanning {
		// set scanning to true and scan
		scan.IsScanning = true
		// scanning routine
		go func() {
			// scan here
			// first check the chan for any interrupt
			// if interrupt is there ... break the loop and set isScanning false
			for {
				if scan.IsScanning && device.Connection != nil {
					device.Connect(transmission.Tunnel(), frame, toPort)
				} else {
					break
				}
				time.Sleep(time.Second * 2)
			}
			if scan.IsScanning {
				scan.IsScanning = false
			}
		}()
	}
}
func (scan *Scan) Stop() {
	if true == scan.IsScanning {
		scan.IsScanning = false
	}
}

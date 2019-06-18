package USB

import (
	"bytes"
	"encoding/binary"
	"errors"
	"io"
	"log"
	"net"

	"github.com/yichengchen/go-usbmuxd/frames"
	"howett.net/plist"
)

var peerTalKVersion = []byte{0, 0, 0, 1}

type (
	ConnectedDeviceDelegate interface {
		USBDeviceDidSuccessfullyConnect(device ConnectedDevices, deviceID int, toPort int)
		USBDeviceDidFailToConnect(device ConnectedDevices, deviceID int, toPort int, err error)
		USBDeviceDidReceiveData(device ConnectedDevices, deviceID int, messageTAG uint32, tag uint32, data []byte)
		USBDeviceDidDisconnect(devices ConnectedDevices, deviceID int, toPort int)
	}
	ConnectedDevices struct {
		Delegate   ConnectedDeviceDelegate
		Connection net.Conn
	}
)

func (device ConnectedDevices) Connect(conn net.Conn, frame frames.USBDeviceAttachedDetachedFrame, port int) net.Conn {
	device.Connection.Close()
	device.Connection = conn
	setupTunnel(device.Connection, frame.DeviceID, port, device)

	go connectFrameParser(device.Connection, frame.DeviceID, port, device)

	return device.Connection
}

func setupTunnel(conn net.Conn, deviceID int, toPort int, device ConnectedDevices) {
	// now send a connect request to the device
	device.Connection.Write(sendConnectRequestToSocket(deviceID, toPort))

	lengthBuf := make([]byte, 16)
	_, err := io.ReadFull(conn, lengthBuf)
	if err != nil {
		device.Delegate.USBDeviceDidFailToConnect(device, deviceID, toPort, errors.New("[USB-ERROR-CONNECT-SetupTunnel]: Get Header length fail"))
		return
	}
	length := binary.LittleEndian.Uint32(lengthBuf[0:4]) - 16
	version := binary.LittleEndian.Uint32(lengthBuf[4:8])
	// msgType := binary.LittleEndian.Uint32(lengthBuf[8:12])
	// tag := binary.LittleEndian.Uint32(lengthBuf[12:16])
	// fmt.Println(length, version, msgType, tag)

	if version != 1 {
		device.Delegate.USBDeviceDidFailToConnect(device, deviceID, toPort, errors.New("[USB-ERROR-CONNECT-SetupTunnel]: Version Fault"))
		return
	}

	bodyBuf := make([]byte, length)
	_, err = io.ReadFull(conn, bodyBuf)

	if err != nil {
		device.Delegate.USBDeviceDidFailToConnect(device, deviceID, toPort, errors.New("[USB-ERROR-CONNECT-SetupTunnel]: Get Header fail"))
		return
	}

	// initial check for message type
	var data frames.USBGenericACKFrame
	decoder := plist.NewDecoder(bytes.NewReader(bodyBuf))
	decoder.Decode(&data)
	if data.MessageType == "Result" && data.Number == 0 {
		device.Delegate.USBDeviceDidSuccessfullyConnect(device, deviceID, toPort)
	} else if data.MessageType == "Result" && data.Number != 0 {
		var errorMessage string
		switch data.Number {
		case 2:
			// Device Disconnected
			errorMessage = "Unable to connect to Device, might be issue with the cable or turned off"
		case 3:
			// Port isn't available/ busy
			errorMessage = "Port you're requesting is unavailable"
		case 5:
			// UNKNOWN Error
			errorMessage = "[IDK]: Malformed request received in the device"
		}
		device.Delegate.USBDeviceDidFailToConnect(device, deviceID, toPort, errors.New(errorMessage))
	}

}

func (device ConnectedDevices) SendData(data []byte, tag uint32, messageTagType uint32) {
	// create a 20byte standard header that's used by peertalk to parse tag and other info
	headerBuffer := make([]byte, 16)

	// preparing the header
	binary.BigEndian.PutUint32(headerBuffer[:4], 1)                    //version
	binary.BigEndian.PutUint32(headerBuffer[4:8], messageTagType)      //type
	binary.BigEndian.PutUint32(headerBuffer[8:12], tag)                //tag
	binary.BigEndian.PutUint32(headerBuffer[12:16], uint32(len(data))) //payloadSize
	if device.Connection != nil {
		_, err := device.Connection.Write(append(headerBuffer, data...))
		if err != nil {
			log.Println(err.Error())
		}
	}
}

func byteSwap(val int) int {
	return ((val & 0xFF) << 8) | ((val >> 8) & 0xFF)
}

func sendConnectRequestToSocket(deviceID int, toPort int) []byte {
	// constructing body
	body := &frames.USBConnectRequestFrame{
		DeviceID:            deviceID,
		PortNumber:          byteSwap(toPort),
		MessageType:         "Connect",
		ClientVersionString: "1.0.0",
		ProgName:            "go-usbmuxd",
	}
	bodyBuffer := &bytes.Buffer{}
	encoder := plist.NewEncoder(bodyBuffer)
	encoder.Encode(body)

	//constructing header
	headerBuffer := make([]byte, 16)
	binary.LittleEndian.PutUint32(headerBuffer, uint32(bodyBuffer.Len()+16))
	headerBuffer[4] = byte(1)
	headerBuffer[8] = byte(8)
	headerBuffer[12] = byte(1)

	requestBuffer := append(headerBuffer, bodyBuffer.Bytes()...)
	return requestBuffer
}

var (
	messageOffset  = 0
	messageType    = uint32(0)
	messageTag     = uint32(0)
	messagePayload = make([]byte, 0)
)

func connectFrameParser(conn net.Conn, deviceID int, toPort int, device ConnectedDevices) {
	buf := make([]byte, 1024*50)

	for {
		n, err := conn.Read(buf)
		if err != nil {
			if err != io.EOF {
				log.Println(err)
				log.Println("[USB-ERROR-iCONNECT-1] : Unable to read data stream from the USB channel")
			}
			device.Delegate.USBDeviceDidDisconnect(device, deviceID, toPort)
			break
		}
		parseData(buf, n, deviceID, device)
	}
}

func parseData(buf []byte, n int, deviceID int, device ConnectedDevices) {
	if messageOffset == -1 {
		// parse the TAG and other relevant header info
		if len(buf) < 16 {
			log.Println("len(buf) < 16", buf)
			return
		}
		headerBuffer := buf[:16]
		version := headerBuffer[0:4]
		if !bytes.Equal(version, peerTalKVersion) {
			messageOffset = -1
			return
		}
		messageType = binary.BigEndian.Uint32(headerBuffer[4:8])
		messageTag = binary.BigEndian.Uint32(headerBuffer[8:12])
		messageSize := int(binary.BigEndian.Uint32(headerBuffer[12:16]))
		messagePayload = make([]byte, messageSize)

		messageOffset = 0
		remainData := make([]byte, n-16)
		copy(remainData[:], buf[16:])
		parseData(remainData, len(remainData), deviceID, device)
		return
	}

	if messageOffset+n > len(messagePayload) {
		// fmt.Println("messageOffset+n > len(messagePayload)", len(messagePayload)-messageOffset, n)
		copy(messagePayload[messageOffset:], buf[:len(messagePayload)-messageOffset])
		device.Delegate.USBDeviceDidReceiveData(device, deviceID, messageTag, messageType, messagePayload)
		remainData := make([]byte, n-messageOffset)
		copy(remainData, buf[len(messagePayload):n])
		messageOffset = -1
		parseData(remainData, len(remainData), deviceID, device)
		return
	}
	copy(messagePayload[messageOffset:], buf[:n])
	messageOffset += n

	messageSize := len(messagePayload)
	switch {
	case messageOffset == messageSize:
		device.Delegate.USBDeviceDidReceiveData(device, deviceID, messageTag, messageType, messagePayload)
		messageOffset = -1
	case messageOffset < messageSize:
		// fmt.Println("messageOffset < messageSize", messageLength, messageSize)
		return
	}
}

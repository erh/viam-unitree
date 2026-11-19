package main

import (
	"encoding/binary"
	"errors"
	"fmt"
	"sync"
	"time"

	"golang.org/x/sys/unix"
)

// Minimal Modbus RTU client for BrainCo Revo2 hands over RS485.
//
// The G1's Jetson reaches each hand through one channel of an FTDI FT4232H
// USB-to-RS485 adapter. Revo2 hands speak Modbus RTU at 460800 baud; by
// default the left hand is slave 126 (0x7E) and the right is 127 (0x7F).
// Register map from BrainCo's brainco-hand-sdk (tools/canfd-diagnostics):
//
//	0x03 holding 901       SKU: 1/3 = right, 2/4 = left
//	0x04 input   2000 x6   finger positions, 0 (open) .. 1000 (closed)
//	0x04 input   2006 x6   finger speeds (signed)
//	0x04 input   2012 x6   finger currents (signed)
//	0x04 input   2018 x6   motor status (0 = ok)
//	0x04 input   3000 x10  firmware version (ASCII)
//	0x10 write   1070 x3   six finger targets in percent, 0 (open) .. 100 (closed),
//	                       packed two per register (see packPercentTargets)
//
// Finger order everywhere: thumb, thumb_aux (rotation), index, middle, ring, pinky.

const (
	brainCoBaud = 460800

	brainCoRegSKU            = 901
	brainCoRegStatus         = 2000
	brainCoRegFirmware       = 3000
	brainCoRegPercentTargets = 1070

	brainCoNumFingers = 6
)

var brainCoFingerNames = [brainCoNumFingers]string{"thumb", "thumb_aux", "index", "middle", "ring", "pinky"}

// BrainCoStatus is one read of a hand's motor state.
type BrainCoStatus struct {
	Positions [brainCoNumFingers]int // 0 (open) .. 1000 (closed)
	Speeds    [brainCoNumFingers]int
	Currents  [brainCoNumFingers]int
	Motor     [brainCoNumFingers]int // 0 = ok
}

// BrainCoHand talks Modbus RTU to one hand. Safe for concurrent use.
type BrainCoHand struct {
	mu      sync.Mutex
	fd      int
	slaveID byte
	timeout time.Duration
}

// OpenBrainCoHand opens the serial port at 460800 8N1.
func OpenBrainCoHand(path string, slaveID byte) (*BrainCoHand, error) {
	fd, err := unix.Open(path, unix.O_RDWR|unix.O_NOCTTY|unix.O_CLOEXEC, 0)
	if err != nil {
		return nil, fmt.Errorf("open %s: %w", path, err)
	}
	t, err := unix.IoctlGetTermios(fd, unix.TCGETS)
	if err != nil {
		unix.Close(fd)
		return nil, fmt.Errorf("get termios %s: %w", path, err)
	}
	t.Iflag = 0
	t.Oflag = 0
	t.Lflag = 0
	t.Cflag = unix.CS8 | unix.CREAD | unix.CLOCAL | unix.B460800
	t.Ispeed = unix.B460800
	t.Ospeed = unix.B460800
	t.Cc[unix.VMIN] = 0
	t.Cc[unix.VTIME] = 1 // reads return after at most 100 ms of silence
	if err := unix.IoctlSetTermios(fd, unix.TCSETS, t); err != nil {
		unix.Close(fd)
		return nil, fmt.Errorf("set termios %s: %w", path, err)
	}
	return &BrainCoHand{fd: fd, slaveID: slaveID, timeout: 300 * time.Millisecond}, nil
}

func (h *BrainCoHand) Close() error {
	h.mu.Lock()
	defer h.mu.Unlock()
	if h.fd < 0 {
		return nil
	}
	err := unix.Close(h.fd)
	h.fd = -1
	return err
}

func modbusCRC(b []byte) uint16 {
	crc := uint16(0xFFFF)
	for _, x := range b {
		crc ^= uint16(x)
		for i := 0; i < 8; i++ {
			if crc&1 != 0 {
				crc = (crc >> 1) ^ 0xA001
			} else {
				crc >>= 1
			}
		}
	}
	return crc
}

// transact sends a request (without CRC) and returns the response body
// (without slave id, function code or CRC). responseLen is the full expected
// response length including CRC.
func (h *BrainCoHand) transact(req []byte, responseLen int) ([]byte, error) {
	h.mu.Lock()
	defer h.mu.Unlock()
	if h.fd < 0 {
		return nil, errors.New("hand is closed")
	}

	frame := binary.LittleEndian.AppendUint16(append([]byte{}, req...), modbusCRC(req))
	_ = unix.IoctlSetInt(h.fd, unix.TCFLSH, unix.TCIOFLUSH)
	if _, err := unix.Write(h.fd, frame); err != nil {
		return nil, fmt.Errorf("write: %w", err)
	}

	resp := make([]byte, 0, responseLen)
	buf := make([]byte, 256)
	deadline := time.Now().Add(h.timeout)
	for len(resp) < responseLen && time.Now().Before(deadline) {
		n, err := unix.Read(h.fd, buf)
		if err != nil && err != unix.EINTR && err != unix.EAGAIN {
			return nil, fmt.Errorf("read: %w", err)
		}
		resp = append(resp, buf[:max(n, 0)]...)
		// A Modbus exception reply is 5 bytes: id, fn|0x80, code, crc.
		if len(resp) >= 5 && resp[1] == req[1]|0x80 {
			return nil, fmt.Errorf("modbus exception %d (function 0x%02x)", resp[2], req[1])
		}
	}
	if len(resp) < responseLen {
		return nil, fmt.Errorf("timeout: got %d of %d bytes (slave %d)", len(resp), responseLen, h.slaveID)
	}
	resp = resp[:responseLen]
	if got, want := binary.LittleEndian.Uint16(resp[responseLen-2:]), modbusCRC(resp[:responseLen-2]); got != want {
		return nil, fmt.Errorf("bad CRC from slave %d", h.slaveID)
	}
	if resp[0] != h.slaveID || resp[1] != req[1] {
		return nil, fmt.Errorf("unexpected reply header % x", resp[:2])
	}
	return resp[2 : responseLen-2], nil
}

func (h *BrainCoHand) readRegisters(function byte, address, count uint16) ([]uint16, error) {
	req := []byte{h.slaveID, function, 0, 0, 0, 0}
	binary.BigEndian.PutUint16(req[2:], address)
	binary.BigEndian.PutUint16(req[4:], count)
	body, err := h.transact(req, 5+2*int(count))
	if err != nil {
		return nil, err
	}
	if int(body[0]) != 2*int(count) {
		return nil, fmt.Errorf("unexpected byte count %d", body[0])
	}
	values := make([]uint16, count)
	for i := range values {
		values[i] = binary.BigEndian.Uint16(body[1+2*i:])
	}
	return values, nil
}

// writeRegistersRequest builds a function 0x10 request, without the CRC.
func writeRegistersRequest(slaveID byte, address uint16, values []uint16) []byte {
	req := []byte{slaveID, 0x10, 0, 0, 0, 0, byte(2 * len(values))}
	binary.BigEndian.PutUint16(req[2:], address)
	binary.BigEndian.PutUint16(req[4:], uint16(len(values)))
	for _, v := range values {
		req = binary.BigEndian.AppendUint16(req, v)
	}
	return req
}

func (h *BrainCoHand) writeRegisters(address uint16, values []uint16) error {
	_, err := h.transact(writeRegistersRequest(h.slaveID, address, values), 8)
	return err
}

// SKU returns the hand's SKU register: 1/3 = right, 2/4 = left.
func (h *BrainCoHand) SKU() (int, error) {
	v, err := h.readRegisters(0x03, brainCoRegSKU, 1)
	if err != nil {
		return 0, err
	}
	return int(v[0]), nil
}

// Firmware returns the firmware version string.
func (h *BrainCoHand) Firmware() (string, error) {
	v, err := h.readRegisters(0x04, brainCoRegFirmware, 10)
	if err != nil {
		return "", err
	}
	b := make([]byte, 0, 20)
	for _, w := range v {
		b = binary.BigEndian.AppendUint16(b, w)
	}
	for len(b) > 0 && b[len(b)-1] == 0 {
		b = b[:len(b)-1]
	}
	return string(b), nil
}

// Status reads positions, speeds, currents and motor status in one request.
func (h *BrainCoHand) Status() (BrainCoStatus, error) {
	v, err := h.readRegisters(0x04, brainCoRegStatus, 4*brainCoNumFingers)
	if err != nil {
		return BrainCoStatus{}, err
	}
	var st BrainCoStatus
	for i := 0; i < brainCoNumFingers; i++ {
		st.Positions[i] = int(v[i])
		st.Speeds[i] = int(int16(v[brainCoNumFingers+i]))
		st.Currents[i] = int(int16(v[2*brainCoNumFingers+i]))
		st.Motor[i] = int(v[3*brainCoNumFingers+i])
	}
	return st, nil
}

// packPercentTargets packs six 0–100 targets two per register, the way
// BrainCo's own tooling does: register k holds finger 2k in its low byte and
// finger 2k+1 in its high byte.
func packPercentTargets(percent [brainCoNumFingers]int) []uint16 {
	words := make([]uint16, brainCoNumFingers/2)
	for k := range words {
		lo := uint16(min(max(percent[2*k], 0), 100))
		hi := uint16(min(max(percent[2*k+1], 0), 100))
		words[k] = lo | hi<<8
	}
	return words
}

// SetTargetsPercent commands all six fingers, 0 (open) .. 100 (closed).
func (h *BrainCoHand) SetTargetsPercent(percent [brainCoNumFingers]int) error {
	return h.writeRegisters(brainCoRegPercentTargets, packPercentTargets(percent))
}

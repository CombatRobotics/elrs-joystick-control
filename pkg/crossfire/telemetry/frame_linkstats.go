// SPDX-FileCopyrightText: © 2023 OneEyeFPV oneeyefpv@gmail.com
// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-License-Identifier: FS-0.9-or-later

package telemetry

import (
	"fmt"
	"github.com/kaack/elrs-joystick-control/pkg/crossfire"
)

// LinkStatsFrameSize is the on-the-wire byte count of a LinkStatistics frame:
//
//	[0xEA][len=0x0c][type=0x14][10 bytes payload][crc] = 14 bytes
const LinkStatsFrameSize int = 14

// LinkStatsFrame is sent by the TX firmware ONLY when connectionState ==
// connected, i.e. when bound to an RX that is sending telemetry. Presence of
// these frames is therefore a positive "bound" signal; absence for >2s is a
// reliable "unbound" signal.
//
// Payload layout (matches CRSF.cpp's crsfPayloadLinkstatistics_s):
//
//	[3]  uplink_RSSI_1     (uint8, dBm * -1)
//	[4]  uplink_RSSI_2     (uint8, dBm * -1)
//	[5]  uplink_LQ         (uint8, %)
//	[6]  uplink_SNR        (int8, dB)
//	[7]  active_antenna    (uint8)
//	[8]  rf_mode           (uint8 enum: 4fps=0, 50fps, 150hz, ...)
//	[9]  uplink_TX_power   (uint8 enum: 0mW, 10mW, 25mW, 100mW, 500mW, 1W, 2W)
//	[10] downlink_RSSI     (uint8, dBm * -1)
//	[11] downlink_LQ       (uint8, %)
//	[12] downlink_SNR      (int8, dB)
type LinkStatsFrame struct {
	RawData []uint8
}

func (t *LinkStatsFrame) Addr() crossfire.Endpoint {
	return crossfire.Endpoint(t.RawData[0])
}

func (t *LinkStatsFrame) Type() crossfire.FrameType {
	return crossfire.FrameType(t.RawData[2])
}

func (t *LinkStatsFrame) Data() []uint8 {
	return t.RawData[3:]
}

// UplinkRSSI returns dBm (negative number) of antenna 1.
func (t *LinkStatsFrame) UplinkRSSI() int32 {
	return -int32(t.RawData[3])
}

// UplinkRSSI2 returns dBm (negative number) of antenna 2.
func (t *LinkStatsFrame) UplinkRSSI2() int32 {
	return -int32(t.RawData[4])
}

// UplinkLQ returns uplink link quality as a 0..100 percentage.
func (t *LinkStatsFrame) UplinkLQ() uint32 {
	return uint32(t.RawData[5])
}

// UplinkSNR returns uplink signal-to-noise ratio in dB.
func (t *LinkStatsFrame) UplinkSNR() int32 {
	return int32(int8(t.RawData[6]))
}

func (t *LinkStatsFrame) ActiveAntenna() uint32 {
	return uint32(t.RawData[7])
}

// RfMode is the air-rate enum (50/100/150/250/333/500Hz, F1000, ...).
func (t *LinkStatsFrame) RfMode() uint32 {
	return uint32(t.RawData[8])
}

// TxPower is an enum: 0mW=0, 10mW, 25mW, 100mW, 500mW, 1W, 2W.
func (t *LinkStatsFrame) TxPower() uint32 {
	return uint32(t.RawData[9])
}

// DownlinkRSSI returns dBm (negative number) reported by the RX.
func (t *LinkStatsFrame) DownlinkRSSI() int32 {
	return -int32(t.RawData[10])
}

func (t *LinkStatsFrame) DownlinkLQ() uint32 {
	return uint32(t.RawData[11])
}

func (t *LinkStatsFrame) DownlinkSNR() int32 {
	return int32(int8(t.RawData[12]))
}

func (t *LinkStatsFrame) String() string {
	return fmt.Sprintf(
		"(link-stats) up_rssi=%ddBm up_lq=%d%% up_snr=%ddB rf_mode=%d tx_pwr=%d dn_rssi=%ddBm dn_lq=%d%% dn_snr=%ddB",
		t.UplinkRSSI(), t.UplinkLQ(), t.UplinkSNR(),
		t.RfMode(), t.TxPower(),
		t.DownlinkRSSI(), t.DownlinkLQ(), t.DownlinkSNR(),
	)
}

#pragma once

//=============================================================================
// REGISTER ADDRESSES
//=============================================================================

// General Configuration
#define REG_FIFO 0x00
#define REG_OPMODE 0x01
#define REG_DATAMODUL 0x02
#define REG_BITRATEMSB 0x03
#define REG_BITRATELSB 0x04
#define REG_FDEVMSB 0x05
#define REG_FDEVLSB 0x06

// Frequency Registers
#define REG_FRFMSB 0x07
#define REG_FRFMID 0x08
#define REG_FRFLSB 0x09

// Transmitter Registers
#define REG_PALEVEL 0x11
#define REG_PARAMP 0x12

// Receiver Registers
#define REG_RXBW 0x19
#define REG_AFCBW 0x1A
#define REG_AFCFEI 0x1E
#define REG_RSSICONFIG 0x23
#define REG_RSSIVALUE 0x24

// IRQ and DIO Mapping
#define REG_DIOMAPPING1 0x25
#define REG_DIOMAPPING2 0x26
#define REG_IRQFLAGS1 0x27
#define REG_IRQFLAGS2 0x28

// Packet Engine Registers
#define REG_SYNCCONFIG 0x2E
#define REG_SYNCVALUE1 0x2F
#define REG_SYNCVALUE2 0x30
#define REG_SYNCVALUE3 0x31
#define REG_SYNCVALUE4 0x32
#define REG_SYNCVALUE5 0x33
#define REG_SYNCVALUE6 0x34
#define REG_SYNCVALUE7 0x35
#define REG_SYNCVALUE8 0x36
#define REG_PACKETCONFIG1 0x37
#define REG_PAYLOADLENGTH 0x38
#define REG_NODEADRS 0x39
#define REG_BROADCASTADRS 0x3A
#define REG_FIFOTHRESH 0x3C
#define REG_PACKETCONFIG2 0x3D

// Version and Test Registers
#define REG_VERSION 0x10
#define REG_TESTPA1 0x5A
#define REG_TESTPA2 0x5C
#define REG_TESTDAGC 0x6F

//=============================================================================
// REG_OPMODE (0x01) - Operating Mode and Sequencer Control
//=============================================================================

// Sequencer control
#define OPMODE_SEQUENCER_OFF 0x80  // Bit 7: Disable automatic sequencer
#define OPMODE_SEQUENCER_ON 0x00   // Bit 7: Enable automatic sequencer
#define OPMODE_SEQUENCER_MASK 0x80 // Mask for sequencer bit

// Listen mode control
#define OPMODE_LISTEN_ON 0x40    // Bit 6: Enable Listen mode
#define OPMODE_LISTEN_OFF 0x00   // Bit 6: Disable Listen mode
#define OPMODE_LISTEN_ABORT 0x20 // Bit 5: Abort Listen mode

// Mode bits mask and shift
#define OPMODE_MODE_MASK 0x1C // Bits 4-2: Mode bits
#define OPMODE_MODE_SHIFT 2   // Shift for mode bits

// Operating modes (bits 4-2, already shifted)
#define OPMODE_SLEEP (0x00 << 2)   // 0x00: Sleep mode
#define OPMODE_STANDBY (0x01 << 2) // 0x04: Standby mode
#define OPMODE_FS (0x02 << 2)      // 0x08: Frequency Synthesis mode
#define OPMODE_TX (0x03 << 2)      // 0x0C: Transmitter mode
#define OPMODE_RX (0x04 << 2)      // 0x10: Receiver mode

// Convenience mask for preserving non-mode bits when changing mode
#define OPMODE_PRESERVE_MASK 0xE3 // Preserve Sequencer + Listen settings (bits 7-6)

//=============================================================================
// REG_DATAMODUL (0x02) - Data Processing Mode
//=============================================================================

// Data mode (bits 6-5)
#define DATAMODUL_PACKET_MODE 0x00       // Packet mode
#define DATAMODUL_CONTINUOUS_SYNC 0x40   // Continuous mode with sync
#define DATAMODUL_CONTINUOUS_NOSYNC 0x60 // Continuous mode without sync
#define DATAMODUL_MODE_MASK 0x60         // Mask for data mode bits

// Modulation type (bit 3)
#define DATAMODUL_FSK 0x00             // FSK modulation
#define DATAMODUL_OOK 0x08             // OOK modulation
#define DATAMODUL_MODULATION_MASK 0x08 // Mask for modulation bit

// Modulation shaping (bits 1-0)
#define DATAMODUL_SHAPING_NONE 0x00   // No shaping
#define DATAMODUL_SHAPING_BT_1_0 0x01 // Gaussian filter BT = 1.0 (FSK) / RC filter (OOK)
#define DATAMODUL_SHAPING_BT_0_5 0x02 // Gaussian filter BT = 0.5 (FSK) / 2*RC filter (OOK)
#define DATAMODUL_SHAPING_BT_0_3 0x03 // Gaussian filter BT = 0.3 (FSK) / RC + BR filter (OOK)
#define DATAMODUL_SHAPING_MASK 0x03   // Mask for shaping bits

//=============================================================================
// REG_PALEVEL (0x11) - PA Power Level Control
//=============================================================================

#define PALEVEL_PA0_ON 0x80       // Bit 7: Enable PA0 (low power)
#define PALEVEL_PA1_ON 0x40       // Bit 6: Enable PA1 (high power)
#define PALEVEL_PA2_ON 0x20       // Bit 5: Enable PA2 (high power + boost)
#define PALEVEL_OUTPUT_POWER 0x1F // Bits 4-0: Output power setting (0-31)

//=============================================================================
// REG_IRQFLAGS1 (0x27) - IRQ Status Flags 1
//=============================================================================

#define IRQ1_MODEREADY 0x80        // Bit 7: Mode ready
#define IRQ1_RXREADY 0x40          // Bit 6: RX mode ready
#define IRQ1_TXREADY 0x20          // Bit 5: TX mode ready
#define IRQ1_PLLLOCK 0x10          // Bit 4: PLL locked
#define IRQ1_RSSI 0x08             // Bit 3: RSSI threshold exceeded
#define IRQ1_TIMEOUT 0x04          // Bit 2: Timeout occurred
#define IRQ1_AUTOMODE 0x02         // Bit 1: Auto mode intermediate
#define IRQ1_SYNCADDRESSMATCH 0x01 // Bit 0: Sync word or address match

//=============================================================================
// REG_IRQFLAGS2 (0x28) - IRQ Status Flags 2
//=============================================================================

#define IRQ2_FIFO_FULL 0x80      // Bit 7: FIFO full
#define IRQ2_FIFO_NOT_EMPTY 0x40 // Bit 6: FIFO not empty
#define IRQ2_FIFO_LEVEL 0x20     // Bit 5: FIFO level threshold exceeded
#define IRQ2_FIFO_OVERRUN 0x10   // Bit 4: FIFO overrun occurred
#define IRQ2_PACKET_SENT 0x08    // Bit 3: Packet transmission complete
#define IRQ2_PAYLOAD_READY 0x04  // Bit 2: Payload ready to read
#define IRQ2_CRC_OK 0x02         // Bit 1: CRC check passed
#define IRQ2_LOW_BAT 0x01        // Bit 0: Low battery detected

//=============================================================================
// REG_PACKETCONFIG1 (0x37) - Packet Format Configuration
//=============================================================================

// Packet format (bit 7)
#define PACKET1_FORMAT_FIXED 0x00    // Fixed length packets
#define PACKET1_FORMAT_VARIABLE 0x80 // Variable length packets

// DC-free encoding (bits 6-5)
#define PACKET1_DC_FREE_OFF 0x00        // No DC-free encoding
#define PACKET1_DC_FREE_MANCHESTER 0x20 // Manchester encoding
#define PACKET1_DC_FREE_WHITENING 0x40  // Whitening
#define PACKET1_DC_FREE_MASK 0x60       // Mask for DC-free bits

// CRC (bit 4)
#define PACKET1_CRC_ON 0x10  // Enable CRC calculation/check
#define PACKET1_CRC_OFF 0x00 // Disable CRC

// CRC auto-clear (bit 3)
#define PACKET1_CRC_AUTOCLR_ON 0x00  // Clear FIFO on CRC fail
#define PACKET1_CRC_AUTOCLR_OFF 0x08 // Keep FIFO on CRC fail

// Address filtering (bits 2-1)
#define PACKET1_ADDR_FILTER_OFF 0x00  // No address filtering
#define PACKET1_ADDR_FILTER_NODE 0x02 // Filter on node address
#define PACKET1_ADDR_FILTER_BOTH 0x06 // Filter on node or broadcast address
#define PACKET1_ADDR_FILTER_MASK 0x06 // Mask for address filter bits

// Promiscuous mode - ALTERNATIVE definition for bit 2
// (When address filtering is off, bit 2 can act as promiscuous mode)
#define PACKET1_PROMISCUOUS_ON 0x04  // Accept all packets
#define PACKET1_PROMISCUOUS_OFF 0x00 // Normal filtering
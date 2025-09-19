#pragma once

// General
#define REG_FIFO           0x00
#define REG_OPMODE         0x01
#define REG_DATAMODUL      0x02
#define REG_BITRATEMSB     0x03
#define REG_BITRATELSB     0x04
#define REG_FDEVMSB        0x05
#define REG_FDEVLSB        0x06

// Frequency
#define REG_FRFMSB         0x07
#define REG_FRFMID         0x08
#define REG_FRFLSB         0x09

// Transmitter
#define REG_PALEVEL        0x11
#define REG_PARAMP         0x12

// Receiver
#define REG_RXBW           0x19
#define REG_AFCBW          0x1A
#define REG_AFCFEI         0x1E
#define REG_RSSICONFIG     0x23
#define REG_RSSIVALUE      0x24

// IRQ & DIO
#define REG_DIOMAPPING1    0x25
#define REG_DIOMAPPING2    0x26
#define REG_IRQFLAGS1      0x27
#define REG_IRQFLAGS2      0x28

// Packet handling
#define REG_SYNCCONFIG     0x2E
#define REG_SYNCVALUE1     0x2F
#define REG_SYNCVALUE2     0x30
#define REG_SYNCVALUE3     0x31
#define REG_SYNCVALUE4     0x32
#define REG_SYNCVALUE5     0x33
#define REG_SYNCVALUE6     0x34
#define REG_SYNCVALUE7     0x35
#define REG_SYNCVALUE8     0x36
#define REG_PACKETCONFIG1  0x37
#define REG_PAYLOADLENGTH  0x38
#define REG_NODEADRS       0x39
#define REG_BROADCASTADRS  0x3A
#define REG_FIFOTHRESH     0x3C
#define REG_PACKETCONFIG2  0x3D

// Version / test
#define REG_VERSION        0x10
#define REG_TESTPA1        0x5A
#define REG_TESTPA2        0x5C
#define REG_TESTDAGC       0x6F

// Opmodes for register REG_OPMODE
// REG_OPMODE (0x01)    
#define OPMODE_SEQUENCER_OFF   0x80
#define OPMODE_LISTEN_ON       0x40
#define OPMODE_LISTEN_ABORT    0x20
#define OPMODE_MODE_MASK       0x1C
#define OPMODE_SLEEP           0x00
#define OPMODE_STANDBY         0x04
#define OPMODE_SYNTHESIZER     0x08
#define OPMODE_RX              0x10
#define OPMODE_TX              0x0C

// REG_IRQFLAGS1 (0x27)
#define IRQ1_MODEREADY         0x80
#define IRQ1_RXREADY           0x40
#define IRQ1_TXREADY           0x20
#define IRQ1_PLLLOCK           0x10
#define IRQ1_RSSI              0x08
#define IRQ1_TIMEOUT           0x04
#define IRQ1_AUTOMODE          0x02
#define IRQ1_SYNCADDRESSMATCH  0x01

// REG_IRQFLAGS2 (0x28)
#define IRQ2_FIFO_FULL         0x80
#define IRQ2_FIFO_NOT_EMPTY    0x40
#define IRQ2_FIFO_LEVEL        0x20
#define IRQ2_FIFO_OVERRUN      0x10
#define IRQ2_PACKET_SENT       0x08
#define IRQ2_PAYLOAD_READY     0x04
#define IRQ2_CRC_OK            0x02
#define IRQ2_LOW_BAT           0x01

// REG_PACKETCONFIG1 (0x37)
#define REG_PACKETCONFIG1   0x37
#define PACKET1_DCFREE_MASK      0x60
#define PACKET1_DC_FREE_OFF      0x00
#define PACKET1_FORMAT_VARIABLE 0x80    
#define PACKET1_DC_FREE_MASK    0x60
#define PACKET1_CRC_ON          0x10
#define PACKET1_CRC_AUTOCLR_OFF 0x08
#define PACKET1_ADDRESS_FILTER  0x06
#define PACKET1_PACKET_FORMAT   0x01
#define PACKET1_PACKET_FIXED    0x00    
#define PACKET1_PACKET_VARIABLE 0x01
#define PACKET1_PACKET_MANCHESTER 0x02
#define PACKET1_PACKET_WHITENING 0x03
#define PACKET1_PROMISCUOUS_OFF 0x00
#define PACKET1_PROMISCUOUS_ON  0x04
#define PACKET1_PROMISCUOUS     0x08

// RFM69 Register definitions (you'll need these in your header)


// REG_PALEVEL (0x11)
#define PALEVEL_PA0_ON          0x80
#define PALEVEL_PA1_ON          0x40
#define PALEVEL_PA2_ON          0x20
#define PALEVEL_OUTPUT_POWER    0x1F

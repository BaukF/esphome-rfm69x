# ESPHome RFM69x Component

## Architecture Vision

This repository is evolving from a monolithic Duco-specific implementation to a modular, reusable architecture that benefits the entire ESPHome community:

### Current State
- **Monolithic**: Single component handling both RFM69 radio operations and Duco protocol

### Target Architecture
- **RFM69x Radio Component**: Generic sub-GHz radio driver for RFM69x family chips
- **Separate Protocol Components**: Protocol-specific implementations (starting with Duco) that can work with multiple radio transports
- **Uart/serial component**: usage of the existing serial components to do additional reads

### Design Goals

**Hardware Flexibility**: Protocol implementations should work with different radio chips (RFM69x, CC1101, SX127x) by implementing a common interface.

**Transport Agnostic**: Future protocol implementations may support multiple transport methods (RF, serial, etc.) using the same protocol logic.

**Community Driven**: Clean separation enables community contributions of additional radio drivers and protocol implementations without interdependencies.

**ESPHome Native**: Full integration with ESPHome's component system, configuration patterns, and entity types.

### Migration Plan

1. **Phase 1**: Refactor current code to separate RFM69x radio operations from Duco protocol logic
2. **Phase 2**: Create `esphome-duco` repository for protocol-specific implementation
3. **Phase 3**: Establish common interfaces to enable support for additional radio chips and transport methods

### Future Ecosystem

```
Radio Components:                Protocol Components:
├── esphome-rfm69x              ├── esphome-duco
├── esphome-cc1101              ├── esphome-mysensors
└── esphome-sx127x (extended)   └── esphome-[protocol]
```

Users will be able to mix and match radio hardware with protocols based on their specific needs, hardware availability, and system requirements.

---

**Note**: This repository currently contains the original monolithic implementation. The refactoring is in progress - watch for updates!


---
## First comment when this repo was still called esphome-rfm69-duco
In order to create a non invasive communication/connection solution to my Duco Ventition system, I am in the process of creating a esphome based solution.

It uses two (later 3) technical parts to get it to work. First, we have the ESP32 platform that allows for interfacing and connection to (a) home assistant implementation, if preferred. I may consider some autonomous functions, but first lets see if we can get som basics going.
For the 868MHz communications I've decided to work with a RFM69w module (low power variant). In my research, I found that the libs would enable me to move away from difficult radio intrinsics and focus on the data and the Duco Box itself.

In the future, I hope to create a serial connection to read values such as voltages and power etcetera but I haven't dived in it yet, as my first aim is to control the device (low-mid-medium-high debit).

## My hardware setup
ESP32-devkit-1
HopeRF RFM69w

### wiring:
```
                             ┌──────────────────────┐
                       NC    │ ●                  ● │  RST --- GPIO 14 -- D14
      D5 -- GPIO 5 --- NSS   │ ●      ●    ●      ● │ DIO0 --- GPIO 26 -- D26
    D23 -- GPIO 23 --- MOSI  │ ●                  ● │ DIO1
    D19 -- GPIO 19 --- MISO  │ ●                  ● │ DIO2  
    D18 -- GPIO 18 --- SCK   │ ●                  ● │ DIO3
                       GND   │ ●                  ● │ DIO4
                       ANA   │ ●                  ● │ DIO5
               GND --- GND   │ ●                  ● │ 3.3V ----  3.3v
                             └──────────────────────┘
```
## About
This is my autoddidactically challenging project of transforming the interesting work at the project https://github.com/arnemauer/Ducobox-ESPEasy-Plugin into ESPHome.

I have extremely basic programming skills and am using all that is at my disposal to get to a working Duco RF bridge with serial communications. I have an enormous list of todo's before I get to a working sample.

## To do
- [ ] Start sniffing for duco commands
  - [ ] Configure the rfm69 for promiscuous mode
  - [ ] Write caught commands to output
  - [ ] Filter on Duco commands
  - [ ] Find logic on narrowing down on a specific Duco
- [ ] Get hardware
  - [x] ESP32Devkit1
  - [x] RFM69W chip
  - [x] Wiring for chip
  - [ ] wiring for serial
- [x] Rename and redefine
- [x] Upload to GIT
- [x] Get bare minimum SPI component working
  - [x] Get the SPI bus talking and return value from FRFM, using loop as delayed log output (v0.1)
  - [x] Get the module working through the correct setup() function
  - [x] Consider test and try interactions with chip
    - [x] write: REG_OPMODE, 0x00
    - [x] write: REG_FRFMSB, 0xD9
    - [x] write: REG_FRFMID, 0x00
    - [x] write: REG_FRFLSB, 0x00
    - [x] read:   OPMODE:
    - [x] read:   Frequency register (FRF): 0xE4C000
    - [x] read: PA Level: 0x9F
    - [x] read: RSSI: 0 dB
    - [x] read: IRQ Flags: 1=0x80 2=0x00


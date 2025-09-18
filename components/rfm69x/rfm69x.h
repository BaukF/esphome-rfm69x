#pragma once

#include "esphome/core/component.h"
#include "esphome/components/spi/spi.h"

namespace esphome {
namespace rfm69x {

// RFM69 register addresses
static const uint8_t REG_OPMODE     = 0x01;
static const uint8_t REG_FRFMSB     = 0x07;
static const uint8_t REG_FRFMID     = 0x08;
static const uint8_t REG_FRFLSB     = 0x09;
static const uint8_t REG_PALEVEL    = 0x11;
static const uint8_t REG_VERSION    = 0x10;
static const uint8_t REG_RSSIVALUE  = 0x24;
static const uint8_t REG_IRQFLAGS1  = 0x27;
static const uint8_t REG_IRQFLAGS2  = 0x28;

class RFM69x : public Component,
                  public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST,
                                        spi::CLOCK_POLARITY_LOW,
                                        spi::CLOCK_PHASE_LEADING,
                                        spi::DATA_RATE_8MHZ> {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

 protected:
  bool raw_codes_{false};
  const char* decode_opmode(uint8_t opmode);
  uint8_t version_{0};   // Store the version register value
  bool detected_{false};
  uint8_t read_register(uint8_t addr);
  void write_register(uint8_t addr, uint8_t value);
};

}  // namespace rfm69x
}  // namespace esphome

#pragma once

#include "esphome/core/component.h"
#include "esphome/components/spi/spi.h"
#include "rfm69x_reg.h"

namespace esphome {
namespace rfm69x {

class RFM69x : public Component,
                  public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST,
                                        spi::CLOCK_POLARITY_LOW,
                                        spi::CLOCK_PHASE_LEADING,
                                        spi::DATA_RATE_8MHZ> {
 public:
  // Override methods from Component
  void setup() override;
  void loop() override;
  void dump_config() override;

  // Enable/disable raw register values in any output that has raw codes (also see __init__.py)
  void set_raw_codes(bool raw) { this->raw_codes_ = raw; }

 protected:
  //protected variables
  bool detected_{false};      // whether the device was detected during setup

  bool raw_codes_{false};     // whether to show raw register values in dump_config 
  uint8_t version_{0};        // version read from REG_VERSION
  
  // helpful methods
  const char* decode_opmode_(uint8_t opmode);
  std::string decode_irqflags1_(uint8_t val);
  std::string decode_irqflags2_(uint8_t val);                       
  uint8_t read_register_(uint8_t addr);
  void write_register_(uint8_t addr, uint8_t value);
};

}  // namespace rfm69x
}  // namespace esphome

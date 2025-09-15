#pragma once

#include "esphome/core/component.h"
#include "esphome/components/spi/spi.h"

namespace esphome {
namespace duco_rfm69 {

// Add a member variable to store the version


class DucoRFM69 : public Component, 
                    public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST,spi::CLOCK_POLARITY_LOW, 
                            spi::CLOCK_PHASE_LEADING,spi::DATA_RATE_1KHZ> {
  public:
    DucoRFM69();
    void setup() override;
    void loop() override;
    void dump_config() override;

  protected:
      uint8_t version_{0};   // Store the version register value
      bool detected_{false};
};


}  // namespace duco_rfm69
}  // namespace esphome

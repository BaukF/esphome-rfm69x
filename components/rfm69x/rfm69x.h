#pragma once

#include <esphome/core/component.h>
#include <esphome/components/spi/spi.h>
#include "rfm69x_reg.h"

namespace esphome
{
  namespace rfm69x
  {

    class RFM69x : public Component,
                   public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST,
                                         spi::CLOCK_POLARITY_LOW,
                                         spi::CLOCK_PHASE_LEADING,
                                         spi::DATA_RATE_8MHZ>
    {
    public:
      // Override methods from Component
      void setup() override;
      void loop() override;
      void dump_config() override;

      // Enable/disable raw register values in any output that has raw codes (also see __init__.py)
      void set_raw_codes(bool raw) { this->raw_codes_ = raw; }
      void set_reset_pin(InternalGPIOPin *reset_pin) { this->reset_pin_ = reset_pin; }
      void set_frequency(uint32_t freq); // set frequency in MHz, e.g. 868000000
      void set_promiscuous_mode(bool promiscuous) { this->promiscuous_mode_ = promiscuous; }

    protected:
      // protected variables
      bool detected_{false}; // whether the device was detected during setup

      bool raw_codes_{false};               // whether to show raw register values in dump_config
      InternalGPIOPin *reset_pin_{nullptr}; // optional reset pin
      uint8_t version_{0};                  // version read from REG_VERSION
      uint32_t frequency_{868000000};       // default frequency 868 MHz
      bool promiscuous_mode_{false};        // whether promiscuous mode is enabled

      // Set and get methods for the module configuration
      void configure_rfm69x();      // configure the module with current settings
      void reset_rfm69x();          // reset the module via reset pin if defined
      void set_mode_(uint8_t mode); // set the operation mode
      uint32_t get_frequency_() const { return this->frequency_; }
      uint32_t get_frequency_actual_(); // get actual frequency based on FRF registers
      bool get_promiscuous_mode_() const { return this->promiscuous_mode_; }

      // Low-level register access
      uint8_t read_register_(uint8_t addr);
      void write_register_(uint8_t addr, uint8_t value);

      // helpful methods
      const char *decode_opmode_(uint8_t opmode);
      std::string decode_irqflags1_(uint8_t val);
      std::string decode_irqflags2_(uint8_t val);
    };

  } // namespace rfm69x
} // namespace esphome

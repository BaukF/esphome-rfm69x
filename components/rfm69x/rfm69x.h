#pragma once

#include <esphome/core/component.h>
#include <esphome/components/spi/spi.h>
#include "rfm69x_reg.h"

namespace esphome
{
  namespace rfm69x
  {
    // Add these BEFORE your class definition
    enum RFM69ModeType
    {
      MODE_SLEEP,
      MODE_STANDBY,
      MODE_FS,
      MODE_RX,
      MODE_TX
    };
    enum RFM69Modulation
    {
      RFM69_FSK,
      RFM69_OOK
    };

    enum RFM69DataMode
    {
      RFM69_PACKET_MODE,
      RFM69_CONTINUOUS_SYNC,
      RFM69_CONTINUOUS_NOSYNC
    };

    enum RFM69Shaping
    {
      RFM69_SHAPING_NONE,
      RFM69_SHAPING_GAUSSIAN_BT_1_0,
      RFM69_SHAPING_GAUSSIAN_BT_0_5,
      RFM69_SHAPING_GAUSSIAN_BT_0_3
    };

    class RFM69x : public Component,
                   public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST,
                                         spi::CLOCK_POLARITY_LOW,
                                         spi::CLOCK_PHASE_LEADING,
                                         spi::DATA_RATE_8MHZ>
    {
    public:
      // Enable/disable raw register values in any output that has raw codes (also see __init__.py)
      void set_raw_codes(bool raw) { this->raw_codes_ = raw; }
      void set_reset_pin(InternalGPIOPin *reset_pin) { this->reset_pin_ = reset_pin; };
      void set_pll_timeout(uint32_t timeout_ms) { this->pll_timeout_ms_ = timeout_ms; }
      struct RadioStatus
      {
        bool detected;
        uint8_t version;
        std::string mode;
        float frequency_mhz;
        int16_t rssi_dbm;
        bool pll_locked;
        std::string irq1_flags;
        std::string irq2_flags;
      };

      // Override methods from Component
      void setup() override;
      void loop() override;
      void dump_config() override;

      // safe setters that need PLL checks
      void set_frequency(uint32_t freq); // set frequency in MHz, e.g. 868000000
      void set_frequency_deviation(uint32_t frequency_deviation);
      void set_mode(RFM69ModeType mode);
      void set_mode_rx();
      void set_mode_tx();
      void set_mode_standby();
      void set_mode_sleep();
      // setters for other parameters
      void set_modulation(RFM69Modulation mod,
                          RFM69DataMode mode,
                          RFM69Shaping shaping);
      void set_bitrate(uint32_t bps);
      void set_power_level(uint8_t level); // 0-31
      void set_promiscuous_mode(bool promiscuous) { this->promiscuous_mode_ = promiscuous; }
      void set_rx_bandwidth(uint32_t bandwidth);
      void set_sync_word(const std::vector<uint8_t> &sync_word);
      void set_sync_word(uint8_t *sync_bytes, uint8_t length);
      void set_packet_length(uint8_t length);
      void set_variable_length_mode(bool variable);

      // actual interaction with radio:
      bool packet_available();
      uint8_t get_rssi() { return read_register_raw_(REG_RSSIVALUE); }
      uint8_t get_irq_flags1() { return read_register_raw_(REG_IRQFLAGS1); }
      uint8_t get_irq_flags2() { return read_register_raw_(REG_IRQFLAGS2); }
      RadioStatus get_radio_status();
      std::vector<uint8_t> read_packet();

    protected:
      // protected variables
      bool detected_{false};                // whether the device was detected during setup
      bool raw_codes_{false};               // whether to show raw register values in dump_config
      uint32_t pll_timeout_ms_{50};         // timeout for PLL lock in ms
      InternalGPIOPin *reset_pin_{nullptr}; // optional reset pin
      uint8_t version_{0};                  // version read from REG_VERSION
      uint32_t frequency_{868000000};       // default frequency 868 MHz
      bool promiscuous_mode_{false};        // whether promiscuous mode is enabled

      // unsafe methods that do not check PLL lock and will be encapsulated in safe methods
      void set_opmode_(uint8_t mode); // set the operation mode
      void set_promiscuous_mode_(bool promiscuous);

      // Set and get methods for the module configuration
      void configure_rfm69x(); // configure the module with current settings
      void reset_rfm69x();     // reset the module via reset pin if defined

      uint32_t get_frequency_() const { return this->frequency_; }
      uint32_t get_frequency_actual(); // get actual frequency based on FRF registers
      bool get_promiscuous_mode_() const { return this->promiscuous_mode_; }

      // Low-level register access
      uint8_t read_register_(uint8_t addr);
      void write_register_(uint8_t addr, uint8_t value);
      uint8_t read_register_raw_(uint8_t addr);
      void write_register_raw_(uint8_t addr, uint8_t value);
      bool wait_for_pll_lock_(uint32_t timeout_ms = 100);

      // helpful methods
      bool is_verbose_();
      const char *decode_opmode_(uint8_t opmode);
      std::string decode_irqflags1_(uint8_t val);
      std::string decode_irqflags2_(uint8_t val);
      bool test_pll_lock();
      void test_spi_communication();
    };

  } // namespace rfm69x
} // namespace esphome

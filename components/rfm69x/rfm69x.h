#pragma once

#include <atomic>
#include <functional>
#include <string>
#include <vector>

#include <esphome/core/component.h>
#include <esphome/components/spi/spi.h>
#include <esphome/core/helpers.h>

#include "rfm69x_reg.h"

namespace esphome
{
  namespace rfm69x
  {
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

      // ------------------------------------------------------------------
      // Basic configuration
      // ------------------------------------------------------------------
      void set_raw_codes(bool raw) { this->raw_codes_ = raw; }
      void set_reset_pin(InternalGPIOPin *reset_pin) { this->reset_pin_ = reset_pin; };
      void set_pll_timeout(uint32_t timeout_ms) { this->pll_timeout_ms_ = timeout_ms; }

      // ------------------------------------------------------------------
      // IRQ / event helpers
      // ------------------------------------------------------------------
      void set_irq_gpio(int gpio_num); // configure which GPIO the radio DIO is connected to
      void enable_irq(bool enable);    // enable/disable irq handling in driver
      void handle_isr_deferred();      // call this from component loop to service pending irq
      void set_on_packet_callback(std::function<void(const std::vector<uint8_t> &)> cb);

      // ------------------------------------------------------------------
      // Component lifecycle
      // ------------------------------------------------------------------
      void setup() override;
      void loop() override;
      void dump_config() override;

      // ------------------------------------------------------------------
      // Radio configuration (public API)
      // ------------------------------------------------------------------
      void set_frequency(uint32_t freq); // set frequency in Hz, e.g. 868000000
      void set_frequency_deviation(uint32_t frequency_deviation);
      void set_mode(RFM69ModeType mode);
      void set_mode_rx();
      void set_mode_tx();
      void set_mode_standby();
      void set_mode_sleep();
      void set_modulation(RFM69Modulation mod,
                          RFM69DataMode mode,
                          RFM69Shaping shaping);
      void set_bitrate(uint32_t bps);
      void set_power_level(uint8_t level); // 0-31
      void set_promiscuous_mode(bool promiscuous);
      void set_rx_bandwidth(uint32_t bandwidth);
      void set_sync_word(const std::vector<uint8_t> &sync_word);
      void set_sync_word(uint8_t *sync_bytes, uint8_t length);
      void set_packet_length(uint8_t length);
      void set_variable_length_mode(bool variable);
      void enable_crc(bool crc);

      // ------------------------------------------------------------------
      // Packet handling (public API)
      // ------------------------------------------------------------------
      bool packet_available();
      uint8_t get_rssi() { return read_register_(REG_RSSIVALUE); }
      uint8_t get_irq_flags1() { return read_register_(REG_IRQFLAGS1); }
      uint8_t get_irq_flags2() { return read_register_(REG_IRQFLAGS2); }
      RadioStatus get_radio_status();
      std::vector<uint8_t> read_packet();

      // ------------------------------------------------------------------
      // ISR bridging helpers
      // ------------------------------------------------------------------
      void isr_set_packet_ready();
      static void gpio_isr_wrapper(void *arg);

    protected:
      // ------------------------------------------------------------------
      // Core driver state
      // ------------------------------------------------------------------
      bool detected_{false};
      bool raw_codes_{false};
      uint32_t pll_timeout_ms_{50};
      InternalGPIOPin *reset_pin_{nullptr};
      uint8_t version_{0};
      uint32_t frequency_{868000000};
      bool promiscuous_mode_{false};
      esphome::Mutex spi_mutex_;
      std::atomic<bool> packet_ready_{false};
      std::atomic<bool> irq_enabled_{false};
      int irq_gpio_{-1};
      std::function<void(const std::vector<uint8_t> &)> on_packet_cb_;

      // ------------------------------------------------------------------
      // Packet handling (private helpers)
      // ------------------------------------------------------------------
      std::vector<uint8_t> read_packet_locked();

      // ------------------------------------------------------------------
      // Protected radio helpers
      // ------------------------------------------------------------------
      void set_opmode_(uint8_t mode);
      void set_promiscuous_mode_(bool promiscuous);

      // ------------------------------------------------------------------
      // Protected configuration helpers
      // ------------------------------------------------------------------
      void configure_rfm69x();
      void reset_rfm69x();

      // ------------------------------------------------------------------
      // Internal configuration accessors
      // ------------------------------------------------------------------
      uint32_t get_frequency_() const { return this->frequency_; }
      uint32_t get_frequency_actual();
      bool get_promiscuous_mode_() const { return this->promiscuous_mode_; }

      // ------------------------------------------------------------------
      // Low-level register access
      // ------------------------------------------------------------------
      uint8_t read_register_(uint8_t addr);
      void write_register_(uint8_t addr, uint8_t value);
      uint8_t read_register_raw_(uint8_t addr);
      void write_register_raw_(uint8_t addr, uint8_t value);
      bool wait_for_pll_lock_(uint32_t timeout_ms = 100);

      // ------------------------------------------------------------------
      // Decoding helpers
      // ------------------------------------------------------------------
      const char *decode_opmode_(uint8_t opmode);
      std::string decode_irqflags1_(uint8_t val);
      std::string decode_irqflags2_(uint8_t val);

      bool is_verbose_();

      // ------------------------------------------------------------------
      // Diagnostics helpers
      // ------------------------------------------------------------------
      bool test_pll_lock();
      void test_spi_communication();
    };

  } // namespace rfm69x
} // namespace esphome

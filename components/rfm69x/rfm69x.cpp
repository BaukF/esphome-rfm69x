#include "rfm69x.h"
#include "rfm69x_reg.h"
#include <esphome/core/log.h>
#include <esphome/core/helpers.h>

/* This file implements basic RFM69 functionality: first it was experimental,
 * but method by method I'm trying to align the code to the code that is presented
 * in the sx128x component of esphome. Hopefully this will make it easier to integrate
 * RFM69 support into esphome in the future.
 *
 * RFM69 is a very popular low-cost sub-GHz radio module, and it would be great
 * to have it supported in esphome.
 * The RFM69 is used in many applications, including the DucoBox heating controller
 * from Duco, which is quite popular in the Netherlands. But that is another component.
 *
 * TODO:
 * - Implement sending packets
 * - Implement interrupts for packet reception
 * - Implement more configuration options
 * - Implement power management (sleep modes)
 * - Add examples and documentation
 * - Temperature sensor reading (RFM69HCW)
 * - Hardware protection (RFM69HCW)
 */

#define IS_VERBOSE (esp_log_level_get(TAG) >= ESPHOME_LOG_LEVEL_VERBOSE)
namespace esphome
{
  namespace rfm69x
  {

    static const char *TAG = "rfm69x.component";

    static const char *mode_to_string_(RFM69ModeType mode)
    {
      switch (mode)
      {
      case MODE_SLEEP:
        return "SLEEP";
      case MODE_STANDBY:
        return "STANDBY";
      case MODE_FS:
        return "FS (Frequency Synthesis)";
      case MODE_RX:
        return "RX";
      case MODE_TX:
        return "TX";
      default:
        return "UNKNOWN";
      }
    }
    static uint8_t mode_to_opmode_(RFM69ModeType mode)
    {
      switch (mode)
      {
      case MODE_SLEEP:
        return OPMODE_SLEEP;
      case MODE_STANDBY:
        return OPMODE_STANDBY;
      case MODE_FS:
        return OPMODE_FS;
      case MODE_RX:
        return OPMODE_RX;
      case MODE_TX:
        return OPMODE_TX;
      default:
        return OPMODE_STANDBY;
      }
    }

    // --------------------------------------------------------------------
    // IRQ management (public API)
    // --------------------------------------------------------------------

    void RFM69x::set_irq_gpio(int gpio_num)
    {
      this->irq_gpio_ = gpio_num;
      ESP_LOGD(TAG, "set_irq_gpio(%d)", gpio_num);
    }

    void RFM69x::enable_irq(bool enable)
    {
      this->irq_enabled_.store(enable ? true : false);
      ESP_LOGD(TAG, "enable_irq(%s)", enable ? "true" : "false");

      // When enabling: clear pending flag to avoid spurious immediate reads
      if (enable)
      {
        this->packet_ready_.store(false, std::memory_order_release);
      }
      // NOTE: actual platform-specific ISR attach/detach should be performed by
      // platform glue (see examples below). This keeps driver portable.
    }

    void RFM69x::handle_isr_deferred()
    {
      // Quick exits: IRQ handling disabled or nothing pending
      if (!this->irq_enabled_.load(std::memory_order_acquire))
        return;
      if (!this->packet_ready_.load(std::memory_order_acquire))
        return;

      // Lock SPI to read IRQ registers safely and to perform any eventual FIFO read
      esphome::LockGuard lock(this->spi_mutex_);

      // Read IRQ registers (raw reads because we're within the lock)
      uint8_t irq1 = this->read_register_raw_(REG_IRQFLAGS1);
      uint8_t irq2 = this->read_register_raw_(REG_IRQFLAGS2);
      ESP_LOGD(TAG, "handle_isr_deferred: IRQ1=0x%02X IRQ2=0x%02X", irq1, irq2);

      if (irq2 & IRQ2_PAYLOAD_READY)
      {
        if (this->on_packet_cb_)
        {
          // Driver-eager: read the packet here (we already hold the lock) and invoke callback.
          std::vector<uint8_t> pkt = this->read_packet_locked();
          if (!pkt.empty())
          {
            // Keep callback work fast; if heavier processing is needed, copy pkt and queue it.
            this->on_packet_cb_(pkt);
          }
          // read_packet_locked cleared packet_ready_
        }
        else
        {
          // Leave packet_ready_ set so higher-level code will call read_packet()
          this->packet_ready_.store(true, std::memory_order_release);
        }
      }
      else
      {
        // other events (FIFO overrun etc.)
        if (irq2 & IRQ2_FIFO_OVERRUN)
        {
          ESP_LOGW(TAG, "FIFO OVERRUN detected in deferred handler");
        }
        // Clear pending flag to avoid repeated wakeups
        this->packet_ready_.store(false, std::memory_order_release);
      }
    }

    void RFM69x::set_on_packet_callback(std::function<void(const std::vector<uint8_t> &)> cb)
    {
      this->on_packet_cb_ = cb;
      ESP_LOGD(TAG, "set_on_packet_callback(%s)", this->on_packet_cb_ ? "set" : "cleared");
    }

    // --------------------------------------------------------------------
    // Component lifecycle
    // --------------------------------------------------------------------

    // start default methods for esphome component
    void RFM69x::setup()
    {
      if (IS_VERBOSE)
      {
        ESP_LOGW(TAG, "=== setup() START ===");
      }

      ESP_LOGW(TAG, "Initializing RFM69x...");

      this->spi_setup();
      this->reset_rfm69x();

      this->version_ = this->read_register_(REG_VERSION);

      if (this->version_ == 0x24 || this->version_ == 0x22)
      {
        this->detected_ = true;
        ESP_LOGW(TAG, "Detected RFM69, version=0x%02X", this->version_);
        ESP_LOGW(TAG, "About to call configure_rfm69x()");
        this->configure_rfm69x();
        delay(10); // wait a bit as this is first setup
      }
      else
      {
        this->detected_ = false;
        ESP_LOGW(TAG, "RFM69 not detected (last read=0x%02X)", this->version_);
      }
    }

    void RFM69x::loop()
    {
      // Static variables persist between loop() calls
      static uint32_t last_debug_dump = 0;
      static bool reset_done = false;

      uint32_t now = millis();

      // VERBOSE: Periodic debug dumps (expensive - only in verbose mode)
      if (is_verbose_())
      {
        if (now - last_debug_dump > 10000)
        { // Every 10 seconds
          ESP_LOGV(TAG, "=== Periodic Status Check ===");
          uint8_t rssi = this->read_register_(REG_RSSIVALUE);
          uint8_t irq1 = this->read_register_(REG_IRQFLAGS1);
          uint8_t irq2 = this->read_register_(REG_IRQFLAGS2);

          ESP_LOGV(TAG, "RSSI: -%d dBm", rssi / 2);
          ESP_LOGV(TAG, "IRQ1: 0x%02X (%s)", irq1, decode_irqflags1_(irq1).c_str());
          ESP_LOGV(TAG, "IRQ2: 0x%02X (%s)", irq2, decode_irqflags2_(irq2).c_str());

          last_debug_dump = now;
        }
      }

      // VERBOSE: 60-second reset test (expensive - only in verbose mode)
      if (is_verbose_() && !reset_done && now > 60000)
      {
        ESP_LOGV(TAG, "=== 60 SECOND RESET TEST STARTING ===");
        this->test_spi_communication();
        this->reset_rfm69x();
        this->configure_rfm69x();
        this->test_pll_lock();
        ESP_LOGV(TAG, "=== 60 SECOND RESET TEST COMPLETE ===");
        reset_done = true;
      }

      // If IRQ handling is enabled and a deferred handler should run, run it here.
      // This will decode IRQ registers and optionally call callbacks.
      this->handle_isr_deferred();
    }

    void RFM69x::dump_config()
    {
      ESP_LOGCONFIG(TAG, "RFM69x:");
      LOG_PIN("  CS Pin: ", this->cs_);

      if (this->reset_pin_ != nullptr)
      {
        LOG_PIN("  Reset Pin: ", this->reset_pin_);
      }

      if (this->detected_)
      {
        // Always show basic status
        ESP_LOGCONFIG(TAG, "  Status: DETECTED");
        ESP_LOGCONFIG(TAG, "  Chip Version: 0x%02X (%s)",
                      this->version_,
                      this->version_ == 0x24 ? "RFM69HCW" : this->version_ == 0x22 ? "RFM69CW"
                                                                                   : "Unknown");

        // VERBOSE: Read full status with all registers
        if (is_verbose_())
        {
          ESP_LOGV(TAG, "Reading full radio status...");
          auto status = this->get_radio_status();

          ESP_LOGCONFIG(TAG, "  Operating Mode: %s", status.mode.c_str());
          ESP_LOGCONFIG(TAG, "  Frequency: %.3f MHz", status.frequency_mhz);
          ESP_LOGCONFIG(TAG, "  RSSI: %d dBm", status.rssi_dbm);
          ESP_LOGCONFIG(TAG, "  PLL Lock: %s", status.pll_locked ? "YES" : "NO");
          ESP_LOGCONFIG(TAG, "  IRQ Flags 1: %s", status.irq1_flags.c_str());
          ESP_LOGCONFIG(TAG, "  IRQ Flags 2: %s", status.irq2_flags.c_str());
        }
        else
        {
          // Quick status without reading all registers
          ESP_LOGCONFIG(TAG, "  Frequency: %.3f MHz (cached)", this->frequency_ / 1e6);
        }

        // VERBOSE: Detailed modulation and packet config
        if (is_verbose_())
        {
          this->enable();
          uint8_t datamodul = this->read_register_raw_(REG_DATAMODUL);
          uint8_t packet_config = this->read_register_raw_(REG_PACKETCONFIG1);
          uint8_t sync_config = this->read_register_raw_(REG_SYNCCONFIG);
          this->disable();

          std::string mod_type = (datamodul & DATAMODUL_MODULATION_MASK) ? "OOK" : "FSK";
          std::string data_mode;
          uint8_t mode_bits = datamodul & DATAMODUL_MODE_MASK;
          if (mode_bits == DATAMODUL_PACKET_MODE)
            data_mode = "Packet";
          else if (mode_bits == DATAMODUL_CONTINUOUS_SYNC)
            data_mode = "Continuous+Sync";
          else if (mode_bits == DATAMODUL_CONTINUOUS_NOSYNC)
            data_mode = "Continuous NoSync";
          else
            data_mode = "Unknown";

          ESP_LOGCONFIG(TAG, "  Modulation: %s, Mode: %s",
                        mod_type.c_str(), data_mode.c_str());

          // ... rest of detailed config ...
          ESP_LOGCONFIG(TAG, "  Packet Format: %s",
                        (packet_config & 0x80) ? "Variable Length" : "Fixed Length");
          ESP_LOGCONFIG(TAG, "  CRC: %s",
                        (packet_config & 0x10) ? "Enabled" : "Disabled");
        }

        // Always show health summary
        ESP_LOGCONFIG(TAG, "  Radio Status: OK");

        //---- temporary sync test
        this->enable();
        uint8_t sync_config = this->read_register_raw_(REG_SYNCCONFIG);
        std::string sync_bytes_str;
        uint8_t sync_len = ((sync_config >> 3) & 0x07) + 1;
        for (uint8_t i = 0; i < sync_len; i++)
        {
          uint8_t sync_byte = this->read_register_raw_(REG_SYNCVALUE1 + i);
          char hex[4];
          snprintf(hex, sizeof(hex), "%02X ", sync_byte);
          sync_bytes_str += hex;
        }
        this->disable();

        ESP_LOGCONFIG(TAG, "  Sync Word: %d bytes [%s]", sync_len, sync_bytes_str.c_str());
        //-----
      }
      else
      {
        ESP_LOGE(TAG, "  Status: NOT DETECTED");
      }
    }

    // --------------------------------------------------------------------
    // Radio configuration (public API)
    // --------------------------------------------------------------------
    // Writing the frequency touches three registers and requires a PLL lock check.
    /// @brief Frequency setter with PLL lock check,
    /// @param freq e.g. 868000000, this is fixed for 868 MHz set
    void RFM69x::set_frequency(uint32_t freq)
    {
      this->frequency_ = freq;

      if (is_verbose_())
      {
        ESP_LOGV(TAG, ">> set_frequency(%.3f MHz) - START", freq / 1e6);
      }

      constexpr double FSTEP = 32000000.0 / 524288.0;
      uint32_t frf = (uint32_t)(freq / FSTEP);

      set_opmode_(OPMODE_STANDBY);
      delay(10);

      if (is_verbose_())
      {
        ESP_LOGV(TAG, "   Writing FRF registers: 0x%06X", frf);
      }

      this->enable();
      write_register_raw_(REG_FRFMSB, (uint8_t)(frf >> 16));
      write_register_raw_(REG_FRFMID, (uint8_t)(frf >> 8));
      write_register_raw_(REG_FRFLSB, (uint8_t)(frf));
      this->disable();

      set_opmode_(OPMODE_FS);
      delay(15);

      if (is_verbose_())
      {
        ESP_LOGV(TAG, "   Waiting for PLL lock...");
      }

      bool plllock = wait_for_pll_lock_(pll_timeout_ms_);

      if (plllock)
      {
        ESP_LOGI(TAG, "Successfully set frequency to %.3f MHz", freq / 1e6);
      }
      else
      {
        ESP_LOGE(TAG, "PLL failed to lock after frequency set");
      }

      if (is_verbose_())
      {
        ESP_LOGV(TAG, "<< set_frequency() - END");
      }

      set_opmode_(OPMODE_STANDBY);
    }

    // Adjusting the frequency deviation touches two registers; keep SPI toggles minimal.

    void RFM69x::set_frequency_deviation(uint32_t frequency_deviation)
    {
      set_opmode_(OPMODE_STANDBY);
      delay(10);

      uint16_t fdev_reg = ((uint64_t)frequency_deviation << 19) / 32000000;

      this->enable();
      write_register_raw_(REG_FDEVMSB, (fdev_reg >> 8) & 0xFF);
      write_register_raw_(REG_FDEVLSB, fdev_reg & 0xFF);
      this->disable();

      delay(1);

      ESP_LOGD(TAG, "Set frequency deviation: %.2f kHz [FDEV=0x%04X]",
               frequency_deviation / 1000.0, fdev_reg);
    }

    void RFM69x::set_mode(RFM69ModeType mode)
    {
      if (is_verbose_())
      {
        ESP_LOGV(TAG, ">> set_mode(%s) - START", mode_to_string_(mode));
      }

      // Step 1: Always go to Standby first (safe state)
      if (mode != MODE_STANDBY)
      {
        set_opmode_(OPMODE_STANDBY);
        delay(10);
      }

      // Step 2: Set target mode
      uint8_t target_opmode = mode_to_opmode_(mode);
      set_opmode_(target_opmode);

      if (is_verbose_())
      {
        ESP_LOGV(TAG, "   Mode register set to %s", mode_to_string_(mode));
      }

      // Step 3: For RX/TX modes, wait for PLL lock
      bool pll_locked = true; // Assume success for non-RF modes

      if (mode == MODE_RX || mode == MODE_TX)
      {
        if (is_verbose_())
        {
          ESP_LOGV(TAG, "   Waiting for PLL lock in %s mode...", mode_to_string_(mode));
        }

        pll_locked = wait_for_pll_lock_(pll_timeout_ms_);

        if (pll_locked)
        {
          ESP_LOGI(TAG, "Set mode to %s - PLL locked", mode_to_string_(mode));
          if (is_verbose_())
          {
            ESP_LOGV(TAG, "<< set_mode(%s) - SUCCESS", mode_to_string_(mode));
          }
        }
        else
        {
          ESP_LOGE(TAG, "PLL failed to lock after setting %s mode", mode_to_string_(mode));
          if (is_verbose_())
          {
            ESP_LOGV(TAG, "<< set_mode(%s) - FAILED (PLL lock timeout)", mode_to_string_(mode));
          }
        }
      }
      else
      {
        // Non-RF modes (Sleep, Standby) don't need PLL
        ESP_LOGD(TAG, "Set mode to %s", mode_to_string_(mode));
        if (is_verbose_())
        {
          ESP_LOGV(TAG, "<< set_mode(%s) - SUCCESS (no PLL required)", mode_to_string_(mode));
        }
      }
    }

    void RFM69x::set_mode_rx()
    {
      this->set_mode(MODE_RX);
    }

    void RFM69x::set_mode_tx()
    {
      this->set_mode(MODE_TX);
    }

    void RFM69x::set_mode_standby()
    {
      this->set_mode(MODE_STANDBY);
    }

    void RFM69x::set_mode_sleep()
    {
      this->set_mode(MODE_SLEEP);
    }

    void RFM69x::set_modulation(RFM69Modulation mod,
                                RFM69DataMode mode,
                                RFM69Shaping shaping)
    {
      uint8_t reg_value = 0;

      // Data mode
      switch (mode)
      {
      case RFM69_PACKET_MODE:
        reg_value |= DATAMODUL_PACKET_MODE;
        break;
      case RFM69_CONTINUOUS_SYNC:
        reg_value |= DATAMODUL_CONTINUOUS_SYNC;
        break;
      case RFM69_CONTINUOUS_NOSYNC:
        reg_value |= DATAMODUL_CONTINUOUS_NOSYNC;
        break;
      }

      // Modulation type
      reg_value |= (mod == RFM69_FSK) ? DATAMODUL_FSK : DATAMODUL_OOK;

      // Shaping - now properly mapped
      switch (shaping)
      {
      case RFM69_SHAPING_NONE:
        reg_value |= DATAMODUL_SHAPING_NONE;
        break;
      case RFM69_SHAPING_GAUSSIAN_BT_1_0:
        reg_value |= DATAMODUL_SHAPING_BT_1_0;
        break;
      case RFM69_SHAPING_GAUSSIAN_BT_0_5:
        reg_value |= DATAMODUL_SHAPING_BT_0_5;
        break;
      case RFM69_SHAPING_GAUSSIAN_BT_0_3:
        reg_value |= DATAMODUL_SHAPING_BT_0_3;
        break;
      }
      this->write_register_(REG_DATAMODUL, reg_value);
    }

    void RFM69x::set_bitrate(uint32_t bps)
    {
      uint16_t bitrate_reg = 32000000 / bps;

      this->enable();
      write_register_raw_(REG_BITRATEMSB, (bitrate_reg >> 8) & 0xFF);
      write_register_raw_(REG_BITRATELSB, bitrate_reg & 0xFF);
      this->disable();

      ESP_LOGD(TAG, "Set bitrate: %u bps", bps);
    }

    // Power level 0-31: 0 = -18dBm, 31 = +13dBm (RFM69CW/HCW)
    // TODO: Add support for PA Boost (PA1+PA2) for +20dBm
    // TODO: add method to read current power level
    void RFM69x::set_power_level(uint8_t level)
    {
      uint8_t val = level & 0x1F; // PA level is 5 bits
      this->write_register_(REG_PALEVEL, val);
      ESP_LOGD(TAG, "Set PA level: %u", val);
    }

    void RFM69x::set_rx_bandwidth(uint32_t bandwidth)
    {
      // RX BW register calculation for RFM69
      // Target: 101.5625 kHz per Arne's data
      // REG_RXBW format: bits [7:5] DccFreq, bits [4:3] RxBwMant, bits [2:0] RxBwExp

      // For ~100 kHz: Mant=16 (0b10), Exp=2 (0b010)
      // This gives: BW = F_XOSC / (RxBwMant * 2^(RxBwExp + 2))
      //           = 32MHz / (16 * 2^4) = 32MHz / 256 = 125 kHz

      // For closer to 101.5625: Mant=20 (0b01), Exp=2
      //           = 32MHz / (20 * 2^4) = 32MHz / 320 = 100 kHz

      uint8_t reg_value = 0x42; // DccFreq=010, Mant=01 (20), Exp=010 (2)
      this->write_register_(REG_RXBW, reg_value);
      ESP_LOGD(TAG, "Set RX bandwidth: ~100 kHz [REG_RXBW=0x%02X]", reg_value);
    }

    void RFM69x::set_sync_word(const std::vector<uint8_t> &sync_word)
    {
      if (sync_word.empty() || sync_word.size() > 8)
      {
        ESP_LOGE(TAG, "Invalid sync word length: %d (must be 1-8)", sync_word.size());
        return;
      }

      uint8_t sync_config = 0x80;
      sync_config |= ((sync_word.size() - 1) << 3);

      this->enable();
      this->write_register_raw_(REG_SYNCCONFIG, sync_config);
      for (size_t i = 0; i < sync_word.size(); i++)
      {
        this->write_register_raw_(REG_SYNCVALUE1 + i, sync_word[i]);
      }
      this->disable();

      ESP_LOGD(TAG, "Configured sync word: %d bytes", sync_word.size());
    }

    void RFM69x::set_sync_word(uint8_t *sync_bytes, uint8_t length)
    {
      if (sync_bytes == nullptr || length == 0 || length > 8)
      {
        ESP_LOGE(TAG, "Invalid sync word pointer/length");
        return;
      }
      std::vector<uint8_t> v(sync_bytes, sync_bytes + length);
      this->set_sync_word(v);
    }

    void RFM69x::set_packet_length(uint8_t length)
    {
      this->write_register_(REG_PAYLOADLENGTH, length);
      ESP_LOGI(TAG, "Set packet length: %d bytes", length);
    }

    void RFM69x::set_variable_length_mode(bool variable)
    {
      this->enable();
      uint8_t config = this->read_register_raw_(REG_PACKETCONFIG1);

      if (variable)
      {
        config |= PACKET1_FORMAT_VARIABLE;
      }
      else
      {
        config &= ~PACKET1_FORMAT_VARIABLE;
      }

      this->write_register_raw_(REG_PACKETCONFIG1, config);
      this->disable();

      ESP_LOGI(TAG, "Variable length mode: %s", variable ? "enabled" : "disabled");
    }

    void RFM69x::enable_crc(bool enabled)
    {
      this->enable();
      uint8_t config = this->read_register_raw_(REG_PACKETCONFIG1);
      if (enabled)
        config |= PACKET1_CRC_ON;
      else
        config &= ~PACKET1_CRC_ON;
      this->write_register_raw_(REG_PACKETCONFIG1, config);
      this->disable();
    }

    // --------------------------------------------------------------------
    // Packet handling (public API)
    // --------------------------------------------------------------------

    bool RFM69x::packet_available()
    {
      uint8_t irq_flags = this->read_register_(REG_IRQFLAGS2);
      return (irq_flags & IRQ2_PAYLOAD_READY);
    }

    RFM69x::RadioStatus RFM69x::get_radio_status()
    {
      RadioStatus status;
      status.detected = this->detected_;
      status.version = this->version_;

      if (!this->detected_)
      {
        status.mode = "Not Detected";
        status.frequency_mhz = 0.0f;
        status.rssi_dbm = -128;
        status.pll_locked = false;
        status.irq1_flags = "N/A";
        status.irq2_flags = "N/A";
        return status;
      }

      // Use cached frequency (reliable and fast)
      status.frequency_mhz = this->frequency_ / 1e6f;

      // ONE transaction for all hardware reads
      this->enable();

      uint8_t opmode = this->read_register_raw_(REG_OPMODE);
      status.mode = this->decode_opmode_(opmode);

      uint8_t rssi_raw = this->read_register_raw_(REG_RSSIVALUE);
      status.rssi_dbm = -(rssi_raw / 2);

      uint8_t irq1 = this->read_register_raw_(REG_IRQFLAGS1);
      status.pll_locked = (irq1 & IRQ1_PLLLOCK);
      status.irq1_flags = this->decode_irqflags1_(irq1);

      uint8_t irq2 = this->read_register_raw_(REG_IRQFLAGS2);
      status.irq2_flags = this->decode_irqflags2_(irq2);

      this->disable();

      return status;
    }

    // --------------------------------------------------------------------
    // Packet handling (private helpers)
    // --------------------------------------------------------------------
    // Low-level FIFO read variant that assumes the caller already holds spi_mutex_.
    // This avoids nested locking with esphome::Mutex (non-recursive).
    std::vector<uint8_t> RFM69x::read_packet_locked()
    {
      std::vector<uint8_t> packet;

      // Caller must hold spi_mutex_ (assert is optional)
      // Read payload length and payload from FIFO (chip select must be handled by helpers)
      this->enable();
      uint8_t length = this->read_register_raw_(REG_FIFO);
      if (length == 0)
      {
        ESP_LOGW(TAG, "read_packet_locked: length==0");
        this->disable();
        this->packet_ready_.store(false, std::memory_order_release);
        return packet;
      }
      if (length > 255)
      {
        ESP_LOGW(TAG, "read_packet_locked: suspicious length %u, clamping to 255", length);
        length = 255;
      }
      packet.reserve(length);
      for (uint8_t i = 0; i < length; ++i)
      {
        packet.push_back(this->read_register_raw_(REG_FIFO));
      }

      uint8_t irq2 = this->read_register_raw_(REG_IRQFLAGS2);
      if (irq2 & IRQ2_CRC_OK)
      {
        ESP_LOGD(TAG, "read_packet_locked: CRC OK");
      }
      else
      {
        ESP_LOGD(TAG, "read_packet_locked: CRC not OK (IRQ2=0x%02X)", irq2);
      }

      this->disable();

      // consumed the packet
      this->packet_ready_.store(false, std::memory_order_release);
      return packet;
    }

    // Public read_packet() keeps behaviour but uses the locked variant internally.
    std::vector<uint8_t> RFM69x::read_packet()
    {
      esphome::LockGuard lock(this->spi_mutex_);
      return this->read_packet_locked();
    }

    // --------------------------------------------------------------------
    // Protected radio helpers
    // --------------------------------------------------------------------

    // This method assumes you have already enabled the radio (chip select low)
    // It sets the OPMODE register directly
    void RFM69x::set_opmode_(uint8_t mode)
    {
      // Read current mode
      uint8_t opmode = this->read_register_(REG_OPMODE);

      // Set sequencer off + new mode
      opmode = OPMODE_SEQUENCER_OFF | mode;

      // Write new mode
      this->write_register_(REG_OPMODE, opmode);

      // WAIT for ModeReady flag
      uint32_t start = millis();
      while (millis() - start < 100)
      {
        uint8_t irq1 = this->read_register_(REG_IRQFLAGS1);
        if (irq1 & IRQ1_MODEREADY)
        {
          break;
        }
        delay(1);
      }

      // Verify
      uint8_t readback = this->read_register_(REG_OPMODE);
      if (readback != opmode)
      {
        ESP_LOGE(TAG, "Mode write failed! Tried 0x%02X, read 0x%02X", opmode, readback);
      }
    }

    // needs rework to use the same method as rest of this code.
    void RFM69x::set_promiscuous_mode_(bool promiscuous)
    {

      this->promiscuous_mode_ = promiscuous;

      uint8_t val = this->read_register_raw_(REG_PACKETCONFIG1);
      if (promiscuous)
      {
        val |= 0x04; // Set bit 2
        ESP_LOGD(TAG, "Promiscuous mode enabled");
      }
      else
      {
        val &= ~0x04; // Clear bit 2
        ESP_LOGD(TAG, "Promiscuous mode disabled");
      }
      this->enable();
      this->write_register_raw_(REG_PACKETCONFIG1, val);
      this->disable();
    }

    // --------------------------------------------------------------------
    // Protected configuration helpers
    // --------------------------------------------------------------------

    void RFM69x::configure_rfm69x()
    {
      ESP_LOGW(TAG, ">>> configure_rfm69x() START");

      // Always set frequency (even if using default)
      ESP_LOGW(TAG, "About to set frequency to %.3f MHz", this->frequency_ / 1e6);
      set_frequency(this->frequency_);
      ESP_LOGW(TAG, "Frequency setting completed");

      // Set default modulation
      ESP_LOGW(TAG, "Setting modulation...");
      set_modulation(RFM69_FSK, RFM69_PACKET_MODE, RFM69_SHAPING_NONE);

      // Set default bitrate
      ESP_LOGW(TAG, "Setting bitrate...");
      set_bitrate(4800);

      // Set default power
      ESP_LOGW(TAG, "Setting power...");
      set_power_level(0);

      // Configure promiscuous mode if enabled
      ESP_LOGW(TAG, "Setting promiscuous mode...");
      set_promiscuous_mode_(this->promiscuous_mode_);

      ESP_LOGW(TAG, "<<< configure_rfm69x() END");
    }

    void RFM69x::reset_rfm69x()
    {
      if (this->reset_pin_ != nullptr)
      {
        ESP_LOGD(TAG, "Hardware reset via reset pin...");
        this->reset_pin_->digital_write(true); // Active high reset
        delay(10);
        this->reset_pin_->digital_write(false); // Release reset
        delay(10);                              // Wait for chip to stabilize

        // Verify chip is responding
        uint8_t version = this->read_register_(REG_VERSION);
        ESP_LOGD(TAG, "Version after reset: 0x%02X", version);
      }
      else
      {
        ESP_LOGW(TAG, "Reset pin not defined, performing soft reset via register");

        // Software reset: write to REG_OPMODE with sequencer off, sleep mode
        this->write_register_(REG_OPMODE, OPMODE_SEQUENCER_OFF | OPMODE_SLEEP);
        delay(10);
        this->write_register_(REG_OPMODE, OPMODE_SEQUENCER_OFF | OPMODE_STANDBY);
        delay(10);
      }
    }

    // --------------------------------------------------------------------
    // Low-level register access
    // --------------------------------------------------------------------

    uint8_t RFM69x::read_register_(uint8_t addr)
    {
      this->enable();
      uint8_t value = this->read_register_raw_(addr);
      this->disable();
      return value;
    }

    uint8_t RFM69x::read_register_raw_(uint8_t addr)
    {
      this->write_byte(addr & 0x7F); // clear MSB for read
      uint8_t value = this->read_byte();
      return value;
    }

    void RFM69x::write_register_(uint8_t addr, uint8_t value)
    {
      this->enable();
      this->write_register_raw_(addr, value);
      this->disable();
    }

    void RFM69x::write_register_raw_(uint8_t addr, uint8_t value)
    {
      uint8_t buffer[2] = {static_cast<uint8_t>(addr | 0x80), value};
      this->write_array(buffer, 2);
    }

    bool RFM69x::wait_for_pll_lock_(uint32_t timeout_ms)
    {
      uint32_t start = millis();

      while (true)
      {
        // Each poll is a separate transaction
        this->enable();
        uint8_t flags = read_register_raw_(REG_IRQFLAGS1);
        this->disable();

        if (flags & IRQ1_PLLLOCK)
          return true;
        if ((millis() - start) >= timeout_ms)
          return false;

        delay(1); // Don't hammer the SPI bus
      }
    }

    // --------------------------------------------------------------------
    // Decoding helpers
    // --------------------------------------------------------------------
    const char *RFM69x::decode_opmode_(uint8_t opmode)
    {
      // Mask the opmode to isolate the Mode bits (bits 4, 3, 2).
      uint8_t mode = (opmode & 0x1C) >> 2;

      switch (mode)
      {
      case 0x00:
        return "Sleep Mode";
      case 0x01:
        return "Standby Mode";
      case 0x02:
        return "Synthesizer Mode";
      case 0x03:
        return "Transmitter Mode";
      case 0x04:
        return "Receiver Mode";
      default:
        return "Unknown Mode";
      }
    }

    static const struct
    {
      uint8_t mask;
      const char *name;
    } irq1_flags[] = {
        {IRQ1_MODEREADY, "ModeReady"},
        {IRQ1_RXREADY, "RxReady"},
        {IRQ1_TXREADY, "TxReady"},
        {IRQ1_PLLLOCK, "PLLLock"},
        {IRQ1_RSSI, "RSSI"},
        {IRQ1_TIMEOUT, "Timeout"},
        {IRQ1_AUTOMODE, "AutoMode"},
        {IRQ1_SYNCADDRESSMATCH, "SyncMatch"},
    };

    std::string RFM69x::decode_irqflags1_(uint8_t val)
    {
      std::string res;
      for (auto &f : irq1_flags)
      {
        if (val & f.mask)
        {
          if (!res.empty())
            res += " | ";
          res += f.name; // safe: const char* appended into std::string
        }
      }
      return res.empty() ? "None" : res;
    }

    static const struct
    {
      uint8_t mask;
      const char *name;
    } irq2_flags[] = {
        {IRQ2_FIFO_FULL, "FifoFull"},
        {IRQ2_FIFO_NOT_EMPTY, "FifoNotEmpty"},
        {IRQ2_FIFO_LEVEL, "FifoLevel"},
        {IRQ2_FIFO_OVERRUN, "FifoOverrun"},
        {IRQ2_PACKET_SENT, "PacketSent"},
        {IRQ2_PAYLOAD_READY, "PayloadReady"},
        {IRQ2_CRC_OK, "CRC_OK"},
        {IRQ2_LOW_BAT, "LowBat"},
    };

    std::string RFM69x::decode_irqflags2_(uint8_t val)
    {
      std::string res;
      for (auto &f : irq2_flags)
      {
        if (val & f.mask)
        {
          if (!res.empty())
            res += " | ";
          res += f.name; // safe: const char* appended into std::string
        }
      }
      return res.empty() ? "None" : res;
    }

    // Helper - returns true only if verbose logging is enabled
    bool RFM69x::is_verbose_()
    {
      return esp_log_level_get(TAG) >= ESP_LOG_VERBOSE;
    }

    // --------------------------------------------------------------------
    // Diagnostics helpers
    // --------------------------------------------------------------------

    bool RFM69x::test_pll_lock()
    {
      if (!is_verbose_())
      {
        ESP_LOGD(TAG, "PLL test skipped (enable VERBOSE logging to run)");
        return true; // Assume OK if not testing
      }

      ESP_LOGV(TAG, "=== Testing PLL Lock ===");
      set_opmode_(OPMODE_FS);
      delay(20);
      bool locked = wait_for_pll_lock_(100);

      if (locked)
      {
        ESP_LOGV(TAG, "✓ PLL LOCKED in FS mode!");
      }
      else
      {
        ESP_LOGE(TAG, "✗ PLL NOT LOCKED after 100ms");
      }

      set_opmode_(OPMODE_STANDBY);
      return locked;
    }

    void RFM69x::test_spi_communication()
    {
      if (!is_verbose_())
      {
        ESP_LOGD(TAG, "SPI test skipped (enable VERBOSE logging to run)");
        return;
      }

      ESP_LOGV(TAG, "=== Testing Basic SPI Communication ===");

      uint8_t version = this->read_register_(REG_VERSION);
      ESP_LOGV(TAG, "VERSION register: 0x%02X (expected 0x24)", version);

      uint8_t opmode_before = this->read_register_(REG_OPMODE);
      ESP_LOGV(TAG, "OPMODE before: 0x%02X (%s)", opmode_before,
               decode_opmode_(opmode_before));

      this->write_register_(REG_OPMODE, 0x80);
      delay(10);

      uint8_t opmode_after = this->read_register_(REG_OPMODE);
      ESP_LOGV(TAG, "OPMODE after: 0x%02X (%s)", opmode_after,
               decode_opmode_(opmode_after));

      if (opmode_after == 0x80)
      {
        ESP_LOGV(TAG, "✓ SPI write successful!");
      }
      else
      {
        ESP_LOGE(TAG, "✗ SPI write FAILED");
      }

      ESP_LOGV(TAG, "=== SPI Communication Test Complete ===");
    }

    // --------------------------------------------------------------------
    // ISR bridging helpers
    // --------------------------------------------------------------------

    void IRAM_ATTR RFM69x::isr_set_packet_ready()
    {
      this->packet_ready_.store(true, std::memory_order_release);
    }

    void IRAM_ATTR RFM69x::gpio_isr_wrapper(void *arg)
    {
      if (!arg)
        return;
      auto drv = reinterpret_cast<RFM69x *>(arg);
      drv->isr_set_packet_ready();
    }

    extern "C" void IRAM_ATTR rfm69x_simple_isr(void *arg)
    {
      if (!arg)
        return;
      auto drv = reinterpret_cast<esphome::rfm69x::RFM69x *>(arg);
      drv->isr_set_packet_ready();
    }

  } // namespace rfm69x
} // namespace esphome

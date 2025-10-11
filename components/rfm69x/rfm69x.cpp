#include "rfm69x.h"
#include "rfm69x_reg.h"
#include "esphome/core/log.h"

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

namespace esphome
{
  namespace rfm69x
  {

    static const char *TAG = "rfm69x.component";

    // start default methods for esphome component
    void RFM69x::setup()
    {
      ESP_LOGD(TAG, "Initializing RFM69x...");

      // Prepare SPI
      this->spi_setup();

      this->reset_rfm69x();

      this->configure_rfm69x();
      delay(10); // Give it a moment to start

      this->enable(); // Acquire Lock
      // Probe version register
      this->version_ = this->read_register_raw_(REG_VERSION);
      this->disable(); // Release Lock

      if (this->version_ == 0x24 || this->version_ == 0x22)
      { // 0x24 = RFM69HCW, 0x22 = RFM69CW
        this->detected_ = true;
        ESP_LOGI(TAG, "Detected RFM69, version=0x%02X", this->version_);
        this->configure_rfm69x();
      }
      else
      {
        this->detected_ = false;
        ESP_LOGE(TAG, "RFM69 not detected (last read=0x%02X)", this->version_);
      }
    }

    void RFM69x::loop()
    {
      // No continuous work yet.
      // Later: you'll poll status or handle RadioLib here.
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
        // Get comprehensive status
        auto status = this->get_radio_status();

        ESP_LOGCONFIG(TAG, "  Status: DETECTED");
        ESP_LOGCONFIG(TAG, "  Chip Version: 0x%02X (%s)",
                      status.version,
                      status.version == 0x24 ? "RFM69HCW" : status.version == 0x22 ? "RFM69CW"
                                                                                   : "Unknown");

        ESP_LOGCONFIG(TAG, "  Operating Mode: %s", status.mode.c_str());
        ESP_LOGCONFIG(TAG, "  Frequency: %.3f MHz", status.frequency_mhz);
        ESP_LOGCONFIG(TAG, "  RSSI: %d dBm", status.rssi_dbm);
        ESP_LOGCONFIG(TAG, "  PLL Lock: %s", status.pll_locked ? "YES" : "NO");

        if (!status.pll_locked)
        {
          ESP_LOGW(TAG, "  WARNING: PLL not locked - check frequency configuration!");
        }
        ESP_LOGCONFIG(TAG, "  IRQ Flags 1: %s", status.irq1_flags.c_str());
        ESP_LOGCONFIG(TAG, "  IRQ Flags 2: %s", status.irq2_flags.c_str());

        // Configuration settings
        this->enable();
        uint8_t datamodul = this->read_register_raw_(REG_DATAMODUL);
        uint8_t packet_config = this->read_register_raw_(REG_PACKETCONFIG1);
        uint8_t sync_config = this->read_register_raw_(REG_SYNCCONFIG);

        // Decode data modulation
        std::string mod_type = (datamodul & 0x08) ? "OOK" : "FSK";
        std::string data_mode;
        uint8_t mode_bits = datamodul & 0x60;
        if (mode_bits == 0x00)
          data_mode = "Packet";
        else if (mode_bits == 0x40)
          data_mode = "Continuous+Sync";
        else
          data_mode = "Continuous NoSync";

        ESP_LOGCONFIG(TAG, "  Modulation: %s, Mode: %s", mod_type.c_str(), data_mode.c_str());

        // Decode shaping
        std::string shaping;
        uint8_t shaping_bits = datamodul & 0x03;
        switch (shaping_bits)
        {
        case 0x00:
          shaping = "None";
          break;
        case 0x01:
          shaping = "Gaussian BT=1.0";
          break;
        case 0x02:
          shaping = "Gaussian BT=0.5";
          break;
        case 0x03:
          shaping = "Gaussian BT=0.3";
          break;
        }
        ESP_LOGCONFIG(TAG, "  Pulse Shaping: %s", shaping.c_str());

        // Bitrate
        uint16_t bitrate_reg = (this->read_register_raw_(REG_BITRATEMSB) << 8) |
                               this->read_register_raw_(REG_BITRATELSB);
        uint32_t bitrate = 32000000 / bitrate_reg;
        ESP_LOGCONFIG(TAG, "  Bitrate: %u bps (%.2f kBaud)", bitrate, bitrate / 1000.0f);

        // Frequency deviation
        uint16_t fdev_reg = (this->read_register_raw_(REG_FDEVMSB) << 8) |
                            this->read_register_raw_(REG_FDEVLSB);
        uint32_t fdev = (fdev_reg * 32000000ULL) >> 19;
        ESP_LOGCONFIG(TAG, "  Frequency Deviation: %.2f kHz", fdev / 1000.0f);

        // Sync word configuration
        if (sync_config & 0x80)
        {
          uint8_t sync_size = ((sync_config >> 3) & 0x07) + 1;
          ESP_LOGCONFIG(TAG, "  Sync Word: ENABLED (%d bytes)", sync_size);

          std::string sync_hex;
          for (uint8_t i = 0; i < sync_size; i++)
          {
            uint8_t sync_byte = this->read_register_raw_(REG_SYNCVALUE1 + i);
            sync_hex += str_sprintf("%02X ", sync_byte);
          }
          ESP_LOGCONFIG(TAG, "    Bytes: %s", sync_hex.c_str());
        }
        else
        {
          ESP_LOGCONFIG(TAG, "  Sync Word: DISABLED");
        }

        // Packet configuration
        bool variable_length = (packet_config & 0x80) != 0;
        bool crc_on = (packet_config & 0x10) != 0;
        ESP_LOGCONFIG(TAG, "  Packet Format: %s", variable_length ? "Variable Length" : "Fixed Length");
        ESP_LOGCONFIG(TAG, "  CRC: %s", crc_on ? "Enabled" : "Disabled");
        ESP_LOGCONFIG(TAG, "  Promiscuous Mode: %s", this->promiscuous_mode_ ? "YES" : "NO");

        if (variable_length)
        {
          uint8_t max_len = this->read_register_raw_(REG_PAYLOADLENGTH);
          ESP_LOGCONFIG(TAG, "  Max Packet Length: %d bytes", max_len);
        }

        // Power settings
        uint8_t pa_level = this->read_register_raw_(REG_PALEVEL);
        uint8_t power = pa_level & 0x1F;
        int8_t power_dbm = -18 + power; // Simplified calculation
        ESP_LOGCONFIG(TAG, "  TX Power: %d (≈%d dBm)", power, power_dbm);

        this->disable();

        // Summary health check
        ESP_LOGCONFIG(TAG, "");
        if (status.pll_locked && status.mode == "Receiver Mode")
        {
          ESP_LOGCONFIG(TAG, "  Radio Status: HEALTHY - Ready to receive");
        }
        else if (!status.pll_locked)
        {
          ESP_LOGW(TAG, "  Radio Status: WARNING - PLL not locked");
        }
        else
        {
          ESP_LOGCONFIG(TAG, "  Radio Status: OK - Mode: %s", status.mode.c_str());
        }
      }
      else
      {
        ESP_LOGE(TAG, "  Status: NOT DETECTED");
        ESP_LOGE(TAG, "  Last Read: 0x%02X (expected 0x24 for HCW or 0x22 for CW)", this->version_);
        ESP_LOGE(TAG, "  Check SPI wiring: MOSI, MISO, SCK, CS pins");
        ESP_LOGE(TAG, "  Check power supply: 3.3V to RFM69");
        ESP_LOGE(TAG, "  Verify chip is RFM69W/CW/HCW");
      }
    }

    /// @brief Frequency setter with PLL lock check,
    /// @param freq e.g. 868000000, this is fixed for 868 MHz set
    void RFM69x::set_frequency(uint32_t freq)
    {
      this->enable();
      this->set_frequency_unsafe_(freq);

      bool pll_locked = this->wait_for_pll_lock_(this->pll_timeout_ms_);

      this->disable();

      if (pll_locked)
      {
        ESP_LOGI(TAG, "Configured frequency: %.2f MHz [FRF=0x%06X]",
                 this->frequency_ / 1e6, freq);
      }
      else
      {
        ESP_LOGE(TAG, "PLL failed to lock");
      }
    }

    void RFM69x::set_frequency_deviation(uint32_t frequency_deviation)
    {
      this->enable();
      this->set_frequency_deviation_unsafe_(frequency_deviation);

      bool pll_locked = this->wait_for_pll_lock_(this->pll_timeout_ms_);

      this->disable();

      if (pll_locked)
      {
        ESP_LOGI(TAG, "Configured frequency deviation: %.2f kHz",
                 frequency_deviation / 1000.0);
      }
      else
      {
        ESP_LOGE(TAG, "PLL failed to lock");
      }
    }

    void RFM69x::set_mode_rx()
    {
      this->enable(); // Acquire Lock

      // Must transition through Standby (or FS) for reliable change
      this->set_opmode_unsafe_(OPMODE_STANDBY);
      delay(10); // Wait for transition

      this->set_opmode_unsafe_(OPMODE_RX);

      // PLL check is essential after setting TX/RX mode
      bool pll_locked = this->wait_for_pll_lock_(this->pll_timeout_ms_);

      this->disable(); // Release Lock

      if (pll_locked)
      {
        ESP_LOGI(TAG, "Set mode to RX");
      }
      else
      {
        ESP_LOGE(TAG, "PLL failed to lock after setting RX mode");
      }
    }

    // PUBLIC: Handles lock and PLL check
    void RFM69x::set_mode_tx()
    {
      this->enable(); // Acquire Lock

      // Must transition through Standby (or FS) for reliable change
      this->set_opmode_unsafe_(OPMODE_STANDBY);
      delay(10); // Wait for transition

      this->set_opmode_unsafe_(OPMODE_TX);

      // PLL check is essential after setting TX/RX mode
      bool pll_locked = this->wait_for_pll_lock_(this->pll_timeout_ms_);

      this->disable(); // Release Lock

      if (pll_locked)
      {
        ESP_LOGI(TAG, "Set mode to TX");
      }
      else
      {
        ESP_LOGE(TAG, "PLL failed to lock after setting TX mode");
      }
    }

    void RFM69x::set_mode_standby()
    {
      this->enable();
      this->set_opmode_unsafe_(OPMODE_STANDBY);
      this->disable();
      ESP_LOGI(TAG, "Set mode to STANDBY");
    }

    void RFM69x::set_mode_sleep()
    {
      this->enable();
      this->set_opmode_unsafe_(OPMODE_SLEEP);
      this->disable();
      ESP_LOGI(TAG, "Set mode to SLEEP");
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
      this->enable();
      this->write_register_raw_(REG_DATAMODUL, reg_value);
      this->disable();
    }

    void RFM69x::set_bitrate(uint32_t bps)
    {
      uint16_t bitrate_reg = 32000000 / bps;
      this->enable();
      write_register_raw_(REG_BITRATEMSB, (bitrate_reg >> 8) & 0xFF);
      write_register_raw_(REG_BITRATELSB, bitrate_reg & 0xFF);
      this->disable();
    }

    // Power level 0-31: 0 = -18dBm, 31 = +13dBm (RFM69CW/HCW)
    // TODO: Add support for PA Boost (PA1+PA2) for +20dBm
    // TODO: add method to read current power level
    void RFM69x::set_power_level(uint8_t level)
    {
      uint8_t val = level & 0x1F; // PA level is 5 bits
      this->enable();
      this->write_register_raw_(REG_PALEVEL, val);
      this->disable();
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
      //           = 32MHz / (20 * 2^4) = 32MHz / 320 = 100 kHz 

      uint8_t reg_value = 0x42; // DccFreq=010, Mant=01 (20), Exp=010 (2)
      this->enable();
      this->write_register_raw_(REG_RXBW, reg_value);
      this->disable();
      ESP_LOGI(TAG, "Set RX bandwidth: ~100 kHz [REG_RXBW=0x%02X]", reg_value);
    }

    void RFM69x::set_sync_word(const std::vector<uint8_t> &sync_word)
    {
      if (sync_word.empty() || sync_word.size() > 8)
      {
        ESP_LOGE(TAG, "Invalid sync word length: %d (must be 1-8)", sync_word.size());
        return;
      }

      // REG_SYNCCONFIG (0x2E)
      // Bit 7: SyncOn (1 = enabled)
      // Bit 6: FifoFillCondition (0 = if sync word seen)
      // Bits 5-4: SyncSize (sync word length - 1)
      // Bits 3-0: SyncTol (bit errors tolerated, 0 = no errors)

      uint8_t sync_config = 0x80;                   // Enable sync
      sync_config |= ((sync_word.size() - 1) << 3); // Set sync size
      // sync_config |= 0x00;  // No bit errors tolerated (optional: add tolerance)
      this->enable();
      this->write_register_raw_(REG_SYNCCONFIG, sync_config);
      delay(1); // Small delay to ensure register is set

      // Write sync bytes to SYNCVALUE1..SYNCVALUE8
      for (size_t i = 0; i < sync_word.size(); i++)
      {
        this->write_register_raw_(REG_SYNCVALUE1 + i, sync_word[i]);
      }
      this->disable();
      ESP_LOGI(TAG, "Configured sync word: %d bytes", sync_word.size());
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
      this->enable();
      this->write_register_raw_(REG_PAYLOADLENGTH, length);
      this->disable();
      ESP_LOGI(TAG, "Set packet length: %d bytes", length);
    }

    void RFM69x::set_variable_length_mode(bool variable)
    {
      this->enable();
      uint8_t config = this->read_register_raw_(REG_PACKETCONFIG1);

      if (variable)
      {
        config |= PACKET1_FORMAT_VARIABLE; // Set bit 7
      }
      else
      {
        config &= ~PACKET1_FORMAT_VARIABLE; // Clear bit 7
      }

      this->write_register_raw_(REG_PACKETCONFIG1, config);
      this->disable();
      ESP_LOGI(TAG, "Variable length mode: %s", variable ? "enabled" : "disabled");
    }

    // actual radio methods
    bool RFM69x::packet_available()
    {
      this->enable();
      uint8_t irq_flags = read_register_raw_(REG_IRQFLAGS2);
      this->disable();
      return (irq_flags & IRQ2_PAYLOAD_READY);
    }

    RFM69x::RadioStatus RFM69x::get_radio_status()
    {
      RadioStatus status;

      status.detected = this->detected_;
      status.version = this->version_;

      if (!this->detected_)
      {
        // Return early with safe defaults if not detected
        status.mode = "Not Detected";
        status.frequency_mhz = 0.0f;
        status.rssi_dbm = -128;
        status.pll_locked = false;
        status.irq1_flags = "N/A";
        status.irq2_flags = "N/A";
        return status;
      }

      this->enable();

      // Read current mode
      uint8_t opmode = this->read_register_raw_(REG_OPMODE);
      status.mode = this->decode_opmode_(opmode);

      // Get frequency
      uint32_t frf = this->get_frequency_actual_unsafe_();
      status.frequency_mhz = (frf * 32000000.0 / 524288.0) / 1e6;

      // Get RSSI
      uint8_t rssi_raw = this->read_register_raw_(REG_RSSIVALUE);
      status.rssi_dbm = -(rssi_raw / 2);

      // Check PLL lock
      uint8_t irq1 = this->read_register_raw_(REG_IRQFLAGS1);
      status.pll_locked = (irq1 & IRQ1_PLLLOCK);

      // Get IRQ flags
      status.irq1_flags = this->decode_irqflags1_(irq1);
      uint8_t irq2 = this->read_register_raw_(REG_IRQFLAGS2);
      status.irq2_flags = this->decode_irqflags2_(irq2);

      this->disable();

      return status;
    }

    std::vector<uint8_t> RFM69x::read_packet()
    {
      std::vector<uint8_t> packet;

      this->enable();
      // For variable length packets the first byte in FIFO is length
      uint8_t length = this->read_register_raw_(REG_FIFO);

      for (uint8_t i = 0; i < length; i++)
      {
        packet.push_back(this->read_register_raw_(REG_FIFO));
      }
      this->disable();

      return packet;
    }

    void RFM69x::set_frequency_unsafe_(uint32_t freq)
    {
      this->frequency_ = freq;

      uint8_t current_mode = this->read_register_raw_(REG_OPMODE) & 0x1C;

      // Datasheet: frequency registers must be written in Standby
      this->set_opmode_unsafe_(OPMODE_STANDBY); // Standby mode

      // Step size = 32 MHz / 2^19 = 61.03515625 Hz
      constexpr double FSTEP = 32000000.0 / 524288.0;

      uint32_t frf = (uint32_t)(this->frequency_ / FSTEP);

      this->write_register_raw_(REG_FRFMSB, (uint8_t)(frf >> 16));
      this->write_register_raw_(REG_FRFMID, (uint8_t)(frf >> 8));
      this->write_register_raw_(REG_FRFLSB, (uint8_t)(frf));

      ESP_LOGI(TAG, "Configured frequency: %.2f MHz [FRF=0x%06X]",
               this->frequency_ / 1e6, frf);
    }

    void RFM69x::set_frequency_deviation_unsafe_(uint32_t frequency_deviation)
    {
      uint16_t fdev_reg = ((uint64_t)frequency_deviation << 19) / 32000000;
      write_register_raw_(REG_FDEVMSB, (fdev_reg >> 8) & 0xFF);
      write_register_raw_(REG_FDEVLSB, fdev_reg & 0xFF);
    }

    void RFM69x::set_opmode_unsafe_(uint8_t mode)
    {
      uint8_t opmode = this->read_register_raw_(REG_OPMODE);
      opmode = (opmode & 0x80) | mode;

      this->write_register_raw_(REG_OPMODE, opmode);
      delay(5);

      // Verify the write
      uint8_t readback = this->read_register_raw_(REG_OPMODE);
      if (readback != opmode)
      {
        ESP_LOGW(TAG, "Mode write failed! Tried to write 0x%02X, read back 0x%02X",
                 opmode, readback);
      }
    }

    // start helper methods for rfm69x
    void RFM69x::configure_rfm69x()
    {
      // set frequency
      if (this->frequency_ != 0)
      {
        set_frequency(this->frequency_);
      }
      // set promiscuous mode
      if (this->promiscuous_mode_)
      {
        this->enable();
        uint8_t val = this->read_register_raw_(REG_PACKETCONFIG1);
        val |= 0x04; // set bit 2
        this->write_register_raw_(REG_PACKETCONFIG1, val);
        this->disable();
      }
      // other configuration can be added here
    }

    void RFM69x::reset_rfm69x()
    {
      if (this->reset_pin_ != 0)
      {
        this->reset_pin_->digital_write(false);
        delay(2);
        this->reset_pin_->digital_write(true);
        delay(5); // wait 5 ms to stabilize
      }
      else
      {
        ESP_LOGW(TAG, "Reset pin not defined, No reset RFM69x at initialization");
      }
    }

    uint32_t RFM69x::get_frequency_actual_unsafe_()
    {

      uint8_t msb = this->read_register_raw_(REG_FRFMSB);
      uint8_t mid = this->read_register_raw_(REG_FRFMID);
      uint8_t lsb = this->read_register_raw_(REG_FRFLSB);

      uint32_t frf = ((uint32_t)msb << 16) | ((uint32_t)mid << 8) | lsb;
      return frf;
    }

    uint8_t RFM69x::read_register_raw_(uint8_t addr)
    {
      this->write_byte(addr & 0x7F); // clear MSB  read
      uint8_t value = this->read_byte();
      return value;
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
        uint8_t flags = this->read_register_raw_(REG_IRQFLAGS1);
        if (flags & IRQ1_PLLLOCK)
          return true;
        if ((millis() - start) >= timeout_ms)
          return false;
        delay(1);
      }
      return false; // Timeout
    }

    // Start decoding methods for rfm69x
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

  } // namespace rfm69x
} // namespace esphome

#include "rfm69x.h"
#include "rfm69x_reg.h"
#include "esphome/core/log.h"

/* This file implements basic RFM69 functionality: first it was experimental,
 * but method by method I'm trying to align the code to the code that is presented
 * in the sx128x component of esphome. Hopefully this will make it easier to integrate
 * RFM69 support into esphome in the future.
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

      // Probe version register
      this->version_ = this->read_register_(REG_VERSION);
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
      // Later: you’ll poll status or handle RadioLib here.
    }

    void RFM69x::dump_config()
    {
      /*ESP_LOGCONFIG(TAG, "RFM69x:");
      LOG_PIN("  CS Pin: ", this->cs_);

      if (this->detected_)
      {
        ESP_LOGCONFIG(TAG, "  Detected RFM69%s, version=0x%02X",
                      this->raw_codes_ ? " [REG_VERSION=0x10]" : "", this->version_);

        uint8_t opmode = this->read_register_(REG_OPMODE);
        ESP_LOGCONFIG(TAG, "  OPMODE: %s%s",
                      decode_opmode_(opmode),
                      this->raw_codes_ ? str_sprintf(" [0x%02X]", opmode).c_str() : "");

        uint32_t frf = get_frequency_actual_();
        double freq_hz = frf * (32000000.0 / 524288.0);

        ESP_LOGCONFIG(TAG, "  Frequency: %.2f MHz%s",
                      freq_hz / 1e6,
                      this->raw_codes_ ? str_sprintf(" [FRF=0x%06X]", frf).c_str() : "");

        uint8_t pa = this->read_register_(REG_PALEVEL);
        ESP_LOGCONFIG(TAG, "  PA Level: %u%s", pa & 0x1F,
                      this->raw_codes_ ? str_sprintf(" [0x%02X]", pa).c_str() : "");

        uint8_t rssi = this->read_register_(REG_RSSIVALUE);
        ESP_LOGCONFIG(TAG, "  RSSI: %u dB%s", rssi,
                      this->raw_codes_ ? str_sprintf(" [0x%02X]", rssi).c_str() : "");

        uint8_t irq1 = this->read_register_(REG_IRQFLAGS1);
        uint8_t irq2 = this->read_register_(REG_IRQFLAGS2);

        ESP_LOGCONFIG(TAG, "  IRQFLAGS1: %s%s",
                      decode_irqflags1_(irq1).c_str(),
                      this->raw_codes_ ? str_sprintf(" [0x%02X]", irq1).c_str() : "");
        ESP_LOGCONFIG(TAG, "  IRQFLAGS2: %s%s",
                      decode_irqflags2_(irq2).c_str(),
                      this->raw_codes_ ? str_sprintf(" [0x%02X]", irq2).c_str() : "");

        ESP_LOGCONFIG(TAG, "  Promiscuous mode: %s%s",
                      this->promiscuous_mode_ ? "Enabled" : "Disabled",
                      this->raw_codes_ ? str_sprintf(" [0x%02X]", this->read_register_(REG_PACKETCONFIG1)).c_str() : "");
      }
      else
      {
        ESP_LOGE(TAG, "  RFM69 not detected (last read=0x%02X), check SPI/MOSI/MISO/CS/wiring.", this->version_);
      }*/

      ESP_LOGCONFIG(TAG, "RFM69x:");
      LOG_PIN("  CS Pin: ", this->cs_);

      // Version check
      uint8_t version = read_register_(REG_VERSION);
      ESP_LOGCONFIG(TAG, "  Detected RFM69 [REG_VERSION=0x%02X], version=0x%02X", REG_VERSION, version);

      // Operating Mode
      uint8_t opmode = read_register_(REG_OPMODE);
      const char *mode_str = "Unknown";
      switch (opmode & OPMODE_MODE_MASK)
      {
      case OPMODE_SLEEP:
        mode_str = "Sleep";
        break;
      case OPMODE_STANDBY:
        mode_str = "Standby";
        break;
      case OPMODE_FS:
        mode_str = "Frequency Synthesizer";
        break;
      case OPMODE_TX:
        mode_str = "Transmit";
        break;
      case OPMODE_RX:
        mode_str = "Receive";
        break;
      }
      ESP_LOGCONFIG(TAG, "  OPMODE: %s Mode [0x%02X]", mode_str, opmode);

      // Frequency
      uint8_t frf_msb = read_register_(REG_FRFMSB);
      uint8_t frf_mid = read_register_(REG_FRFMID);
      uint8_t frf_lsb = read_register_(REG_FRFLSB);
      uint32_t frf = ((uint32_t)frf_msb << 16) | ((uint32_t)frf_mid << 8) | frf_lsb;
      float freq_mhz = (frf * 32000000.0f) / (1 << 19) / 1000000.0f;
      ESP_LOGCONFIG(TAG, "  Frequency: %.2f MHz [FRF=0x%06X]", freq_mhz, frf);

      // Modulation
      uint8_t datamodul = read_register_(REG_DATAMODUL);
      const char *mod_type = (datamodul & DATAMODUL_OOK) ? "OOK" : "FSK";
      const char *data_mode = "Packet";
      if (datamodul & DATAMODUL_CONTINUOUS_NOSYNC)
        data_mode = "Continuous (no sync)";
      else if (datamodul & DATAMODUL_CONTINUOUS_SYNC)
        data_mode = "Continuous (sync)";
      ESP_LOGCONFIG(TAG, "  Modulation: %s, %s mode [0x%02X]", mod_type, data_mode, datamodul);

      // Bitrate
      uint8_t br_msb = read_register_(REG_BITRATEMSB);
      uint8_t br_lsb = read_register_(REG_BITRATELSB);
      uint16_t br_reg = ((uint16_t)br_msb << 8) | br_lsb;
      uint32_t bitrate = 32000000 / br_reg;
      ESP_LOGCONFIG(TAG, "  Bitrate: %u bps [0x%04X]", bitrate, br_reg);

      // Frequency Deviation
      uint8_t fdev_msb = read_register_(REG_FDEVMSB);
      uint8_t fdev_lsb = read_register_(REG_FDEVLSB);
      uint16_t fdev_reg = ((uint16_t)fdev_msb << 8) | fdev_lsb;
      uint32_t fdev_hz = (fdev_reg * 32000000ULL) >> 19;
      ESP_LOGCONFIG(TAG, "  Frequency Deviation: %u Hz [0x%04X]", fdev_hz, fdev_reg);

      // RX Bandwidth
      uint8_t rxbw = read_register_(REG_RXBW);
      ESP_LOGCONFIG(TAG, "  RX Bandwidth: [0x%02X]", rxbw);

      // PA Level
      uint8_t palevel = read_register_(REG_PALEVEL);
      ESP_LOGCONFIG(TAG, "  PA Level: %u [0x%02X]", palevel & PALEVEL_OUTPUT_POWER, palevel);

      // RSSI
      uint8_t rssi = read_register_(REG_RSSIVALUE);
      ESP_LOGCONFIG(TAG, "  RSSI: %d dBm [0x%02X]", -(rssi / 2), rssi);

      // IRQ Flags
      uint8_t irq1 = read_register_(REG_IRQFLAGS1);
      ESP_LOGCONFIG(TAG, "  IRQFLAGS1: %s%s%s%s [0x%02X]",
                    (irq1 & IRQ1_MODEREADY) ? "ModeReady " : "",
                    (irq1 & IRQ1_RXREADY) ? "RxReady " : "",
                    (irq1 & IRQ1_TXREADY) ? "TxReady " : "",
                    (irq1 & IRQ1_PLLLOCK) ? "PLLLock " : "",
                    irq1);

      uint8_t irq2 = read_register_(REG_IRQFLAGS2);
      ESP_LOGCONFIG(TAG, "  IRQFLAGS2: %s%s%s [0x%02X]",
                    (irq2 & IRQ2_FIFO_FULL) ? "FifoFull " : "",
                    (irq2 & IRQ2_FIFO_NOT_EMPTY) ? "FifoNotEmpty " : "",
                    (irq2 & IRQ2_PAYLOAD_READY) ? "PayloadReady " : "",
                    irq2);

      // Packet Config
      uint8_t pkt_cfg1 = read_register_(REG_PACKETCONFIG1);
      bool promiscuous = (pkt_cfg1 & 0x06) == 0; // Address filter disabled
      ESP_LOGCONFIG(TAG, "  Promiscuous mode: %s [0x%02X]", promiscuous ? "Enabled" : "Disabled", pkt_cfg1);
    }

    void RFM69x::set_bitrate(uint32_t bps)
    {
      uint16_t bitrate_reg = 32000000 / bps;
      write_register_(REG_BITRATEMSB, (bitrate_reg >> 8) & 0xFF);
      write_register_(REG_BITRATELSB, bitrate_reg & 0xFF);
    }

    void RFM69x::set_frequency(uint32_t freq)
    {
      this->frequency_ = freq;

      uint8_t current_mode = this->read_register_(REG_OPMODE) & 0x1C;

      // Datasheet: frequency registers must be written in Standby
      this->set_mode_((current_mode & 0xE3) | OPMODE_STANDBY); // Standby mode

      // Step size = 32 MHz / 2^19 = 61.03515625 Hz
      constexpr double FSTEP = 32000000.0 / 524288.0;

      uint32_t frf = (uint32_t)(this->frequency_ / FSTEP);

      this->write_register_(REG_FRFMSB, (uint8_t)(frf >> 16));
      this->write_register_(REG_FRFMID, (uint8_t)(frf >> 8));
      this->write_register_(REG_FRFLSB, (uint8_t)(frf));

      ESP_LOGI(TAG, "Configured frequency: %.2f MHz [FRF=0x%06X]",
               this->frequency_ / 1e6, frf);

      this->set_mode_(current_mode); // restore previous mode
    }

    void RFM69x::set_frequency_deviation(uint32_t frequency_deviation)
    {
      uint16_t fdev_reg = ((uint64_t)frequency_deviation << 19) / 32000000;
      write_register_(REG_FDEVMSB, (fdev_reg >> 8) & 0xFF);
      write_register_(REG_FDEVLSB, fdev_reg & 0xFF);
    }

    // actual radio methods
    bool RFM69x::packet_available()
    {
      uint8_t irq_flags = read_register_(REG_IRQFLAGS2);
      return (irq_flags & IRQ2_PAYLOAD_READY) != 0;
    }
    /*(std::vector<uint8_t> RFM69x::read_packet()
    {
      /*std::vector<uint8_t> packet;

      // Read payload length (first byte in FIFO for variable length packets)
      uint8_t length = read_register(REG_FIFO);

      // Read the actual data
      for (uint8_t i = 0; i < length; i++)
      {
        packet.push_back(read_register(REG_FIFO));
      }

      return packet;}*/

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
        uint8_t val = this->read_register_(REG_PACKETCONFIG1);
        val |= 0x04; // set bit 2
        this->write_register_(REG_PACKETCONFIG1, val);
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

    void RFM69x::set_mode_(uint8_t mode)
    {
      // Keep other OPMODE bits (like listen) intact
      uint8_t opmode = this->read_register_(REG_OPMODE);
      opmode = (opmode & 0xE3) | (mode << 2); // Mode bits are [4:2]
      this->write_register_(REG_OPMODE, opmode);

      // Wait for ModeReady
      while (!(this->read_register_(REG_IRQFLAGS1) & 0x80))
      {
        delay(1);
      }
    }

    uint32_t RFM69x::get_frequency_actual_()
    {
      uint8_t msb = this->read_register_(REG_FRFMSB);
      uint8_t mid = this->read_register_(REG_FRFMID);
      uint8_t lsb = this->read_register_(REG_FRFLSB);

      uint32_t frf = ((uint32_t)msb << 16) | ((uint32_t)mid << 8) | lsb;
      return frf;
    }

    void RFM69x::set_modulation(RFM69Modulation mod,
                                RFM69DataMode mode,
                                RFM69Shaping shaping)
    {
      uint8_t reg_value = 0;

      // Build register value from parameters
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

      reg_value |= (mod == RFM69_FSK) ? DATAMODUL_FSK : DATAMODUL_OOK;
      reg_value |= shaping; // Using the bit values directly

      this->write_register_(REG_DATAMODUL, reg_value);
    }

    uint8_t RFM69x::read_register_(uint8_t addr)
    {
      this->enable();
      this->write_byte(addr & 0x7F); // clear MSB → read
      uint8_t value = this->read_byte();
      this->disable();
      return value;
    }

    void RFM69x::write_register_(uint8_t addr, uint8_t value)
    {
      uint8_t buffer[2] = {static_cast<uint8_t>(addr | 0x80), value};
      this->enable();
      this->write_array(buffer, 2);
      this->disable();
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

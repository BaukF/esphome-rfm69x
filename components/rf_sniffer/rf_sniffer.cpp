#include <esphome/core/component.h>
#include <esphome/core/log.h>
#include "rf_sniffer.h"
#include "../rfm69x/rfm69x.h"

/*
 * TODO:
 * - Actually test with RFM69x hardware, DucoBox messages
 * - Support CC1101 in addition to RFM69x
 * - add option to switch between scanning mode and fixed frequency mode
 * - in fixed frequency mode, display RSSI periodically and when packets are detected
 * - read and display packet data when packets are detected
 * - implement events for home assistant when packets are detected
 * - add option to configure scan range and step size
 * - add option to configure fixed frequency
 * - add option to configure radio parameters (frequency, bitrate, deviation, bandwidth, sync word, etc.)
 * - add option to configure status update interval
 * - extend documentation
 */

namespace esphome
{
  namespace rf_sniffer
  {

    void RfSniffer::setup()
    {
      if (this->radio_ != nullptr)
      {
        /*
        In https://github.com/arnemauer/Ducobox-ESPEasy-Plugin/blob/master/lib/Duco/DucoCC1101.cpp:
        Base frequency		868.326447 MHz 		(868.294312 FREQ2-0 0x21 0x65 0x5C) / (868.292358 FREQ2-0 0x21 0x65 0x57)
        Channel				0
        Channel spacing		199.951172 kHz
        Carrier frequency	868.326447 MHz  		(868.294312 FREQ2-0 0x21 0x65 0x5C) / (868.292358 FREQ2-0 0x21 0x65 0x57)
        Xtal frequency		26.000000 MHz
        Data rate			38.3835 kBaud
        RX filter BW		101.562500 kHz
        Manchester			disabled
        Modulation			GFSK
        Deviation			20.629883 kHz
        TX power			0xC5,0x00,0x00,0x00,0x00,0x00,0x00,0x00
        PA ramping			disabled
        Whitening			disabled
        IF frequency 		(26000 / 2^10 ) * 6 = 152,34375 kHz
        */

        // later: initialize sniffer mode in radio
        // this->radio_->set_promiscuous_mode(true);
        ESP_LOGI("RfSniffer", "Configuring radio for Duco protocol...");

        // Frequency: 868.326447 MHz
        this->radio_->set_frequency(868326447);

        // Modulation: FSK, packet mode, Gaussian shaping
        this->radio_->set_modulation(rfm69x::RFM69_FSK,
                                     rfm69x::RFM69_PACKET_MODE,
                                     rfm69x::RFM69_SHAPING_GAUSSIAN_BT_0_5);

        // Bitrate: 38384 bps
        this->radio_->set_bitrate(38384);

        // Frequency deviation: 20630 Hz
        this->radio_->set_frequency_deviation(20630);

        // RX bandwidth: 101562.5 Hz (~101.5625 kHz)
        this->radio_->set_rx_bandwidth(101562);

        // Power level (receiver mode, so set to minimum)
        this->radio_->set_power_level(0);

        // Packet format: variable length
        this->radio_->set_variable_length_mode(true);

        // Max packet length: 32 bytes
        this->radio_->set_packet_length(32);

        // Enable CRC
        this->radio_->enable_crc(true);

        // No address filtering (promiscuous mode)
        this->radio_->set_promiscuous_mode(true);

        // Sync word: 0xD3 0x91
        std::vector<uint8_t> duco_sync = {0xD3, 0x91};
        this->radio_->set_sync_word(duco_sync);

        // Enter RX mode
        this->radio_->set_mode_rx();
        delay(50);
      }
    }

    void RfSniffer::loop()
    {
      if (this->radio_ == nullptr)
        return;

      uint32_t now = millis();
      static uint32_t last_rssi_check = 0;
      static uint32_t last_packet_check = 0;
      static uint32_t last_irq_dump = 0;
      static bool last_packet_detected = false;

      // ============================================================
      // RSSI Check - Every 500ms (diagnostic - was 2s, too slow)
      // ============================================================
      if (now - last_rssi_check > 500)
      {
        uint8_t rssi = this->radio_->get_rssi();
        uint8_t irq1 = this->radio_->get_irq_flags1();
        uint8_t irq2 = this->radio_->get_irq_flags2();

        ESP_LOGI("RfSniffer", "RSSI: -%d dBm | IRQ1: 0x%02X | IRQ2: 0x%02X",
                 rssi / 2, irq1, irq2);
        last_rssi_check = now;
      }

      // ============================================================
      // Detailed IRQ Flags Dump - Every 500ms (diagnostic)
      // ============================================================
      if (now - last_irq_dump > 500)
      {
        uint8_t irq1 = this->radio_->get_irq_flags1();
        uint8_t irq2 = this->radio_->get_irq_flags2();

        // Decode IRQ2 flags
        std::string irq2_status;
        if (irq2 & 0x80)
          irq2_status += "FifoFull ";
        if (irq2 & 0x40)
          irq2_status += "FifoNotEmpty ";
        if (irq2 & 0x20)
          irq2_status += "FifoLevel ";
        if (irq2 & 0x10)
          irq2_status += "FifoOverrun ";
        if (irq2 & 0x08)
          irq2_status += "PacketSent ";
        if (irq2 & 0x04)
          irq2_status += "PayloadReady ";
        if (irq2 & 0x02)
          irq2_status += "CRC_OK ";
        if (irq2 & 0x01)
          irq2_status += "LowBat ";

        if (irq2_status.empty())
          irq2_status = "None";

        ESP_LOGD("RfSniffer", "IRQ1: 0x%02X | IRQ2: 0x%02X [%s]", irq1, irq2, irq2_status.c_str());

        // Special warnings for issues
        if (irq2 & 0x10)
        {
          ESP_LOGW("RfSniffer", "FIFO OVERRUN - data was lost!");
        }

        last_irq_dump = now;
      }

      // ============================================================
      // Packet Check - Every 100ms (responsive)
      // ============================================================
      if (now - last_packet_check > 100)
      {
        bool packet_available = this->radio_->packet_available();

        // Only log when state CHANGES (packet arrives or timeout)
        if (packet_available && !last_packet_detected)
        {
          ESP_LOGW("RfSniffer", "*** PACKET DETECTED! ***");
          last_packet_detected = true;

          // TODO: Read packet here
          // std::vector<uint8_t> packet = this->radio_->read_packet();
          // ESP_LOGW("RfSniffer", "Packet length: %d", packet.size());
          // for (size_t i = 0; i < packet.size(); i++) {
          //   ESP_LOGW("RfSniffer", "  [%d]: 0x%02X", i, packet[i]);
          // }
        }
        else if (!packet_available && last_packet_detected)
        {
          ESP_LOGI("RfSniffer", "Packet read (FIFO cleared)");
          last_packet_detected = false;
        }

        last_packet_check = now;
      }
    }

    void RfSniffer::dump_config()
    {
      (TAG, "RF Sniffer:");

      if (this->radio_ != nullptr)
      {
        ESP_LOGD(TAG, "  Radio Component: Connected");
        // TODO: ESP_LOGCONFIG(TAG, "  Status Update Interval: %u ms", this->status_update_interval_);
        ESP_LOGD(TAG, "  Scanning Mode: %s", this->scanning_mode_ ? "YES" : "NO");

        // Let the radio dump its own detailed config
        ESP_LOGD(TAG, "");
        ESP_LOGD(TAG, "Radio Configuration:");
        this->radio_->dump_config();
      }
      else
      {
        ESP_LOGW(TAG, "  ⚠️  No radio component set!");
      }
    }

    // in future the parameter would be a component, but now we just assume RFM69x
    void RfSniffer::set_radio(rfm69x::RFM69x *radio)
    {
      // in this method we could check if parent is of type rfm69x::RFM69x or cc1101::CC1101, but for now we just store the pointer assuming RFM69x
      // in the future we could also support CC1101
      // this->radio_ = dynamic_cast<rfm69x::RFM69x *>(radio);
      this->radio_ = (rfm69x::RFM69x *)(radio);
    }

  } // namespace rf_sniffer
} // namespace esphome

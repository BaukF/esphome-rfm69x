#include <esphome/core/component.h>
#include <esphome/core/log.h>
#include "rf_sniffer.h"
#include "../rfm69x/rfm69x.h"

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
        this->radio_->set_promiscuous_mode(true);
        // this->radio_->set_frequency(868326447); // Ducomented by Arne, will not work for mine
        this->radio_->set_frequency(868400000); // My DucoBox is at 868.400 MHz
        this->radio_->set_modulation(rfm69x::RFM69_FSK,
                                     rfm69x::RFM69_PACKET_MODE,
                                     rfm69x::RFM69_SHAPING_GAUSSIAN_BT_0_5);

        this->radio_->set_bitrate(38384);
        this->radio_->set_frequency_deviation(20630);
        this->radio_->set_rx_bandwidth(100000);

        // Configure Duco sync word
        std::vector<uint8_t> duco_sync = {0xD3, 0x91};
        this->radio_->set_sync_word(duco_sync);

        // Configure packet format
        this->radio_->set_variable_length_mode(true);
        this->radio_->set_packet_length(32); // Max length

        this->radio_->set_mode_rx();

        ESP_LOGI("RfSniffer", "Radio configured, checking mode...");
        delay(10); // Give it a moment to switch
      }
    }

    void RfSniffer::scan_frequencies()
    {
      // Scan from 868.0 to 869.0 MHz in 100 kHz steps
      const uint32_t START_FREQ = 868000000;
      const uint32_t END_FREQ = 869000000;
      const uint32_t STEP = 100000; // 100 kHz steps

      static uint32_t last_scan = 0;
      static uint32_t current_freq = START_FREQ;

      // Scan every 100ms
      if (millis() - last_scan < 100)
        return;

      last_scan = millis();

      // Set frequency
      this->radio_->set_frequency(current_freq);
      this->radio_->set_mode_rx();
      delay(50); // Let it settle

      // Read RSSI
      uint8_t rssi = this->radio_->get_rssi();
      int16_t rssi_dbm = -(rssi / 2);

      // Log if we see activity (threshold: > -100 dBm)
      if (rssi_dbm > -100)
      {
        ESP_LOGW(TAG, "*** ACTIVITY at %.3f MHz: RSSI = %d dBm ***",
                 current_freq / 1e6, rssi_dbm);
      }
      else
      {
        ESP_LOGD(TAG, "Scanning %.3f MHz: %d dBm",
                 current_freq / 1e6, rssi_dbm);
      }

      // Next frequency
      current_freq += STEP;
      if (current_freq > END_FREQ)
      {
        current_freq = START_FREQ;
        ESP_LOGI("RfSniffer", "--- Scan cycle complete ---");
      }
    }

    void RfSniffer::loop()
    {
      if (this->radio_ == nullptr)
        return;

      if (false && this->scanning_mode_)
      {
        scan_frequencies();
      }
      else
      {
        //
        static uint32_t last_check = 0;
        if (millis() - last_check > 1000)
        {
          uint8_t rssi = this->radio_->get_rssi();
          uint8_t irq2 = this->radio_->get_irq_flags2();

          ESP_LOGD("RfSniffer", "RSSI: -%d dBm, IRQ2: 0x%02X", rssi / 2, irq2);
          last_check = millis();
        }

        if (this->radio_->packet_available())
        {
          ESP_LOGW("RfSniffer", "*** PACKET DETECTED! ***");
          // TODO: Read and display packet data
        }
      }
    }

    void RfSniffer::dump_config()
    {
      ESP_LOGCONFIG(TAG, "RF Sniffer:");
      if (this->radio_ != nullptr)
      {
        ESP_LOGCONFIG(TAG, "  Using radio component: RFM69x");
        ESP_LOGCONFIG(TAG, "  Radio configured for Duco protocol");
        ESP_LOGCONFIG(TAG, "  Freq: 868.326447 MHz, Rate: 38.384 kBaud, Dev: 20.63 kHz");
        this->radio_->dump_config();
      }
      else
      {
        ESP_LOGW(TAG, "  No radio component set!");
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

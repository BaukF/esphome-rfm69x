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
      // Heartbeat for scanning mode
      if (millis() - last_heartbeat_ > 30000)
      {
        ESP_LOGI(TAG, "Scanning frequencies from %.3f MHz to %.3f MHz in %.1f kHz steps",
                 START_FREQ / 1e6, END_FREQ / 1e6, STEP / 1e3);
        last_heartbeat_ = millis();
      }

      // Scan every 100ms
      if (millis() - last_scan_ < 100)
        return;

      last_scan_ = millis();

      // Set frequency
      this->radio_->set_frequency(current_freq_);
      this->radio_->set_mode_rx();
      delay(50); // Let it settle

      // Read RSSI
      uint8_t rssi = this->radio_->get_rssi();
      int16_t rssi_dbm = -(rssi / 2);

      // Log if we see activity (threshold: > -100 dBm)
      if (rssi_dbm > -100)
      {
        ESP_LOGW(TAG, "*** ACTIVITY at %.3f MHz: RSSI = %d dBm ***",
                 current_freq_ / 1e6, rssi_dbm);
      }
      else
      {
        ESP_LOGD(TAG, "Scanning %.3f MHz: %d dBm",
                 current_freq_ / 1e6, rssi_dbm);
      }

      // Next frequency
      current_freq_ += STEP;
      if (current_freq_ > END_FREQ)
        current_freq_ = START_FREQ;
    }

    void RfSniffer::monitor_frequency()
    {
      // Heartbeat for monitoring mode
      if (millis() - last_heartbeat_ > 30000)
      {
        if (first_packet_detected_)
        {
          int avg = calculate_filtered_average_rssi_();
          ESP_LOGI(TAG, "📡 Monitoring %.3f MHz | Packets: %u | Avg RSSI: %d dBm",
                   current_freq_ / 1e6, packet_count_, avg);
        }
        else
        {
          ESP_LOGI(TAG, "🔍 Monitoring %.3f MHz... (no packets yet)",
                   current_freq_ / 1e6);
        }
        last_heartbeat_ = millis();
      }

      // Check for packets
      if (this->radio_->packet_available())
      {
        // Read RSSI when packet arrives
        int current_rssi = this->radio_->get_rssi();

        // Update history
        update_rssi_history_(current_rssi);

        // Check for significant changes
        if (is_significant_drop_(current_rssi))
        {
          int avg = calculate_filtered_average_rssi_();
          ESP_LOGW(TAG, "⚠️ Signal dropped! Current: %d dBm, Avg: %d dBm (-%d dB)",
                   current_rssi, avg, avg - current_rssi);
        }

        // First packet detection
        if (!first_packet_detected_)
        {
          ESP_LOGI(TAG, "🎯 First packet detected! RSSI: %d dBm", current_rssi);
          first_packet_detected_ = true;
        }

        ESP_LOGW(TAG, "*** PACKET DETECTED! RSSI: %d dBm ***", current_rssi);
        packet_count_++;

        // TODO: Read and display packet data
        // auto packet = this->radio_->read_packet();
      }
    }

    void RfSniffer::loop()
    {
      if (this->radio_ == nullptr)
        return;

      if (this->scanning_mode_)
      {
        scan_frequencies();
      }
      else
      {
        monitor_frequency();
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

    int RfSniffer::calculate_filtered_average_rssi_()
    {
      if (rssi_history_.empty())
        return 0;

      // Step 1: Get median (more robust than mean)
      std::vector<int> sorted(rssi_history_.begin(), rssi_history_.end());
      std::sort(sorted.begin(), sorted.end());
      int median = sorted[sorted.size() / 2];

      // Step 2: Average only values within ±10 dB of median
      int sum = 0;
      int count = 0;
      for (int rssi : rssi_history_)
      {
        if (abs(rssi - median) <= NORMAL_FLUCTUATION)
        {
          sum += rssi;
          count++;
        }
      }

      return count > 0 ? sum / count : median;
    }

    bool RfSniffer::is_significant_drop_(int current_rssi)
    {

      if (rssi_history_.size() < 5)
      {
        return false; // Need baseline first
      }

      int avg = calculate_filtered_average_rssi_();
      int drop = avg - current_rssi; // Positive = signal weaker

      return drop > SIGNIFICANT_DROP;
    }

    void RfSniffer::update_rssi_history_(int rssi)
    {
      rssi_history_.push_back(rssi);

      if (rssi_history_.size() > MAX_HISTORY)
      {
        rssi_history_.pop_front(); // Remove oldest
      }
    }
  } // namespace rf_sniffer
} // namespace esphome

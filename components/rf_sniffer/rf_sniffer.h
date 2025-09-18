#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "../rfm69x/rfm69x.h"

namespace esphome {
namespace rf_sniffer {

class RFSniffer : public Component {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Configuration
  void set_rfm69x(rfm69x::RFM69x *rfm69x) { rfm69x_ = rfm69x; }
  void set_log_raw_packets(bool log_raw) { log_raw_packets_ = log_raw; }
  void set_log_frequency_scan(bool scan) { frequency_scan_ = scan; }
  void set_packet_filter_min_length(uint8_t min_len) { min_packet_length_ = min_len; }
  void set_packet_filter_max_length(uint8_t max_len) { max_packet_length_ = max_len; }

 protected:
  rfm69x::RFM69x *rfm69x_{nullptr};
  bool log_raw_packets_{true};
  bool frequency_scan_{false};
  uint8_t min_packet_length_{0};
  uint8_t max_packet_length_{255};

  // Packet analysis
  void handle_packet(std::vector<uint8_t> data, int16_t rssi);
  void analyze_packet_patterns(const std::vector<uint8_t> &data);
  void log_packet_details(const std::vector<uint8_t> &data, int16_t rssi);
  
  // Duco-specific analysis
  bool looks_like_duco_packet(const std::vector<uint8_t> &data);
  void analyze_potential_duco_patterns(const std::vector<uint8_t> &data);
  
  // Statistics
  uint32_t total_packets_{0};
  uint32_t filtered_packets_{0};
  uint32_t potential_duco_packets_{0};
};

}  // namespace rf_sniffer
}  // namespace esphome
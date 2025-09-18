#include "rf_sniffer.h"
#include "esphome/core/log.h"

namespace esphome {
namespace rf_sniffer {

static const char *const TAG = "rf_sniffer";

void RFSniffer::setup() {
  ESP_LOGCONFIG(TAG, "Setting up RFSniffer...");

  if (this->rfm69x_ != nullptr) {
    this->rfm69x_->set_packet_callback([this](std::vector<uint8_t> data, int16_t rssi) {
      this->handle_packet(data, rssi);
    });
    ESP_LOGCONFIG(TAG, "Packet callback registered with RFM69x");
  } else {
    ESP_LOGE(TAG, "RFM69x component not set!");
  }
}

void Sniffer::dump_config() {
  ESP_LOGCONFIG(TAG, "RFSniffer:");
  ESP_LOGCONFIG(TAG, "  Log raw packets: %s", YESNO(this->log_raw_packets_));
  ESP_LOGCONFIG(TAG, "  Frequency scan: %s", YESNO(this->frequency_scan_));
  ESP_LOGCONFIG(TAG, "  Min packet length: %d", this->min_packet_length_);
  ESP_LOGCONFIG(TAG, "  Max packet length: %d", this->max_packet_length_);
}

void RFSniffer::handle_packet(std::vector<uint8_t> data, int16_t rssi) {
  this->total_packets_++;
  
  // Filter by packet length
  if (data.size() < this->min_packet_length_ || data.size() > this->max_packet_length_) {
    this->filtered_packets_++;
    return;
  }
  
  // Log raw packets if enabled
  if (this->log_raw_packets_) {
    this->log_packet_details(data, rssi);
  }
  
  // Analyze packet patterns
  this->analyze_packet_patterns(data);
  
  // Check for Duco-like characteristics
  if (this->looks_like_duco_packet(data)) {
    this->potential_duco_packets_++;
    ESP_LOGW(TAG, "*** POTENTIAL DUCO PACKET DETECTED! ***");
    this->analyze_potential_duco_patterns(data);
  }
}

void RFSniffer::log_packet_details(const std::vector<uint8_t> &data, int16_t rssi) {
  std::string hex_string;
  hex_string.reserve(data.size() * 3);
  
  for (size_t i = 0; i < data.size(); i++) {
    char hex_byte[4];
    snprintf(hex_byte, sizeof(hex_byte), "%02X ", data[i]);
    hex_string += hex_byte;
  }
  
  ESP_LOGI(TAG, "PKT[%zu]: RSSI=%ddBm Data=%s", data.size(), rssi, hex_string.c_str());
}

void RFSniffer::analyze_packet_patterns(const std::vector<uint8_t> &data) {
  // Basic pattern analysis
  if (data.size() >= 2) {
    // Look for repeating patterns
    bool has_repeating_bytes = false;
    for (size_t i = 1; i < data.size(); i++) {
      if (data[i] == data[i-1]) {
        has_repeating_bytes = true;
        break;
      }
    }
    
    // Check for common header patterns (0xAA, 0x55, 0x00, 0xFF)
    uint8_t first_byte = data[0];
    if (first_byte == 0xAA || first_byte == 0x55 || first_byte == 0x00 || first_byte == 0xFF) {
      ESP_LOGD(TAG, "Common header pattern detected: 0x%02X", first_byte);
    }
    
    // Log statistics every 100 packets
    if (this->total_packets_ % 100 == 0) {
      ESP_LOGI(TAG, "Stats: Total=%d, Filtered=%d, Potential Duco=%d", 
               this->total_packets_, this->filtered_packets_, this->potential_duco_packets_);
    }
  }
}

bool RFSniffer::looks_like_duco_packet(const std::vector<uint8_t> &data) {
  // Placeholder for Duco packet detection heuristics
  // Based on Arne's documentation, add checks for:
  // - Expected packet lengths (typically 8-32 bytes for Duco?)
  // - Known sync patterns
  // - Checksum validation patterns
  // - Device ID patterns
  
  // For now, just basic length filtering
  if (data.size() < 8 || data.size() > 64) {
    return false;
  }
  
  // Add more sophisticated detection logic here as you learn the protocol
  // Examples:
  // - Check for specific sync bytes at known positions
  // - Validate checksum patterns
  // - Look for device address ranges
  
  return false;  // Placeholder - implement based on your findings
}

void RFSniffer::analyze_potential_duco_patterns(const std::vector<uint8_t> &data) {
  ESP_LOGW(TAG, "=== DUCO ANALYSIS ===");
  ESP_LOGW(TAG, "Length: %zu bytes", data.size());
  
  // Analyze potential structure
  if (data.size() >= 4) {
    ESP_LOGW(TAG, "Potential header: %02X %02X", data[0], data[1]);
    ESP_LOGW(TAG, "Potential footer: %02X %02X", data[data.size()-2], data[data.size()-1]);
  }
  
  // Look for ASCII patterns (device names, etc.)
  std::string ascii_check;
  for (auto byte : data) {
    if (byte >= 32 && byte <= 126) {
      ascii_check += static_cast<char>(byte);
    } else {
      ascii_check += '.';
    }
  }
  ESP_LOGW(TAG, "ASCII interpretation: %s", ascii_check.c_str());
  
  ESP_LOGW(TAG, "==================");
}

}  // namespace sniffer
}  // namespace esphome
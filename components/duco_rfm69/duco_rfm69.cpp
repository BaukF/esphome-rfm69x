#include "duco_rfm69.h"
#include "esphome/core/log.h"

namespace esphome {
namespace duco_rfm69 {

static const char *TAG = "duco_rfm69";

void DucoRFM69::setup() {
  ESP_LOGD(TAG, "Running setup for RFM69...");

  // Lets create some breathing room for the RFM69 to power up
  delay(100);

  // Example probe: read version register
  this->spi_setup();
  // Additional delay after SPI init
  delay(50);   // Allow SPI to settle
  
  
  this->enable();
  this->version_ = this->transfer_byte(0x10);  // 0x10 = RFM69 version register
  this->disable();

  if (this->version_ == 0x24) {
    ESP_LOGI(TAG, "RFM69 detected, version=0x%02X", this->version_);
    this->detected_ = true;
  } else {
    ESP_LOGE(TAG, "RFM69 probe failed, got 0x%02X", this->version_);
    this->detected_ = false;
  }
}

void DucoRFM69::loop() {
  // Nothing for now. Will be used later for RadioLib integration or polling.
}

void DucoRFM69::dump_config() {
  ESP_LOGCONFIG(TAG, "DucoRFM69:");
  LOG_PIN("  CS Pin: ", this->cs_);

  if (this->detected_) {
    ESP_LOGCONFIG(TAG, "  RFM69 detected, version=0x%02X", this->version_);
  } else {
    ESP_LOGE(TAG, "  RFM69 not detected (last read=0x%02X)", this->version_);
  }
}

}  // namespace duco_rfm69
}  // namespace esphome
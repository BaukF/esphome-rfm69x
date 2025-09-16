#include "duco_rfm69.h"
#include "esphome/core/log.h"

namespace esphome {
namespace duco_rfm69 {

static const char *TAG = "duco_rfm69.component";

void DucoRFM69::setup() {
  ESP_LOGD(TAG, "Running RFM69 setup...");

  // Initialize SPI device
  this->spi_setup();

  // Read version register (0x10)
  this->enable();                  // Pull CS low
  this->write_byte(0x10 & 0x7F);   // Send address (MSB=0 → read)
  this->version_ = this->read_byte();
  this->disable();                 // Pull CS high

  // Evaluate result
  if (this->version_ == 0x24) {  // 0x24 = expected RFM69 version
    ESP_LOGI(TAG, "RFM69 detected, version=0x%02X", this->version_);
    this->detected_ = true;
  } else {
    ESP_LOGE(TAG, "RFM69 probe failed, got 0x%02X", this->version_);
    this->detected_ = false;
    this->mark_failed();  // tells ESPHome this component isn’t usable
  }
}

void DucoRFM69::loop() {
  // No continuous work yet.
  // Later: you’ll poll status or handle RadioLib here.
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

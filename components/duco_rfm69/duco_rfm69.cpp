#include "duco_rfm69.h"
#include "esphome/core/log.h"

namespace esphome {
namespace duco_rfm69 {

static const char *TAG = "duco_rfm69.component";

// Constructor implementation
DucoRFM69::DucoRFM69() {
  ESP_LOGI("duco_rfm69", "Constructor called!");
}

void DucoRFM69::loop() {
  if (!this->detected_) {
    uint8_t resp = 0;
    this->spi_setup();
    this->enable();
    resp = this->transfer_byte(0x10);  // probe register
    this->disable();

    this->version_ = resp;
    this->detected_ = (resp == 0x24);

    if (this->detected_) {
      ESP_LOGI(TAG, "RFM69 detected, version=0x%02X", this->version_);
    } else {
      ESP_LOGE(TAG, "RFM69 probe failed, got 0x%02X", this->version_);
    }
  }
}

void DucoRFM69::loop() {

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

} // namespace duco_rfm69
} // namespace esphome

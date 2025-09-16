#include "duco_rfm69.h"
#include "esphome/core/log.h"

namespace esphome {
namespace duco_rfm69 {

static const char *TAG = "duco_rfm69.component";

void DucoRFM69::setup() {
  this->needs_init_ = true;  // mark for first loop
}

void DucoRFM69::loop() {
  if (!this->needs_init_)
    return;

  ESP_LOGI(TAG, "Deferred init running...");

  this->spi_setup();
  this->enable();
  this->write_byte(0x10 & 0x7F);
  this->version_ = this->read_byte();
  this->disable();

  if (this->version_ == 0x24) {
    ESP_LOGI(TAG, "RFM69 detected, version=0x%02X", this->version_);
    this->detected_ = true;
  } else {
    ESP_LOGE(TAG, "Unexpected RFM69 version: 0x%02X", this->version_);
    this->mark_failed();
  }

  this->needs_init_ = false;
}

void DucoRFM69::dump_config() {
  ESP_LOGCONFIG(TAG, "Unconfigured DucoRFM69 Component");
}

} // namespace duco_rfm69
} // namespace esphome

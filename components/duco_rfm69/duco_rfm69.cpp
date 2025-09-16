#include "duco_rfm69.h"
#include "esphome/core/log.h"

namespace esphome {
namespace duco_rfm69 {

static const char *TAG = "duco_rfm69";

bool init_rfm69()  override {
  const int max_retries = 10;
  const int retry_delay = 50; // ms
  
  for (int i = 0; i < max_retries; i++) {
    uint8_t version = read_register(RFM69_REG_VERSION);
    if (version == 0x24) {
      ESP_LOGI(TAG, "RFM69 detected on attempt %d", i + 1);
      return true;
    }
    delay(retry_delay);
  }
  
  ESP_LOGE(TAG, "RFM69 not detected after %d attempts", max_retries);
  return false;
}

void DucoRFM69::setup() {

}

void DucoRFM69::loop() {
  if (!initialized_) {
    if (init_rfm69()) {
      initialized_ = true;
      ESP_LOGI(TAG, "RFM69 initialization complete");
    }
    return;
  }
  
  // Normal operation code here}
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

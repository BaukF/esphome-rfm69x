#include "duco_rfm69.h"
#include "esphome/core/log.h"

namespace esphome {
namespace duco_rfm69 {

static const char *TAG = "duco_rfm69.component";

void DucoRFM69::setup() {
   // Do not log here — just mark init needed
  this->needs_init_ = true;
}

void DucoRFM69::loop() {
  ESP_LOGI(TAG, "DucoRFM69 loop running");
  if (this->needs_init_) {
    this->spi_setup();
    ESP_LOGI("duco_rfm69", "spi_setup() called!");
    // Read version register (address 0x10)
    this->enable();                           // Select the chip
    this->write_byte(0x10 & 0x7F);            // Send address with MSB=0 for read

    uint8_t version = this->read_byte();      // Read the version byte
    this->disable();                          // Deselect the chip

    ESP_LOGI(TAG, "RFM69 Version Register: 0x%02X", version);

    if (version == 0x24) { // 0x24 is expected for RFM69
      ESP_LOGI(TAG, "RFM69 module detected!");
    } else {
      ESP_LOGE(TAG, "Unexpected RFM69 version: 0x%02X", version);
      this->mark_failed();
    }

    this->needs_init_ = false; // Initialization done
  }
}

void DucoRFM69::dump_config() {
  ESP_LOGCONFIG(TAG, "Unconfigured DucoRFM69 Component");
}

} // namespace duco_rfm69
} // namespace esphome

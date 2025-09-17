#include "duco_rfm69.h"
#include "esphome/core/log.h"

namespace esphome {
namespace duco_rfm69 {

static const char *TAG = "duco_rfm69.component";


// start default methods for esphome component
void DucoRFM69::setup() {
  ESP_LOGD(TAG, "Running RFM69 setup...");

  // Initialize SPI device
  this->spi_setup();

  // Read version register (0x10)
  this->version_ = this->read_register(0x10);

  // Evaluate results
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

// start custom methods for duco_rfm69
uint8_t read_register(uint8_t addr) {
  this->enable();            // Pull CS low
  this->write_byte(addr & 0x7F);   // Send address (MSB=0 → read)
  uint8_t value = this->read_byte(); // Read value
  
}
  
void DucoRFM69::write_register(uint8_t addr, uint8_t value) {
  this->enable();
  this->write_byte(addr | 0x80);   // set MSB for write
  this->write_byte(value);
  this->disable();
}

}  // namespace duco_rfm69
}  // namespace esphome

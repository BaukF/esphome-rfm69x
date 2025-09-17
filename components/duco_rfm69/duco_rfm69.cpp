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

  // Read version register
  this->version_ = this->read_register(REG_VERSION);

  // Write a test value to a register (optional)
  this->write_register(REG_OPMODE, 0x00);
  this->write_register(REG_FRFMSB, 0xD9);
  this->write_register(REG_FRFMID, 0x00);
  this->write_register(REG_FRFLSB, 0x00);


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

    // Extra status registers
    uint8_t opmode = this->read_register(REG_OPMODE);
    ESP_LOGCONFIG(TAG, "  OPMODE: 0x%02X", opmode);

    uint32_t frf = (uint32_t(this->read_register(REG_FRFMSB)) << 16) |
                   (uint32_t(this->read_register(REG_FRFMID)) << 8) |
                   this->read_register(REG_FRFLSB);
    ESP_LOGCONFIG(TAG, "  Frequency register (FRF): 0x%06lX", frf);

    uint8_t pa = this->read_register(REG_PALEVEL);
    ESP_LOGCONFIG(TAG, "  PA Level: 0x%02X", pa);

    uint8_t rssi = this->read_register(REG_RSSIVALUE);
    ESP_LOGCONFIG(TAG, "  RSSI: %u dB", rssi);

    uint8_t irq1 = this->read_register(REG_IRQFLAGS1);
    uint8_t irq2 = this->read_register(REG_IRQFLAGS2);
    ESP_LOGCONFIG(TAG, "  IRQ Flags: 1=0x%02X 2=0x%02X", irq1, irq2);

  } else {
    ESP_LOGE(TAG, "  RFM69 not detected (last read=0x%02X)", this->version_);
  }
}

// start custom methods for duco_rfm69
uint8_t DucoRFM69::read_register(uint8_t addr) {
  this->enable();
  this->write_byte(addr & 0x7F);   // clear MSB → read
  uint8_t value = this->read_byte();
  this->disable();
  return value;
}

void DucoRFM69::write_register(uint8_t addr, uint8_t value) {
  this->enable();
  this->write_byte(addr | 0x80);   // set MSB → write
  this->write_byte(value);
  this->disable();
}


}  // namespace duco_rfm69
}  // namespace esphome

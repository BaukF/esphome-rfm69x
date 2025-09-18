#include "rfm69x.h"
#include "esphome/core/log.h"

/* This file implements basic RFM69 functionality: first it was experimental,
 * but method by method I'm trying to align the code to the code that is presented
 * in the sx128x component of esphome. Hopefully this will make it easier to integrate
 * RFM69 support into esphome in the future.
 */

namespace esphome {
namespace rfm69x {

static const char *TAG = "rfm69x.component";


// start default methods for esphome component
void RFM69x::setup() {
  ESP_LOGD(TAG, "Initializing RFM69x...");

  // Prepare SPI
  this->spi_setup();

  // Probe version register
  this->version_ = this->read_register(REG_VERSION);

  if (this->version_ == 0x24) {
    this->detected_ = true;
    ESP_LOGI(TAG, "RFM69 detected, version=0x%02X", this->version_);
  } else {
    this->detected_ = false;
    ESP_LOGE(TAG, "RFM69 not detected, read=0x%02X", this->version_);
    this->mark_failed();
  }
}

void RFM69x::loop() {
  // No continuous work yet.
  // Later: you’ll poll status or handle RadioLib here.
}

void RFM69x::dump_config() {
  ESP_LOGCONFIG(TAG, "RFM69x:");
  LOG_PIN("  CS Pin: ", this->cs_);

  if (this->detected_) {
    ESP_LOGCONFIG(TAG, "  Detected RFM69%s, version=0x%02X",
                  this->raw_codes_ ? " [REG_VERSION=0x10]" : "", this->version_);

    uint8_t opmode = this->read_register(REG_OPMODE);
    ESP_LOGCONFIG(TAG, "  OPMODE: %s%s",
                  decode_opmode(opmode),
                  this->raw_codes_ ? str_sprintf(" [0x%02X]", opmode).c_str() : "");

    uint32_t frf = (uint32_t(this->read_register(REG_FRFMSB)) << 16) |
                   (uint32_t(this->read_register(REG_FRFMID)) << 8) |
                   this->read_register(REG_FRFLSB);
    float freq_hz = frf * 61.03515625f;
    ESP_LOGCONFIG(TAG, "  Frequency: %.2f MHz%s",
                  freq_hz / 1e6,
                  this->raw_codes_ ? str_sprintf(" [FRF=0x%06lX]", frf).c_str() : "");

    uint8_t pa = this->read_register(REG_PALEVEL);
    ESP_LOGCONFIG(TAG, "  PA Level: %u%s", pa & 0x1F,
                  this->raw_codes_ ? str_sprintf(" [0x%02X]", pa).c_str() : "");

    uint8_t rssi = this->read_register(REG_RSSIVALUE);
    ESP_LOGCONFIG(TAG, "  RSSI: %u dB%s", rssi,
                  this->raw_codes_ ? str_sprintf(" [0x%02X]", rssi).c_str() : "");

  } else {
    ESP_LOGE(TAG, "  RFM69 not detected (last read=0x%02X)", this->version_);
  }
}


// start custom methods for rfm69x
uint8_t RFM69x::read_register(uint8_t addr) {
  this->enable();
  this->write_byte(addr & 0x7F);   // clear MSB → read
  uint8_t value = this->read_byte();
  this->disable();
  return value;
}

void RFM69x::write_register(uint8_t addr, uint8_t value) {
  this->enable();
  this->write_byte(addr | 0x80);   // set MSB → write
  this->write_byte(value);
  this->disable();
}

const char* RFM69x::decode_opmode(uint8_t opmode) {
    // Mask the opmode to isolate the Mode bits (bits 4, 3, 2).
    uint8_t mode = (opmode & 0x1C) >> 2;

    switch (mode) {
        case 0x00:
            return "Sleep Mode";
        case 0x01:
            return "Standby Mode";
        case 0x02:
            return "Synthesizer Mode";
        case 0x03:
            return "Transmitter Mode";
        case 0x04:
            return "Receiver Mode";
        default:
            return "Unknown Mode";
    }
}

}  // namespace rfm69x
}  // namespace esphome

#include "rfm69x.h"
#include "rfm69x_reg.h"
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

  this->reset_rfm69x();

  this->configure_rfm69x();

  // Probe version register
  this->version_ = this->read_register_(REG_VERSION);
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

    uint8_t opmode = this->read_register_(REG_OPMODE);
    ESP_LOGCONFIG(TAG, "  OPMODE: %s%s",
                  decode_opmode_(opmode),
                  this->raw_codes_ ? str_sprintf(" [0x%02X]", opmode).c_str() : "");

    uint32_t frf = (uint32_t(this->read_register_(REG_FRFMSB)) << 16) |
                   (uint32_t(this->read_register_(REG_FRFMID)) << 8) |
                   this->read_register_(REG_FRFLSB);
    float freq_hz = frf * 61.03515625f;
    ESP_LOGCONFIG(TAG, "  Frequency: %.2f MHz%s",
                  freq_hz / 1e6,
                  this->raw_codes_ ? str_sprintf(" [FRF=0x%06lX]", frf).c_str() : "");

    uint8_t pa = this->read_register_(REG_PALEVEL);
    ESP_LOGCONFIG(TAG, "  PA Level: %u%s", pa & 0x1F,
                  this->raw_codes_ ? str_sprintf(" [0x%02X]", pa).c_str() : "");

    uint8_t rssi = this->read_register_(REG_RSSIVALUE);
    ESP_LOGCONFIG(TAG, "  RSSI: %u dB%s", rssi,
                  this->raw_codes_ ? str_sprintf(" [0x%02X]", rssi).c_str() : "");
    
    // lets also dump IRQ flags
    uint8_t irq1 = this->read_register_(REG_IRQFLAGS1);
    uint8_t irq2 = this->read_register_(REG_IRQFLAGS2);

    std::string irq1_raw_str = raw_codes_ ? str_sprintf(" [0x%02X]", irq1) : "";
    ESP_LOGCONFIG(TAG, "  IRQFLAGS1: %s%s",
                  decode_irqflags1_(irq1).c_str(),
                  irq1_raw_str.c_str());

    std::string irq2_raw_str = raw_codes_ ? str_sprintf(" [0x%02X]", irq2) : "";
    ESP_LOGCONFIG(TAG, "  IRQFLAGS2: %s%s",
                  decode_irqflags2_(irq2).c_str(),
                  irq2_raw_str.c_str());
  } else {
    ESP_LOGE(TAG, "  RFM69 not detected (last read=0x%02X)", this->version_);
  }
}

// start helper methods for rfm69x
uint8_t RFM69x::read_register_(uint8_t addr) {
  this->enable();
  this->write_byte(addr & 0x7F);   // clear MSB → read
  uint8_t value = this->read_byte();
  this->disable();
  return value;
}

void RFM69x::write_register_(uint8_t addr, uint8_t value) {
  this->enable();
  this->write_byte(addr | 0x80);   // set MSB → write
  this->write_byte(value);
  this->disable();
}
/*
void RFM69x::set_frequency(uint32_t freq) {
  // frequency in Hz, e.g. 868000000
  this->frequency_ = freq;
  uint32_t frf = (freq << 2) / 61; // see datasheet
  this->write_register_(REG_FRFMSB, (frf >> 16) & 0xFF);
  this->write_register_(REG_FRFMID, (frf >> 8) & 0xFF);
  this->write_register_(REG_FRFLSB, frf & 0xFF);
}*/

void RFM69x::set_promiscuous_mode(bool promiscuous) {
  // to be implemented
}

void RFM69x::configure_rfm69x() {
  // set frequency
  //sthis->set_frequency(this->frequency_);
  // set promiscuous mode
  this->set_promiscuous_mode(this->promiscuous_mode_);
  // other configuration can be added here
}

void RFM69x::reset_rfm69x() {
  if (this->reset_pin_ != 0) {
    this->reset_pin_->digital_write(false);
    delay(2);
    this->reset_pin_->digital_write(true);
    delay(5); // wait 5 ms to stabilize
  } else {
    ESP_LOGW(TAG, "Reset pin not defined, No reset RFM69x at initialization");
  }
}

// Start decoding methods for rfm69x
const char* RFM69x::decode_opmode_(uint8_t opmode) {
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

static const struct { uint8_t mask; const char *name; } irq1_flags[] = {
  {IRQ1_MODEREADY, "ModeReady"},
  {IRQ1_RXREADY, "RxReady"},
  {IRQ1_TXREADY, "TxReady"},
  {IRQ1_PLLLOCK, "PLLLock"},
  {IRQ1_RSSI, "RSSI"},
  {IRQ1_TIMEOUT, "Timeout"},
  {IRQ1_AUTOMODE, "AutoMode"},
  {IRQ1_SYNCADDRESSMATCH, "SyncMatch"},
};

std::string RFM69x::decode_irqflags1_(uint8_t val) {
  std::string res;
  for (auto &f : irq1_flags) {
    if (val & f.mask) {
      if (!res.empty()) res += " | ";
      res += f.name;  // safe: const char* appended into std::string
    }
  }
  return res.empty() ? "None" : res;
}

static const struct { uint8_t mask; const char *name; } irq2_flags[] = {
  {IRQ2_FIFO_FULL, "FifoFull"},
  {IRQ2_FIFO_NOT_EMPTY, "FifoNotEmpty"},
  {IRQ2_FIFO_LEVEL, "FifoLevel"},
  {IRQ2_FIFO_OVERRUN, "FifoOverrun"},
  {IRQ2_PACKET_SENT, "PacketSent"},
  {IRQ2_PAYLOAD_READY, "PayloadReady"},
  {IRQ2_CRC_OK, "CRC_OK"},
  {IRQ2_LOW_BAT, "LowBat"},
};

std::string RFM69x::decode_irqflags2_(uint8_t val) {
  std::string res;
  for (auto &f : irq2_flags) {
    if (val & f.mask) {
      if (!res.empty()) res += " | ";
      res += f.name;  // safe: const char* appended into std::string
    }
  }
  return res.empty() ? "None" : res;
}


}  // namespace rfm69x
}  // namespace esphome

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

  //this->reset_rfm69x();

  // Probe version register
  this->version_ = this->read_register_(REG_VERSION);
  if (this->version_ == 0x24 || this->version_ == 0x22) { // 0x24 = RFM69HCW, 0x22 = RFM69CW
    this->detected_ = true;
    ESP_LOGI(TAG, "Detected RFM69, version=0x%02X", this->version_);
    this->configure_rfm69x();
  } else {
    this->detected_ = false;
    ESP_LOGE(TAG, "RFM69 not detected (last read=0x%02X)", this->version_);
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

    // dump frequency registers
    uint8_t msb = this->read_register(REG_FRFMSB);
    uint8_t mid = this->read_register(REG_FRFMID);
    uint8_t lsb = this->read_register(REG_FRFLSB);

    uint32_t frf = ((uint32_t)msb << 16) | ((uint32_t)mid << 8) | lsb;
    double freq_hz = frf * (32000000.0 / 524288.0);

    ESP_LOGCONFIG(TAG, "   Frequency registers: 0x%06X => %.2f MHz",
                  frf, freq_hz / 1e6);

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
  uint8_t buffer[2] = { static_cast<uint8_t>(addr | 0x80), value };
  this->enable();
  this->write_array(buffer, 2);
  this->disable();
}

void RFM69x::set_frequency(uint32_t freq) {
  this->frequency_ = freq;

  // FRF = Fcarrier / Fstep, Fstep = 32 MHz / 2^19
  constexpr double FSTEP = 32000000.0 / 524288.0;
  uint32_t frf = static_cast<uint32_t>(freq / FSTEP);

  this->write_register_(REG_FRFMSB, (frf >> 16) & 0xFF);
  this->write_register_(REG_FRFMID, (frf >> 8) & 0xFF);
  this->write_register_(REG_FRFLSB, frf & 0xFF);

  ESP_LOGI(TAG, "Configured frequency: %.2f MHz [FRF=0x%06X]",
           freq / 1e6, frf);
}

void RFM69x::configure_rfm69x() {
  // set frequency
  if (this->frequency_ != 0) {
    // Step size = 32 MHz / 2^19 = 61.03515625 Hz
    constexpr double FSTEP = 32000000.0 / 524288.0;  

    uint32_t frf = (uint32_t)(this->frequency_ / FSTEP);

    this->write_register_(REG_FRFMSB, (uint8_t)(frf >> 16));
    this->write_register_(REG_FRFMID, (uint8_t)(frf >> 8));
    this->write_register_(REG_FRFLSB, (uint8_t)(frf));

    ESP_LOGI(TAG, "Configured frequency: %.2f MHz [FRF=0x%06X]",
             this->frequency_ / 1e6, frf);
  }
  // set promiscuous mode
  if (this->promiscuous_mode_) {
    uint8_t val = this->read_register_(REG_PACKETCONFIG1);
    val |= 0x04; // set bit 2
    this->write_register_(REG_PACKETCONFIG1, val);
  }
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

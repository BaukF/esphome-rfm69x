#include <esphome/core/component.h>
#include <esphome/core/log.h>
#include "rf_sniffer.h"
#include <esphome/components/rfm69x/rfm69x.h>

namespace esphome {
namespace rf_sniffer {

void RfSniffer::setup()  {
  if (this->radio_ != nullptr) {
    //later do some setup thingies
  } 

}

void RfSniffer::loop() {
  if (this->radio_ != nullptr) {
    // later: poll packets
  }
}

void RfSniffer::dump_config() {
  ESP_LOGCONFIG(TAG, "RF Sniffer:");
  if (this->radio_ != nullptr) {
    ESP_LOGCONFIG(TAG, "  Using radio component:");
    this->radio_->dump_config();
  } else {
    ESP_LOGW(TAG, "  No radio component set!");
  }
}

// in future the parameter would be a component, but now we just assume RFM69x
void RfSniffer::set_radio(rfm69x::RFM69x *radio) {
  // in this method we could check if parent is of type rfm69x::RFM69x or cc1101::CC1101, but for now we just store the pointer assuming RFM69x
  // in the future we could also support CC1101
  //this->radio_ = dynamic_cast<rfm69x::RFM69x *>(radio);
  this->radio_ = (rfm69x::RFM69x *)(radio);
}

}  // namespace rf_sniffer
}  // namespace esphome

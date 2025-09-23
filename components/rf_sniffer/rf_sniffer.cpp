#pragma once
#include "esphome/core/component.h"
#include "esphome/components/rfm69x/rfm69x.h"

namespace esphome {
namespace rf_sniffer {

class RfSniffer : public Component {

  void setup() override {
    if (this->radio_ != nullptr) {
      //later do some setup thingies
    } 

  }

  void loop() override {
    if (this->radio_ != nullptr) {
      // later: poll packets
    }
  }

  void set_radio_(Component *parent) {
        this->radio_ = parent;
        
        // Optional: Try to cast to known radio types and store interface
        // This allows compile-time checking for supported methods
        if (auto *rfm69x = dynamic_cast<rfm69x::RFM69x*>(parent)) {
            this->rfm69x_radio_ = rfm69x;
        } else if (auto *cc1101 = dynamic_cast<cc1101::CC1101*>(parent)) {
            this->cc1101_radio_ = cc1101;
        }
        // Add more radio types as needed
    }
};

}  // namespace rf_sniffer
}  // namespace esphome

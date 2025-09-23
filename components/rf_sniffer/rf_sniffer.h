#pragma once

#include <esphome/core/component.h>
#include <esphome/core/log.h>
#include "../rfm69x/rfm69x.h"

namespace esphome {
namespace rf_sniffer {

static const char *const TAG = "rf_sniffer";

class RfSniffer : public Component {
public:
    void setup() override;
    void loop() override;
    void dump_config() override;
    
private:
    Component *radio_{nullptr};
    rfm69x::RFM69x *rfm69x_radio_{nullptr};
    cc1101::CC1101 *cc1101_radio_{nullptr};

protected:
    void set_radio_component_(Component *parent) {
    void set_frequency(float frequency);
};



}  // namespace rf_sniffer
}  // namespace esphome

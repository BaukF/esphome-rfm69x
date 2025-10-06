#include <esphome/core/component.h>
#include <esphome/core/log.h>
#include "rf_sniffer.h"
#include "../rfm69x/rfm69x.h"

namespace esphome
{
  namespace rf_sniffer
  {

    void RfSniffer::setup()
    {
      if (this->radio_ != nullptr)
      {
        // later: initialize sniffer mode in radio
        this->radio_->set_promiscuous_mode(true);
        this->radio_->set_frequency(868950000);
        this->radio_->set_modulation(RFM69_FSK,
                                     RFM69_PACKET_MODE,
                                     RFM69_SHAPING_NONE);

        this->radio_->set_bitrate(38400);
        this->radio_->set_frequency_deviation(50000);

        this->radio_->set_mode_rx();
      }
    }

    void RfSniffer::loop()
    {
      if (this->radio_ != nullptr)
      {
        if (this->radio_ == nullptr)
          return;

        // Check if packet is ready
        if (this->radio_->packet_available())
        {
          ESP_LOGD("RfSniffer", "Packet detected!");
        }
      }
    }

    void RfSniffer::dump_config()
    {
      ESP_LOGCONFIG(TAG, "RF Sniffer:");
      if (this->radio_ != nullptr)
      {
        ESP_LOGCONFIG(TAG, "  Using radio component: RFM69x");

        ESP_LOGCONFIG(TAG, "  let's see if promiscuous mode is set:");
        this->radio_->dump_config();
      }
      else
      {
        ESP_LOGW(TAG, "  No radio component set!");
      }
    }

    // in future the parameter would be a component, but now we just assume RFM69x
    void RfSniffer::set_radio(rfm69x::RFM69x *radio)
    {
      // in this method we could check if parent is of type rfm69x::RFM69x or cc1101::CC1101, but for now we just store the pointer assuming RFM69x
      // in the future we could also support CC1101
      // this->radio_ = dynamic_cast<rfm69x::RFM69x *>(radio);
      this->radio_ = (rfm69x::RFM69x *)(radio);
    }

  } // namespace rf_sniffer
} // namespace esphome

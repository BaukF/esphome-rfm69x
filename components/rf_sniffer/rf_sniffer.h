#pragma once

#include <esphome/core/component.h>
#include <esphome/core/log.h>
#include "../rfm69x/rfm69x.h"

using esphome::rfm69x::RFM69_CONTINUOUS_NOSYNC;
using esphome::rfm69x::RFM69_CONTINUOUS_SYNC;
using esphome::rfm69x::RFM69_FSK;
using esphome::rfm69x::RFM69_OOK;
using esphome::rfm69x::RFM69_PACKET_MODE;
using esphome::rfm69x::RFM69_SHAPING_NONE;

namespace esphome
{
    namespace rf_sniffer
    {

        static const char *const TAG = "rf_sniffer";

        class RfSniffer : public Component
        {
        public:
            void setup() override;
            void loop() override;
            void dump_config() override;
            void scan_frequencies();
            void set_radio(rfm69x::RFM69x *radio);

        private:
            rfm69x::RFM69x *radio_{nullptr};
            bool scanning_mode_{true}; // Make this configurable

            // Scanning mode variables
            uint32_t last_scan_{0};
            uint32_t current_freq_{868000000};

            // Monitoring mode variables
            bool first_packet_detected_{false};
            void monitor_frequency();
            void update_rssi_history_(int rssi);
            bool is_significant_drop_(int current_rssi);
            int calculate_filtered_average_rssi_();
            uint32_t packet_count_{0};
            std::deque<int> rssi_history_;

            // Shared
            uint32_t last_heartbeat_{0};

            static constexpr size_t MAX_HISTORY = 25;
            static constexpr int NORMAL_FLUCTUATION = 10;
            static constexpr int SIGNIFICANT_DROP = 15;
            static constexpr uint32_t START_FREQ = 868000000;
            static constexpr uint32_t END_FREQ = 869000000;
            static constexpr uint32_t STEP = 100000;
        };

    } // namespace rf_sniffer
} // namespace esphome

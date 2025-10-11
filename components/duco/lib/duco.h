// components/duco/include/duco/duco.h
// -----------------------------------------------------------------------------
// Public header for the Duco protocol wrapper
// Derived from arnemauer/Ducobox-ESPEasy-Plugin (see components/duco/NOTICE)
// -----------------------------------------------------------------------------
#pragma once

#include <vector>
#include <optional>
#include <cstdint>

namespace duco
{

    struct Frame
    {
        std::vector<uint8_t> raw;
        uint32_t timestamp_ms;
        // add parsed fields as needed
    };

    class DucoParser
    {
    public:
        DucoParser();
        ~DucoParser();

        // Feed raw bytes (e.g., from rf_sniffer or serial input).
        // Returns true if feeding produced one or more complete frames.
        bool feed_bytes(const uint8_t *data, size_t len);

        // Pop next available parsed frame (std::nullopt if none)
        std::optional<Frame> pop_frame();

        // Validate raw frame (calls into vendor validate or new core)
        static bool validate_frame(const Frame &frame);

        // Non-copyable but movable
        DucoParser(const DucoParser &) = delete;
        DucoParser &operator=(const DucoParser &) = delete;
        DucoParser(DucoParser &&) noexcept;
        DucoParser &operator=(DucoParser &&) noexcept;

    private:
        struct Impl;
        Impl *impl_;
    };

} // namespace duco

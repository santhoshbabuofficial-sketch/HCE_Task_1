/// @file heartbeat.hpp
/// @brief Periodic heartbeat: toggles the PC4 LED and transmits CAN ID
///        0x200 every 1000 ms.
#pragma once

#include <cstdint>

#include "can_bus_driver.hpp"
#include "gpio_overlay.hpp"
#include "message_codec.hpp"

namespace hce::sensor_node {

/// @brief Sensor Node heartbeat publisher.
class Heartbeat {
public:
    Heartbeat(GpioOverlay& overlay, hce::sensor::ICanBusDriver& can_bus);

    /// @brief Toggle the LED and transmit one heartbeat frame.
    ///        Intended to be called every kPeriodMs from a dedicated thread.
    void Tick();

    static constexpr uint32_t kPeriodMs = 1000U;

private:
    GpioOverlay& overlay_;
    hce::sensor::ICanBusDriver& can_bus_;
    uint32_t sequence_ = 0U;
    bool led_state_ = false;
};

}  // namespace hce::sensor_node

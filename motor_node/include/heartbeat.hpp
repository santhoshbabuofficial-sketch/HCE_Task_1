/// @file heartbeat.hpp
/// @brief Periodic heartbeat: toggles the PC4 LED and transmits CAN ID
///        0x201 every 1000 ms.
#pragma once

#include <cstdint>

#include "can_bus_driver.hpp"
#include "gpio_overlay.hpp"
#include "message_codec.hpp"

namespace hce::motor_node {

/// @brief Motor Node heartbeat publisher.
class Heartbeat {
public:
    Heartbeat(GpioOverlay& overlay, hce::motor::ICanBusDriver& can_bus);

    void Tick();

    static constexpr uint32_t kPeriodMs = 1000U;

private:
    GpioOverlay& overlay_;
    hce::motor::ICanBusDriver& can_bus_;
    uint32_t sequence_ = 0U;
    bool led_state_ = false;
};

}  // namespace hce::motor_node

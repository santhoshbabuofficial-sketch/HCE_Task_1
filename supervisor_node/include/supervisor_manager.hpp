/// @file supervisor_manager.hpp
/// @brief Mediator between CAN RX (pressure/flow/heartbeats), alarm_logic,
///        heartbeat_monitor and CAN TX (RUN/STOP). Implements the Mediator
///        design pattern (Rule 4): CAN input handling, fault evaluation and
///        command dispatch never reference each other directly, only
///        through SupervisorManager.
#pragma once

#include <cstdint>

#include "alarm_logic.hpp"
#include "can_bus_driver.hpp"
#include "gpio_overlay.hpp"
#include "heartbeat_monitor.hpp"
#include "message_codec.hpp"

namespace hce::supervisor_node {

/// @brief Supervisor RUN/STOP output state.
enum class SupervisorState : uint8_t {
    kRun = 0U,
    kStop = 1U,
};

/// @brief Top-level Supervisor Node coordinator.
class SupervisorManager {
public:
    SupervisorManager(GpioOverlay& overlay, hce::supervisor::ICanBusDriver& can_bus);

    // ---- CAN RX handlers (called from ISR-safe deferred context) --------
    void OnPressureReceived(const hce::supervisor::MessageCodec::Payload& payload);
    void OnFlowReceived(const hce::supervisor::MessageCodec::Payload& payload);
    void OnSensorHeartbeatReceived();
    void OnMotorHeartbeatReceived();

    /// @brief Evaluate fault logic and drive alarm LED + RUN/STOP CAN
    ///        output. Intended to be called every 1000 ms (Supervisor
    ///        Monitoring period) from a dedicated thread.
    void Tick();

    SupervisorState State() const { return state_; }

private:
    void TransmitStopCommand();
    void TransmitRunCommand();

    GpioOverlay& overlay_;
    hce::supervisor::ICanBusDriver& can_bus_;
    HeartbeatMonitor heartbeat_monitor_;
    AlarmLogic alarm_logic_;

    float latest_pressure_pa_ = 0.0F;
    bool pressure_valid_ = false;
    float latest_flow_lpm_ = 0.0F;
    bool flow_valid_ = false;

    SupervisorState state_ = SupervisorState::kStop;

    static constexpr uint32_t kDefaultRunStepRateHz = 400U;
};

}  // namespace hce::supervisor_node

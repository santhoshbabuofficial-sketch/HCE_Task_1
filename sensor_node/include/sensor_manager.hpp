/// @file sensor_manager.hpp
/// @brief Orchestrates pressure/flow sampling and fans each reading out to
///        the LCD and the CAN bus using the Observer design pattern
///        (Rule 4: named design pattern per module).
#pragma once

#include <array>
#include <cstdint>

#include "can_bus_driver.hpp"
#include "flow_sensor.hpp"
#include "lcd_display.hpp"
#include "message_codec.hpp"
#include "pressure_sensor.hpp"

namespace hce::sensor_node {

/// @brief Observer interface notified whenever a new sample is available.
///        LcdDisplay and the CAN publisher path both implement this.
class ISensorDataObserver {
public:
    virtual ~ISensorDataObserver() = default;
    virtual void OnPressureSample(float pressure_pa) = 0;
    virtual void OnFlowSample(float flow_lpm) = 0;
};

/// @brief Adapts LcdDisplay to the ISensorDataObserver interface.
class LcdSensorObserver final : public ISensorDataObserver {
public:
    explicit LcdSensorObserver(ILcdDisplay& lcd) : lcd_(lcd) {}
    void OnPressureSample(float pressure_pa) override { lcd_.ShowPressure(pressure_pa); }
    void OnFlowSample(float flow_lpm) override { lcd_.ShowFlow(flow_lpm); }

private:
    ILcdDisplay& lcd_;
};

/// @brief Adapts the CAN transport to the ISensorDataObserver interface,
///        publishing CAN ID 0x300 (pressure) / 0x301 (flow).
class CanSensorObserver final : public ISensorDataObserver {
public:
    explicit CanSensorObserver(hce::sensor::ICanBusDriver& can_bus) : can_bus_(can_bus) {}
    void OnPressureSample(float pressure_pa) override;
    void OnFlowSample(float flow_lpm) override;

private:
    hce::sensor::ICanBusDriver& can_bus_;
    uint32_t pressure_sequence_ = 0U;
    uint32_t flow_sequence_ = 0U;
};

/// @brief Top-level Sensor Node sampling coordinator.
class SensorManager {
public:
    static constexpr std::size_t kMaxObservers = 4U;

    SensorManager(IPressureSensor& pressure_sensor, IFlowSensor& flow_sensor);

    /// @brief Register an observer (Observer pattern subject-side API).
    /// @return true if there was a free slot.
    bool AddObserver(ISensorDataObserver& observer);

    /// @brief Sample both sensors once and notify all observers.
    ///        Intended to be called every 1000 ms from a dedicated thread.
    void Tick();

    /// @brief Called by the CAN RX path when a STOP command (0x100) arrives.
    void OnStopCommandReceived();

private:
    IPressureSensor& pressure_sensor_;
    IFlowSensor& flow_sensor_;
    std::array<ISensorDataObserver*, kMaxObservers> observers_{};
    std::size_t observer_count_ = 0U;
};

}  // namespace hce::sensor_node

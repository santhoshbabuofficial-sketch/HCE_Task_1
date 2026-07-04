#include "sensor_manager.hpp"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sensor_manager, LOG_LEVEL_INF);

namespace hce::sensor_node {

void CanSensorObserver::OnPressureSample(float pressure_pa) {
    hce::sensor::PressureMessage msg{};
    msg.pressure_pa = pressure_pa;
    msg.sequence = pressure_sequence_;
    ++pressure_sequence_;

    const hce::sensor::MessageCodec::Payload payload = hce::sensor::MessageCodec::EncodePressure(msg);
    can_bus_.Send(hce::sensor::CanId::kPressure, payload.data(),
                   hce::sensor::MessageCodec::kPayloadBytes);

    LOG_INF("TX Pressure: pressure_pa=%.2f sequence=%u", static_cast<double>(pressure_pa),
            msg.sequence);
}

void CanSensorObserver::OnFlowSample(float flow_lpm) {
    hce::sensor::FlowMessage msg{};
    msg.flow_lpm = flow_lpm;
    msg.sequence = flow_sequence_;
    ++flow_sequence_;

    const hce::sensor::MessageCodec::Payload payload = hce::sensor::MessageCodec::EncodeFlow(msg);
    can_bus_.Send(hce::sensor::CanId::kFlow, payload.data(),
                   hce::sensor::MessageCodec::kPayloadBytes);

    LOG_INF("TX Flow: flow_lpm=%.2f sequence=%u", static_cast<double>(flow_lpm), msg.sequence);
}

SensorManager::SensorManager(IPressureSensor& pressure_sensor, IFlowSensor& flow_sensor)
    : pressure_sensor_(pressure_sensor), flow_sensor_(flow_sensor) {}

bool SensorManager::AddObserver(ISensorDataObserver& observer) {
    if (observer_count_ >= kMaxObservers) {
        return false;
    }
    observers_[observer_count_] = &observer;
    ++observer_count_;
    return true;
}

void SensorManager::Tick() {
    float pressure_pa = 0.0F;
    if (pressure_sensor_.Read(pressure_pa)) {
        for (std::size_t i = 0U; i < observer_count_; ++i) {
            observers_[i]->OnPressureSample(pressure_pa);
        }
    }

    float flow_lpm = 0.0F;
    if (flow_sensor_.Read(flow_lpm)) {
        for (std::size_t i = 0U; i < observer_count_; ++i) {
            observers_[i]->OnFlowSample(flow_lpm);
        }
    }
}

void SensorManager::OnStopCommandReceived() {
    // Forwarded to LCD observers that also implement emergency-stop
    // display; sensor_manager itself stays hardware-agnostic and simply
    // signals the event upward. Handled directly in main.cpp's CAN
    // callback for the LCD, since ISensorDataObserver does not carry an
    // E-STOP notification (kept minimal per Observer interface).
}

}  // namespace hce::sensor_node

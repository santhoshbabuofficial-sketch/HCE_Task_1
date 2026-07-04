#include "heartbeat.hpp"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(motor_heartbeat, LOG_LEVEL_INF);

namespace hce::motor_node {

Heartbeat::Heartbeat(GpioOverlay& overlay, hce::motor::ICanBusDriver& can_bus)
    : overlay_(overlay), can_bus_(can_bus) {}

void Heartbeat::Tick() {
    led_state_ = !led_state_;
    overlay_.SetHeartbeatLed(led_state_);

    hce::motor::HeartbeatMessage msg{};
    msg.sequence = sequence_;
    msg.node_ok = 1U;
    ++sequence_;

    const hce::motor::MessageCodec::Payload payload = hce::motor::MessageCodec::EncodeHeartbeat(msg);
    can_bus_.Send(hce::motor::CanId::kMotorHeartbeat, payload.data(),
                   hce::motor::MessageCodec::kPayloadBytes);

    LOG_INF("TX Heartbeat: sequence=%u node_ok=%u", msg.sequence, msg.node_ok);
}

}  // namespace hce::motor_node

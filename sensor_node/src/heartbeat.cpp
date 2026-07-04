#include "heartbeat.hpp"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(sensor_heartbeat, LOG_LEVEL_INF);

namespace hce::sensor_node {

Heartbeat::Heartbeat(GpioOverlay& overlay, hce::sensor::ICanBusDriver& can_bus)
    : overlay_(overlay), can_bus_(can_bus) {}

void Heartbeat::Tick() {
    led_state_ = !led_state_;
    overlay_.SetHeartbeatLed(led_state_);

    hce::sensor::HeartbeatMessage msg{};
    msg.sequence = sequence_;
    msg.node_ok = 1U;
    ++sequence_;

    const hce::sensor::MessageCodec::Payload payload = hce::sensor::MessageCodec::EncodeHeartbeat(msg);
    can_bus_.Send(hce::sensor::CanId::kSensorHeartbeat, payload.data(),
                   hce::sensor::MessageCodec::kPayloadBytes);

    LOG_INF("TX Heartbeat: sequence=%u node_ok=%u", msg.sequence, msg.node_ok);
}

}  // namespace hce::sensor_node

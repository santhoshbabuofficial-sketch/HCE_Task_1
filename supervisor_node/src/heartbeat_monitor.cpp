#include "heartbeat_monitor.hpp"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(heartbeat_monitor, LOG_LEVEL_INF);

namespace hce::supervisor_node {

void NodeLivenessTracker::OnHeartbeatReceived() {
    received_since_last_tick_ = true;
}

void NodeLivenessTracker::OnMonitoringTick() {
    const bool was_faulted = IsFaulted();
    if (received_since_last_tick_) {
        consecutive_misses_ = 0U;
    } else if (consecutive_misses_ < kMaxConsecutiveMisses) {
        ++consecutive_misses_;
    }
    received_since_last_tick_ = false;

    if (IsFaulted() != was_faulted) {
        LOG_INF("LIVENESS -> %s (consecutive_misses=%u)", IsFaulted() ? "FAULTED" : "OK",
                consecutive_misses_);
    }
}

void HeartbeatMonitor::OnMonitoringTick() {
    sensor_tracker_.OnMonitoringTick();
    motor_tracker_.OnMonitoringTick();
}

}  // namespace hce::supervisor_node

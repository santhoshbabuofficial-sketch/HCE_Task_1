/// @file heartbeat_monitor.hpp
/// @brief Tracks liveness of the Sensor Node and Motor Node heartbeats.
///        A node is declared faulted after three consecutive missed
///        heartbeats (3000 ms at the 1000 ms heartbeat period).
#pragma once

#include <cstdint>

namespace hce::supervisor_node {

/// @brief Per-remote-node liveness tracker.
class NodeLivenessTracker {
public:
    static constexpr uint8_t kMaxConsecutiveMisses = 3U;

    /// @brief Call when a heartbeat frame is received from this node.
    void OnHeartbeatReceived();

    /// @brief Call once per supervisor monitoring tick (1000 ms) to age
    ///        out the current window and evaluate for missed heartbeats.
    void OnMonitoringTick();

    /// @brief true if the node has missed kMaxConsecutiveMisses in a row.
    bool IsFaulted() const { return consecutive_misses_ >= kMaxConsecutiveMisses; }

private:
    uint8_t consecutive_misses_ = 0U;
    bool received_since_last_tick_ = false;
};

/// @brief Aggregates liveness for both remote nodes the Supervisor watches.
class HeartbeatMonitor {
public:
    void OnSensorHeartbeatReceived() { sensor_tracker_.OnHeartbeatReceived(); }
    void OnMotorHeartbeatReceived() { motor_tracker_.OnHeartbeatReceived(); }

    /// @brief Call once per 1000 ms supervisor monitoring tick.
    void OnMonitoringTick();

    bool IsSensorNodeFaulted() const { return sensor_tracker_.IsFaulted(); }
    bool IsMotorNodeFaulted() const { return motor_tracker_.IsFaulted(); }

private:
    NodeLivenessTracker sensor_tracker_;
    NodeLivenessTracker motor_tracker_;
};

}  // namespace hce::supervisor_node

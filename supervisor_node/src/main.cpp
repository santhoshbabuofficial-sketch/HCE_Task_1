/// @file main.cpp
/// @brief Supervisor Node entry point. Wires gpio_overlay -> CAN RX ->
///        supervisor_manager (Mediator) -> {alarm LED, RUN/STOP CAN TX}.
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "can_bus_driver.hpp"
#include "gpio_overlay.hpp"
#include "message_codec.hpp"
#include "supervisor_manager.hpp"

LOG_MODULE_REGISTER(supervisor_node_main, LOG_LEVEL_INF);

namespace {

using hce::supervisor_node::GpioOverlay;
using hce::supervisor_node::SupervisorManager;

constexpr uint32_t kMonitoringPeriodMs = 1000U;
constexpr size_t kMonitorThreadStackSize = 2048U;
constexpr int kMonitorThreadPriority = 5;

GpioOverlay g_overlay;
hce::supervisor::CanBusDriver* g_can_bus = nullptr;
SupervisorManager* g_supervisor_manager = nullptr;

alignas(hce::supervisor::CanBusDriver) uint8_t g_can_bus_storage[sizeof(hce::supervisor::CanBusDriver)];
alignas(SupervisorManager) uint8_t g_supervisor_manager_storage[sizeof(SupervisorManager)];

/// @brief Copies an incoming CAN-FD frame's data bytes into a fixed-size
///        payload without any heap allocation.
hce::supervisor::MessageCodec::Payload ExtractPayload(const can_frame* frame) {
    hce::supervisor::MessageCodec::Payload payload{};
    for (uint8_t i = 0U; i < hce::supervisor::MessageCodec::kPayloadBytes && i < frame->dlc; ++i) {
        payload[i] = frame->data[i];
    }
    return payload;
}

extern "C" void OnPressureRx(const device* /*dev*/, can_frame* frame, void* /*user_data*/) {
    g_supervisor_manager->OnPressureReceived(ExtractPayload(frame));
}

extern "C" void OnFlowRx(const device* /*dev*/, can_frame* frame, void* /*user_data*/) {
    g_supervisor_manager->OnFlowReceived(ExtractPayload(frame));
}

extern "C" void OnSensorHeartbeatRx(const device* /*dev*/, can_frame* /*frame*/, void* /*user_data*/) {
    g_supervisor_manager->OnSensorHeartbeatReceived();
}

extern "C" void OnMotorHeartbeatRx(const device* /*dev*/, can_frame* /*frame*/, void* /*user_data*/) {
    g_supervisor_manager->OnMotorHeartbeatReceived();
}

void MonitorThreadEntry(void* /*p1*/, void* /*p2*/, void* /*p3*/) {
    while (true) {
        g_supervisor_manager->Tick();
        k_msleep(kMonitoringPeriodMs);
    }
}

K_THREAD_STACK_DEFINE(g_monitor_thread_stack, kMonitorThreadStackSize);
k_thread g_monitor_thread{};

}  // namespace

int main() {
    LOG_INF("Supervisor Node booting");

    if (!g_overlay.Init()) {
        LOG_ERR("gpio_overlay init failed - halting");
        return -1;
    }

    g_can_bus = new (g_can_bus_storage) hce::supervisor::CanBusDriver(g_overlay.CanDevice());
    g_supervisor_manager = new (g_supervisor_manager_storage) SupervisorManager(g_overlay, *g_can_bus);

    if (!g_can_bus->Init()) {
        LOG_ERR("CAN bus init failed - halting");
        return -1;
    }

    g_can_bus->AddRxFilter(hce::supervisor::CanId::kPressure, OnPressureRx, nullptr);
    g_can_bus->AddRxFilter(hce::supervisor::CanId::kFlow, OnFlowRx, nullptr);
    g_can_bus->AddRxFilter(hce::supervisor::CanId::kSensorHeartbeat, OnSensorHeartbeatRx, nullptr);
    g_can_bus->AddRxFilter(hce::supervisor::CanId::kMotorHeartbeat, OnMotorHeartbeatRx, nullptr);

    k_thread_create(&g_monitor_thread, g_monitor_thread_stack, kMonitorThreadStackSize,
                     MonitorThreadEntry, nullptr, nullptr, nullptr, kMonitorThreadPriority, 0,
                     K_NO_WAIT);
    k_thread_name_set(&g_monitor_thread, "supervisor_monitor");

    LOG_INF("Supervisor Node running");
    return 0;
}

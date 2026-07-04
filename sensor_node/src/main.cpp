/// @file main.cpp
/// @brief Sensor Node entry point. Wires gpio_overlay -> drivers ->
///        sensor_manager -> {lcd_display, can_bus_driver} and starts the
///        periodic sampling and heartbeat threads.
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "can_bus_driver.hpp"
#include "flow_sensor.hpp"
#include "gpio_overlay.hpp"
#include "heartbeat.hpp"
#include "lcd_display.hpp"
#include "message_codec.hpp"
#include "pressure_sensor.hpp"
#include "sensor_manager.hpp"

LOG_MODULE_REGISTER(sensor_node_main, LOG_LEVEL_INF);

namespace {

using hce::sensor_node::CanSensorObserver;
using hce::sensor_node::FlowSensor;
using hce::sensor_node::GpioOverlay;
using hce::sensor_node::Heartbeat;
using hce::sensor_node::LcdDisplay;
using hce::sensor_node::LcdSensorObserver;
using hce::sensor_node::PressureSensor;
using hce::sensor_node::SensorManager;

constexpr uint32_t kSensorSamplePeriodMs = 1000U;
constexpr size_t kSensorThreadStackSize = 2048U;
constexpr size_t kHeartbeatThreadStackSize = 1024U;
constexpr int kSensorThreadPriority = 5;
constexpr int kHeartbeatThreadPriority = 5;

// Static-storage-duration singletons (Rule 1: no heap allocation).
GpioOverlay g_overlay;
hce::sensor::CanBusDriver* g_can_bus = nullptr;
PressureSensor* g_pressure_sensor = nullptr;
FlowSensor* g_flow_sensor = nullptr;
LcdDisplay* g_lcd = nullptr;
LcdSensorObserver* g_lcd_observer = nullptr;
CanSensorObserver* g_can_observer = nullptr;
SensorManager* g_sensor_manager = nullptr;
Heartbeat* g_heartbeat = nullptr;

// Placement-new storage pools (no dynamic allocation; Rule 1).
alignas(hce::sensor::CanBusDriver) uint8_t g_can_bus_storage[sizeof(hce::sensor::CanBusDriver)];
alignas(PressureSensor) uint8_t g_pressure_storage[sizeof(PressureSensor)];
alignas(FlowSensor) uint8_t g_flow_storage[sizeof(FlowSensor)];
alignas(LcdDisplay) uint8_t g_lcd_storage[sizeof(LcdDisplay)];
alignas(LcdSensorObserver) uint8_t g_lcd_observer_storage[sizeof(LcdSensorObserver)];
alignas(CanSensorObserver) uint8_t g_can_observer_storage[sizeof(CanSensorObserver)];
alignas(SensorManager) uint8_t g_sensor_manager_storage[sizeof(SensorManager)];
alignas(Heartbeat) uint8_t g_heartbeat_storage[sizeof(Heartbeat)];

k_work g_estop_work{};

/// @brief Deferred (thread-context) handler for an E-STOP broadcast.
///        CAN RX callbacks run in ISR context and must not perform I2C
///        transactions directly, so the LCD update is deferred here.
void EstopWorkHandler(k_work* /*work*/) {
    if (g_lcd != nullptr) {
        g_lcd->ShowEmergencyStop();
    }
}

/// @brief ISR-context CAN RX callback for the STOP command (0x100).
extern "C" void OnStopCommandRx(const device* /*dev*/, can_frame* /*frame*/, void* /*user_data*/) {
    k_work_submit(&g_estop_work);
}

void SensorThreadEntry(void* /*p1*/, void* /*p2*/, void* /*p3*/) {
    while (true) {
        g_sensor_manager->Tick();
        k_msleep(kSensorSamplePeriodMs);
    }
}

void HeartbeatThreadEntry(void* /*p1*/, void* /*p2*/, void* /*p3*/) {
    while (true) {
        g_heartbeat->Tick();
        k_msleep(Heartbeat::kPeriodMs);
    }
}

K_THREAD_STACK_DEFINE(g_sensor_thread_stack, kSensorThreadStackSize);
K_THREAD_STACK_DEFINE(g_heartbeat_thread_stack, kHeartbeatThreadStackSize);
k_thread g_sensor_thread{};
k_thread g_heartbeat_thread{};

}  // namespace

int main() {
    LOG_INF("Sensor Node booting");

    if (!g_overlay.Init()) {
        LOG_ERR("gpio_overlay init failed - halting");
        return -1;
    }

    g_can_bus = new (g_can_bus_storage) hce::sensor::CanBusDriver(g_overlay.CanDevice());
    g_pressure_sensor = new (g_pressure_storage) PressureSensor(g_overlay);
    g_flow_sensor = new (g_flow_storage) FlowSensor(g_overlay);
    g_lcd = new (g_lcd_storage) LcdDisplay(g_overlay);
    g_lcd_observer = new (g_lcd_observer_storage) LcdSensorObserver(*g_lcd);
    g_can_observer = new (g_can_observer_storage) CanSensorObserver(*g_can_bus);
    g_sensor_manager = new (g_sensor_manager_storage) SensorManager(*g_pressure_sensor, *g_flow_sensor);
    g_heartbeat = new (g_heartbeat_storage) Heartbeat(g_overlay, *g_can_bus);

    if (!g_can_bus->Init() || !g_pressure_sensor->Init() || !g_flow_sensor->Init() ||
        !g_lcd->Init()) {
        LOG_ERR("Peripheral init failed - halting");
        return -1;
    }

    g_sensor_manager->AddObserver(*g_lcd_observer);
    g_sensor_manager->AddObserver(*g_can_observer);

    k_work_init(&g_estop_work, EstopWorkHandler);
    g_can_bus->AddRxFilter(hce::sensor::CanId::kStopCommand, OnStopCommandRx, nullptr);

    k_thread_create(&g_sensor_thread, g_sensor_thread_stack, kSensorThreadStackSize,
                     SensorThreadEntry, nullptr, nullptr, nullptr, kSensorThreadPriority, 0,
                     K_NO_WAIT);
    k_thread_name_set(&g_sensor_thread, "sensor_sample");

    k_thread_create(&g_heartbeat_thread, g_heartbeat_thread_stack, kHeartbeatThreadStackSize,
                     HeartbeatThreadEntry, nullptr, nullptr, nullptr, kHeartbeatThreadPriority, 0,
                     K_NO_WAIT);
    k_thread_name_set(&g_heartbeat_thread, "sensor_heartbeat");

    LOG_INF("Sensor Node running");
    return 0;
}

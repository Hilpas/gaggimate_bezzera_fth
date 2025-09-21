#ifndef GAGGIMATECONTROLLER_H
#define GAGGIMATECONTROLLER_H
#include "ControllerConfig.h"
#include "NimBLEServerController.h"
#include <peripherals/DigitalInput.h>
#include <peripherals/DistanceSensor.h>
#include <peripherals/Heater.h>
#include <peripherals/LedController.h>
#include <peripherals/FlowSensor.h>
#include <peripherals/FlowThroughHeater.h>
#include <peripherals/Max31855Thermocouple.h>
#include <peripherals/NTCTemperatureSensor.h>
#include <peripherals/PressureSensor.h>
#include <peripherals/Pump.h>
#include <peripherals/SimpleRelay.h>
#include <vector>

constexpr double PING_TIMEOUT_SECONDS = 20.0;

constexpr int DETECT_EN_PIN = 40;
constexpr int DETECT_VALUE_PIN = 11;

#define TASK_PRIO_SENSORS       3  // Highest priority for data acquisition
#define TASK_PRIO_REGULATORS    2  // Medium priority for control loops
#define TASK_PRIO_BACKGROUND    1  // Low priority for non-critical tasks like BLE updates

class GaggiMateController {
  public:
    GaggiMateController();
    void setup(void);
    void loop(void);

    void registerBoardConfig(ControllerConfigBezzera config);

  private:
    void detectBoard();
    void detectAddon();
    void handlePingTimeout(void);
    void thermalRunawayShutdown(void);
    void startPidAutotune(void);
    void stopPidAutotune(void);
    void sendSensorData(void);

    ControllerConfigBezzera _config = ControllerConfigBezzera{};
    NimBLEServerController _ble;

    NTCTemperatureSensor *temperature_sensor = nullptr;
    FlowThroughHeater *heater = nullptr;
    FlowSensor *flowSensor = nullptr;
    SimpleRelay *valve = nullptr;
    SimpleRelay *alt = nullptr;
    Pump *pump = nullptr;
    DigitalInput *brewBtn = nullptr;
    DigitalInput *steamBtn = nullptr;
    PressureSensor *pressureSensor = nullptr;
    LedController *ledController = nullptr;
    DistanceSensor *distanceSensor = nullptr;

    std::vector<ControllerConfigBezzera> configs;

    unsigned long lastPingTime = 0;

    const char *LOG_TAG = "GaggiMateController";
};

#endif // GAGGIMATECONTROLLER_H

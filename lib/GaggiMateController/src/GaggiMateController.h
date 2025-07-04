#ifndef GAGGIMATECONTROLLER_H
#define GAGGIMATECONTROLLER_H
#include "ControllerConfig.h"
#include "NimBLEServerController.h"
#include <MAX31855.h>
#include <peripherals/DigitalInput.h>
#include <peripherals/DimmedPump.h>
#include <peripherals/FlowThroughHeater.h>
#include <peripherals/Heater.h>
#include <peripherals/Max31855Thermocouple.h>
#include <peripherals/NTCTemperatureSensor.h>
#include <peripherals/PressureSensor.h>
#include <peripherals/SimplePump.h>
#include <peripherals/SimpleRelay.h>
#include <vector>

constexpr double PING_TIMEOUT_SECONDS = 20.0;

constexpr int DETECT_EN_PIN = 40;
constexpr int DETECT_VALUE_PIN = 11;

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
    Heater *heater = nullptr;
    SimpleRelay *valve = nullptr;
    SimpleRelay *alt = nullptr;
    Pump *pump = nullptr;
    DigitalInput *brewBtn = nullptr;
    DigitalInput *steamBtn = nullptr;
    PressureSensor *pressureSensor = nullptr;

    std::vector<ControllerConfigBezzera> configs;

    unsigned long lastPingTime = 0;

    const char *LOG_TAG = "GaggiMateController";
};

#endif // GAGGIMATECONTROLLER_H

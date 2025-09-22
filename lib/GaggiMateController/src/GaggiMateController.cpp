#include "GaggiMateController.h"
#include "utilities.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <peripherals/DimmedPump.h>
#include <peripherals/SimplePump.h>
#include <freertos/task.h>
#include <peripherals/DimmedPump.h>
#include <peripherals/SimplePump.h>

GaggiMateController::GaggiMateController() {
    // disable board detection for now, use my config
    //configs.push_back(GM_STANDARD_REV_1X);
    //configs.push_back(GM_STANDARD_REV_2X);
    //configs.push_back(GM_PRO_REV_1x);
    //configs.push_back(GM_PRO_LEGO);
    configs.push_back(GM_BEZZERA);
}

void GaggiMateController::setup() {
    //Manually install the global ISR service once at the start 
    gpio_install_isr_service(0);

    delay(5000);
    // disable board detection for now, use my config
    // detectBoard();       
    _config = GM_BEZZERA;  
    detectAddon();

    this->temperature_sensor = new NTCTemperatureSensor(
        _config.temperaturePin, 10000.0f, 50000.0f, 25.0f, 3976.0f, [this](float temperature) { /* noop */ },
        [this]() { thermalRunawayShutdown(); });
    this->flowSensor = new FlowSensor(_config.flowSensorPin, [this](float flowRate) { /* noop */ });
    // disable stock heater
    //this->heater = new Heater(
    //    this->temperature_sensor, _config.heaterPin, [this]() { thermalRunawayShutdown(); },
    //    [this](float Kp, float Ki, float Kd) { _ble.sendAutotuneResult(Kp, Ki, Kd); });
    this->heater = new FlowThroughHeater(
        this->temperature_sensor, this->flowSensor, _config.heaterPin, _config.overheatPin, [this]() { thermalRunawayShutdown(); });
    this->valve = new SimpleRelay(_config.valvePin, _config.valveOn);
    this->alt = new SimpleRelay(_config.altPin, _config.altOn);
    if (_config.capabilites.pressure) {
        pressureSensor = new PressureSensor(_config.pressureSda, _config.pressureScl, [this](float pressure) { /* noop */ });
    }
    if (_config.capabilites.dimming) {
        pump = new DimmedPump(_config.pumpPin, _config.pumpSensePin, pressureSensor, flowSensor);
    } else {
        pump = new SimplePump(_config.pumpPin, _config.pumpOn, _config.capabilites.ssrPump ? 1000.0f : 5000.0f);
    }
    this->brewBtn = new DigitalInput(_config.brewButtonPin, [this](const bool state) { _ble.sendBrewBtnState(state); });
    this->steamBtn = new DigitalInput(_config.steamButtonPin, [this](const bool state) { _ble.sendSteamBtnState(state); });

    // 5-Pin peripheral port
    //Wire.begin(_config.sunriseSdaPin, _config.sunriseSclPin, 400000);
    //this->ledController = new LedController(&Wire);
    //this->distanceSensor = new DistanceSensor(&Wire, [this](int distance) { _ble.sendTofMeasurement(distance); });
    //if (this->ledController->isAvailable()) {
    //    _config.capabilites.ledControls = true;
    //    _config.capabilites.tof = true;
    //    _ble.registerLedControlCallback(
    //        [this](uint8_t channel, uint8_t brightness) { ledController->setChannel(channel, brightness); });
    //}
    _config.capabilites.ledControls = false;
    _config.capabilites.tof = false;

    String systemInfo = make_system_info(_config);
    _ble.initServer(systemInfo);

    this->temperature_sensor->setup();
    this->heater->setup();
    this->valve->setup();
    this->alt->setup();
    this->pump->setup();
    this->brewBtn->setup();
    this->steamBtn->setup();
    this->flowSensor->setup();
    if (_config.capabilites.pressure) {
        pressureSensor->setup();
        _ble.registerPressureScaleCallback([this](float scale) { this->pressureSensor->setScale(scale); });
    }
    if (_config.capabilites.ledControls) {
        this->ledController->setup();
    }
    if (_config.capabilites.tof) {
        this->distanceSensor->setup();
    }

    // Initialize last ping time
    lastPingTime = millis();

    _ble.registerOutputControlCallback([this](bool valve, float pumpSetpoint, float heaterSetpoint) {
        this->pump->setPower(pumpSetpoint);
        this->valve->set(valve);
        this->heater->setSetpoint(heaterSetpoint);
        if (!_config.capabilites.dimming) {
            return;
        }
        auto dimmedPump = static_cast<DimmedPump *>(pump);
        dimmedPump->setValveState(valve);
    });
    _ble.registerAdvancedOutputControlCallback(
        [this](bool valve, float heaterSetpoint, bool pressureTarget, float pressure, float flow) {
            this->valve->set(valve);
            this->heater->setSetpoint(heaterSetpoint);
            if (!_config.capabilites.dimming) {
                return;
            }
            auto dimmedPump = static_cast<DimmedPump *>(pump);
            if (pressureTarget) {
                dimmedPump->setPressureTarget(pressure, flow);
            } else {
                dimmedPump->setFlowTarget(flow, pressure);
            }
            dimmedPump->setValveState(valve);
        });
    _ble.registerAltControlCallback([this](bool state) { this->alt->set(state); });   
    _ble.registerPidControlCallback([this](float Kp, float Ki, float Kd) { this->heater->setTunings(Kp, Ki, Kd); });
    _ble.registerStandbyPidControlCallback([this](float Kp, float Ki, float Kd) { this->heater->setTunings(Kp, Ki, Kd); });
    _ble.registerFlowPidControlCallback([this](float Kp, float Ki, float Kd) { 
            auto dimmedPump = static_cast<DimmedPump *>(pump);
            dimmedPump->setFlowTuning(Kp, Ki, Kd); 
        });
    _ble.registerPumpModelCoeffsCallback([this](float a, float b, float c, float d) {
        if (_config.capabilites.dimming) {
            auto dimmedPump = static_cast<DimmedPump *>(pump);
            // Check if this is a flow measurement call (a and b are flow measurements, c and d are nan)
            if (isnan(c) && isnan(d)) {
                dimmedPump->setPumpFlowCoeff(a, b); // a = oneBarFlow, b = nineBarFlow
            } else {
                dimmedPump->setPumpFlowPolyCoeffs(a, b, c, d); // a, b, c, d are polynomial coefficients
            }
        }
    });
    _ble.registerPingCallback([this]() {
        lastPingTime = millis();
        ESP_LOGV(LOG_TAG, "Ping received, system is alive");
    });
    // disable autotune from Display, use the preconfigured values
    //_ble.registerAutotuneCallback([this](int goal, int windowSize) { this->heater->autotune(goal, windowSize); });
    _ble.registerTareCallback([this]() {
        if (!_config.capabilites.dimming) {
            return;
        }
        auto dimmedPump = static_cast<DimmedPump *>(pump);
        dimmedPump->tare();
    });
    ESP_LOGI(LOG_TAG, "Initialization done");
}

void GaggiMateController::loop() {
    unsigned long now = millis();
    if ((now - lastPingTime) / 1000 > PING_TIMEOUT_SECONDS) {
        handlePingTimeout();
    }
    sendSensorData();
    delay(250);
}

void GaggiMateController::registerBoardConfig(ControllerConfigBezzera config) { configs.push_back(config); }

void GaggiMateController::detectBoard() {
    pinMode(DETECT_EN_PIN, OUTPUT);
    pinMode(DETECT_VALUE_PIN, INPUT_PULLDOWN);
    digitalWrite(DETECT_EN_PIN, HIGH);
    uint16_t millivolts = analogReadMilliVolts(DETECT_VALUE_PIN);
    digitalWrite(DETECT_EN_PIN, LOW);
    int boardId = round(((float)millivolts) / 100.0f - 0.5f);
    ESP_LOGI(LOG_TAG, "Detected Board ID: %d", boardId);
    for (ControllerConfigBezzera config : configs) {
        if (config.autodetectValue == boardId) {
            _config = config;
            ESP_LOGI(LOG_TAG, "Using Board: %s", _config.name.c_str());
            return;
        }
    }
    ESP_LOGW(LOG_TAG, "No compatible board detected.");
    delay(5000);
    ESP.restart();
}

void GaggiMateController::detectAddon() {
    // TODO: Add I2C scanning for extensions
}

void GaggiMateController::handlePingTimeout() {
    ESP_LOGE(LOG_TAG, "Ping timeout detected. Turning off heater and pump for safety.\n");
    // Turn off the heater and pump as a safety measure
    this->heater->setSetpoint(0);
    this->pump->setPower(0);
    this->valve->set(false);
    this->alt->set(false);
}

void GaggiMateController::thermalRunawayShutdown() {
    ESP_LOGE(LOG_TAG, "Thermal runaway detected! Turning off heater and pump!\n");
    // Turn off the heater and pump immediately
    this->heater->setSetpoint(0);
    this->pump->setPower(0);
    this->valve->set(false);
    this->alt->set(false);
    _ble.sendError(ERROR_CODE_RUNAWAY);
}

void GaggiMateController::sendSensorData() {
    if (_config.capabilites.pressure) {
        auto dimmedPump = static_cast<DimmedPump *>(pump);
        _ble.sendSensorData(this->temperature_sensor->read(), this->pressureSensor->getPressure(), dimmedPump->getPuckFlow(),
                            dimmedPump->getPumpFlow());
        _ble.sendVolumetricMeasurement(dimmedPump->getCoffeeVolume());
    } else {
        _ble.sendSensorData(this->temperature_sensor->read(), 0.0f, 0.0f, 0.0f);
    }
}

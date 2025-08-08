#include "DimmedPump.h"

#include <GaggiMateController.h>

DimmedPump::DimmedPump(uint8_t ssr_pin, uint8_t sense_pin, PressureSensor *pressure_sensor)
    : _ssr_pin(ssr_pin), _sense_pin(sense_pin), _psm(_sense_pin, _ssr_pin, 100, FALLING, 2, 4), _pressureSensor(pressure_sensor),
      _pressureController(0.03f, &_targetPressure, &_currentPressure, &_controllerPower, &_valveStatus) {
    _psm.set(0);
}

void DimmedPump::setup() {
    _cps = _psm.cps();
    if (_cps > 70) {
        _cps = _cps / 2;
    }
    xTaskCreate(loopTask, "DimmedPump::loop", configMINIMAL_STACK_SIZE * 4, this, 1, &taskHandle);
}

void DimmedPump::loop() {
    _currentPressure = _pressureSensor->getRawPressure();
    updatePower();

    // Log the current flow for debugging purposes
    //static uint32_t lastLogTime = 0;
    //uint32_t now = millis();
    //if (now - lastLogTime >= 1000) {
    //    ESP_LOGI(LOG_TAG, "Flow rate: %.2f ml/min", getFlow() * 60.0f);
    //    lastLogTime = now;
    //}
}

void DimmedPump::setPower(float setpoint) {
    ESP_LOGV(LOG_TAG, "Setting power to %2f", setpoint);
    _mode = ControlMode::POWER;
    _targetPressure = setpoint == 100 ? 20.0f : 0.0f;
    _flowLimit = 0.0f;
    _power = std::clamp(setpoint, 0.0f, 100.0f); 
    _psm.set(static_cast<int>(_power)); 
}

float DimmedPump::getCoffeeVolume() { return _pressureController.getcoffeeOutputEstimate(); }

float DimmedPump::getFlow() { return _pressureController.getFlowPerSecond(); }

void DimmedPump::tare() {
    _pressureController.tare();
    _pressureController.reset();
}

void DimmedPump::loopTask(void *arg) {
    auto *pump = static_cast<DimmedPump *>(arg);
    TickType_t lastWake = xTaskGetTickCount();
    while (true) {
        pump->loop();
        xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(30));
    }
}

void DimmedPump::updatePower() {
    _pressureController.update();

    // Ramp start logic
    //if (_power > 0 && !rampStart) {
    //    rampStartTime = millis();
    //    rampStart = true;
    //} else if(_power == 0 && rampStart) {
    //    rampStart = false;
    //}

    switch (_mode) {
    case ControlMode::PRESSURE:
        _power = calculatePowerForPressure(_targetPressure, _currentPressure, _flowLimit);
        break;

    case ControlMode::FLOW:
        _power = calculatePowerForFlow(_targetFlow, _currentPressure, _pressureLimit);
        break;

    case ControlMode::POWER:
        // Ramp start test

        // Erst Rampe, dann ab Druck 3 bar auf 50%
        //if(rampStart) {
        //    uint32_t now = millis();
        //    float elapsed = now - rampStartTime;

        //    if (elapsed < rampDurationMs) {
        //        // Ramp from 5 to targetPumpPower
        //        _power = rampStartValue + ((targetPumpPower - rampStartValue) * (elapsed / rampDurationMs));
        //    } else if (elapsed >= rampDurationMs && _currentPressure >= 3.0f) {
        //        _power = 50;
        //    } 
        //}
        
         //Erst 20% bis 3 bar, dann Rampe auf 50%
        if (_power > 0 && _currentPressure < 4.0f) 
        {
            _power = rampStartValue;
        } 
        else if (_power > 0 && _currentPressure >= 4.0f ) 
        {
            if (!rampStart) {
                rampStartTime = millis();
                rampStart = true;
            }
        
            uint32_t now = millis();
            float elapsed = now - rampStartTime;
        
            if (elapsed < rampDurationMs) {
               // Ramp from 5 to targetPumpPower
                _power = rampStartValue + ((targetPumpPower - rampStartValue) * (elapsed / rampDurationMs));
            }   
            else
            {
                _power = targetPumpPower;
            }     
        } 
        else if(_power == 0 && rampStart) 
        {
            rampStart = false;
        }
        
        break;
    }

    //printf("Power: %2f, Pressure: %2f\n", _power, _currentPressure);

    _psm.set(static_cast<int>(_power));
}

float DimmedPump::calculateFlowRate(float pressure) const {
    float flow = BASE_FLOW_RATE;
    float pressurePercentage = pressure / MAX_PRESSURE;
    return flow * pressurePercentage;
}

float DimmedPump::calculatePowerForPressure(float targetPressure, float currentPressure, float flowLimit) {
    return _controllerPower;
}

float DimmedPump::calculatePowerForFlow(float targetFlow, float currentPressure, float pressureLimit) const {
    float maxFlow = calculateFlowRate(currentPressure) * _cps;
    float powerRatio = std::clamp(targetFlow / maxFlow, 0.0f, 1.0f);
    float basePower = powerRatio * 100.0f;

    if (pressureLimit > 0 && currentPressure > pressureLimit) {
        return 0.0f;
    }

    return basePower;
}

void DimmedPump::setFlowTarget(float targetFlow, float pressureLimit) {
    _mode = ControlMode::FLOW;
    _targetFlow = targetFlow;
    _pressureLimit = pressureLimit;
}

void DimmedPump::setPressureTarget(float targetPressure, float flowLimit) {
    _mode = ControlMode::PRESSURE;
    _targetPressure = targetPressure;
    _flowLimit = flowLimit;
}

void DimmedPump::setValveState(bool open) { _valveStatus = open; }

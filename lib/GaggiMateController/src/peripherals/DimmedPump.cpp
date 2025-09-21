#include "DimmedPump.h"
#include <GaggiMateController.h>

DimmedPump::DimmedPump(uint8_t ssr_pin, uint8_t sense_pin, PressureSensor *pressure_sensor, FlowSensor *flow_sensor)
    : _ssr_pin(ssr_pin), _sense_pin(sense_pin), _psm(_sense_pin, _ssr_pin, 100, FALLING, 2, 4), _pressureSensor(pressure_sensor), _flowSensor(flow_sensor),
      _pressureController(0.03f, &_ctrlPressure, &_ctrlFlow, &_currentPressure, &_controllerPower, &_valveStatus),
      _flowController(0.03f, &_targetFlow, &_currentFlow, &_controllerFlowPower, &_valveStatus) {
      _psm.set(0);
    }

void DimmedPump::setup() {
    _cps = _psm.cps();
    if (_cps > 70) {
        _cps = _cps / 2;
    }
    xTaskCreate(loopTask, "DimmedPump::loop", configMINIMAL_STACK_SIZE * 4, this, TASK_PRIO_REGULATORS, &taskHandle);
}

void DimmedPump::loop() {
    _currentPressure = _pressureSensor->getRawPressure();
    _currentFlow = _flowSensor->read_g_s();
    updatePower();
}

void DimmedPump::setPower(float setpoint) {
    ESP_LOGV(LOG_TAG, "Setting power to %2f", setpoint);
    _ctrlPressure = setpoint > 0 ? 20.0f : 0.0f;
    _mode = ControlMode::POWER;
    _power = std::clamp(setpoint, 0.0f, 100.0f);
    _controllerPower = _power; // Feed manual control back into pressure controller
    if (_power == 0.0f) {
        _currentFlow = 0.0f;
    }
    _psm.set(static_cast<int>(_power));
}

float DimmedPump::getCoffeeVolume() { return _pressureController.getCoffeeOutputEstimate(); }

void DimmedPump::tare() {
    _pressureController.tare();
    _pressureController.reset();
}

void DimmedPump::loopTask(void *arg) {
    auto *pump = static_cast<DimmedPump *>(arg);
    TickType_t lastWake = xTaskGetTickCount();
    while (true) {
        pump->loop();
        xTaskDelayUntil(&lastWake, pdMS_TO_TICKS(PUMP_LOOP_INTERVAL_MS));
    }
}

void DimmedPump::updatePower() {
    //_pressureController.update();
    //_flowController.update();
    //
    //if (_power > 0 && _currentPressure < 6.0f) 
    //{
    //    //4.2g/s ~250ml/min
    //    setFlowTarget(3.9f, 0.0f);
    //    float powerInt = calculatePowerForFlow(3.9f, _currentPressure, _pressureLimit);
    //    _power = std::clamp(powerInt, 0.0001f, 100.0f);
    //    printf("Mode: %d, Power: %2f, FlowSetpoint: %2f, CurrentFlow: %2f\n", static_cast<int>(_mode), powerInt, _targetFlow, _currentFlow);
    //}
    //else if (_power > 0 && _currentPressure >= 6.0f) 
    //{
    //    setPressureTarget(9.0f, 0.0f);
    //        float powerInt = _controllerPower;
    //    _power = std::clamp(powerInt, 0.0001f, 100.0f);
    //    //printf("Mode: %d, Power: %2f, PressureSetpoint: %2f, CurrentPressure: %2f\n", static_cast<int>(_mode), _power, _targetPressure, _currentPressure);
    //}
    //else if (_power == 0) 
    //{
    //    _targetFlow = 0.0f;
    //    _ctrlPressure = 0.0f;
    //    _pressureLimit = 0.0f;
    //    _ctrlFlow = 0.0f;
    //}  

    //_psm.set(static_cast<int>(_power));

    _pressureController.update(static_cast<PressureController::ControlMode>(_mode));
    if (_mode != ControlMode::POWER) {
        _power = _controllerPower;
    }
    _psm.set(static_cast<int>(_power));
}

float DimmedPump::calculateFlowRate(float pressure) const {
    float flow = BASE_FLOW_RATE;
    float pressurePercentage = pressure / MAX_PRESSURE;
    return flow * pressurePercentage;
}

float DimmedPump::calculatePowerForFlow(float targetFlow, float currentPressure, float pressureLimit) const {
    //float maxFlow = calculateFlowRate(currentPressure) * _cps;
    //float powerRatio = std::clamp(targetFlow / maxFlow, 0.0f, 1.0f);
    //float basePower = powerRatio * 100.0f;

    if (pressureLimit > 0 && currentPressure > pressureLimit) {
        return 0.0f;
    }

    //return basePower;

    return _controllerFlowPower;
}

void DimmedPump::setFlowTarget(float targetFlow, float pressureLimit) {
    _mode = ControlMode::FLOW;
    _ctrlFlow = targetFlow;
    _ctrlPressure = pressureLimit;
    _pressureController.setPressureLimit(pressureLimit);
}

void DimmedPump::setFlowTuning(float Kp, float Ki, float Kd) {
    _flowController.setTunings(Kp, Ki, Kd);
    ESP_LOGI(LOG_TAG, "Kp: %.2f, Ki: %.2f, Kd: %.2f", Kp, Ki, Kd);
}

void DimmedPump::setPressureTarget(float targetPressure, float flowLimit) {
    _mode = ControlMode::PRESSURE;
    _ctrlFlow = flowLimit;
    _ctrlPressure = targetPressure;
    _pressureController.setFlowLimit(flowLimit);
}

void DimmedPump::setValveState(bool open) { _valveStatus = open; }

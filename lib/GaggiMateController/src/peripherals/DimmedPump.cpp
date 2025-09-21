#include "DimmedPump.h"
#include <GaggiMateController.h>

DimmedPump::DimmedPump(uint8_t ssr_pin, uint8_t sense_pin, PressureSensor *pressure_sensor, FlowSensor *flow_sensor)
    : _ssr_pin(ssr_pin), _sense_pin(sense_pin), 
      _psm(_sense_pin, _ssr_pin), 
      _pressureSensor(pressure_sensor), _flowSensor(flow_sensor),
      _pressureController(0.03f, &_ctrlPressure, &_ctrlFlow, &_currentPressure, &_controllerPower, &_valveStatus) {
      //_flowController(0.03f, &_targetFlow, &_currentFlow, &_controllerFlowPower, &_valveStatus)
    }

void DimmedPump::setup() {
    _psm.begin();
    xTaskCreate(loopTask, "DimmedPump::loop", configMINIMAL_STACK_SIZE * 4, this, TASK_PRIO_REGULATORS, &taskHandle);
}

void DimmedPump::loop() {
    _currentPressure = _pressureSensor->getRawPressure();
    _currentFlow = _flowSensor->read_g_s();
    updatePower();
    //_currentFlow = 0.1f * (_pressureController.getPumFlowRate() * 1000000.0f) + 0.9f * _currentFlow;
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

    _psm.setPower(_power);
}

float DimmedPump::getCoffeeVolume() { return _pressureController.getcoffeeOutputEstimate(); }

float DimmedPump::getPumpFlow() { return _currentFlow; }

float DimmedPump::getPuckFlow() { return _pressureController.getCoffeeFlowRate(); }

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
    _pressureController.update(static_cast<PressureController::ControlMode>(_mode));
    if (_mode != ControlMode::POWER) {
        _power = _controllerPower;
    }
    _psm.setPower(_power);
}

void DimmedPump::setFlowTarget(float targetFlow, float pressureLimit) {
    _mode = ControlMode::FLOW;
    _ctrlFlow = targetFlow;
    _ctrlPressure = pressureLimit;
    _pressureController.setPressureLimit(pressureLimit);
}

void DimmedPump::setFlowTuning(float Kp, float Ki, float Kd) {
    //_flowController.setTunings(Kp, Ki, Kd);
    ESP_LOGI(LOG_TAG, "Kp: %.2f, Ki: %.2f, Kd: %.2f", Kp, Ki, Kd);
}

void DimmedPump::setPressureTarget(float targetPressure, float flowLimit) {
    _mode = ControlMode::PRESSURE;
    _ctrlFlow = flowLimit;
    _ctrlPressure = targetPressure;
    _pressureController.setFlowLimit(flowLimit);
}

void DimmedPump::setValveState(bool open) { _valveStatus = open; }

void DimmedPump::setPumpFlowCoeff(float oneBarFlow, float nineBarFlow) {
    _pressureController.setPumpFlowCoeff(oneBarFlow, nineBarFlow);
}

void DimmedPump::setPumpFlowPolyCoeffs(float a, float b, float c, float d) {
    _pressureController.setPumpFlowPolyCoeffs(a, b, c, d);
}

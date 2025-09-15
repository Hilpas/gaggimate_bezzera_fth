#include "FlowThroughHeater.h"
#include <Arduino.h>
#include <algorithm>

constexpr float TUNER_OUTPUT_SPAN = 500.0f; // this is the relay duty cycle 500ms is 100% duty cycle

FlowThroughHeater::FlowThroughHeater(TemperatureSensor *sensor_temperature, FlowSensor *sensor_flow, uint8_t heaterPin, uint8_t overheatPin, const heater_error_callback_t &error_callback)
    : sensor_temperature(sensor_temperature), sensor_flow(sensor_flow), heaterPin(heaterPin), overheatPin(overheatPin), taskHandle(nullptr), error_callback(error_callback) {

    flowThroughPID = new FlowThroughPID(&output, &temperature, &flow, &setpoint);
}

void FlowThroughHeater::setup() {
    pinMode(heaterPin, OUTPUT);
    pinMode(overheatPin, INPUT_PULLDOWN);
    setupPid();
    xTaskCreate(loopTask, "FlowThroughHeater::loop", configMINIMAL_STACK_SIZE * 4, this, 1, &taskHandle);
}

void FlowThroughHeater::setupPid() {

    flowThroughPID->setSamplingFrequency(10.0f); //Hz
    flowThroughPID->setCtrlOutputLimits(0.0f, TUNER_OUTPUT_SPAN);
    flowThroughPID->activateFeedForward(true);
    flowThroughPID->setKp(Kp);
    flowThroughPID->setKi(Ki);
    flowThroughPID->setKd(Kd);
    flowThroughPID->reset();
}

void FlowThroughHeater::loop() {
    if (temperature <= 0.0f || setpoint <= 0.0f || digitalRead(overheatPin) == HIGH) {
        flowThroughPID->setMode(FlowThroughPID::Control::manual);
        digitalWrite(heaterPin, LOW);
        relayStatus = false;
        temperature = sensor_temperature->read();
        flow = sensor_flow->read();
        return;
    }
    flowThroughPID->setMode(FlowThroughPID::Control::automatic);

    loopPid();
}

void FlowThroughHeater::setSetpoint(float setpoint) {
    if (this->setpoint != setpoint) {
        this->setpoint = setpoint;
        ESP_LOGV(LOG_TAG, "Set setpoint %f°C", setpoint);
    }
}

void FlowThroughHeater::setTunings(float Kp, float Ki, float Kd) {
    if (flowThroughPID->getKp() != Kp || flowThroughPID->getKi() != Ki || flowThroughPID->getKd() != Kd) {
        flowThroughPID->setControllerPIDGains(Kp, Ki, Kd, 0.0f);
        flowThroughPID->reset();
        ESP_LOGV(LOG_TAG, "Set tunings to Kp: %f, Ki: %f, Kd: %f", Kp, Ki, Kd);
    }
}

void FlowThroughHeater::loopPid() {
    softPwm(TUNER_OUTPUT_SPAN);
    temperature = sensor_temperature->read();
    flow = sensor_flow->read();
    if (flowThroughPID->update()) {
        plot(output, 1.0f, 1);
    }
}

float FlowThroughHeater::softPwm(uint32_t windowSize) {
    // software PWM timer
    unsigned long msNow = millis();
    if (msNow - windowStartTime >= windowSize) {
        windowStartTime = msNow;
    }
    float optimumOutput = output;

    // PWM relay output
    if (!relayStatus && static_cast<unsigned long>(optimumOutput) > (msNow - windowStartTime)) {
        if (msNow > nextSwitchTime) {
            nextSwitchTime = msNow;
            relayStatus = true;
            digitalWrite(heaterPin, HIGH);
        }
    } else if (relayStatus && static_cast<unsigned long>(optimumOutput) < (msNow - windowStartTime)) {
        if (msNow > nextSwitchTime) {
            nextSwitchTime = msNow;
            relayStatus = false;
            digitalWrite(heaterPin, LOW);
        }
    }
    return optimumOutput;
}

void FlowThroughHeater::plot(float optimumOutput, float outputScale, uint8_t everyNth) {
    if (plotCount >= everyNth) {
        plotCount = 1;
        ESP_LOGI(LOG_TAG, "Setpoint: %.2f, Input: %.2f, Output: %.2f, Kp: %.2f, Ki: %.2f, Kd: %.2f",
                 setpoint, temperature, optimumOutput * outputScale, flowThroughPID->getKp(), flowThroughPID->getKi(), flowThroughPID->getKd());
    } else
        plotCount++;
}

void FlowThroughHeater::loopTask(void *arg) {
    auto *heater = static_cast<FlowThroughHeater *>(arg);
    while (true) {
        heater->loop();
        vTaskDelay(100 / portTICK_PERIOD_MS); // 100ms delay
    }
}
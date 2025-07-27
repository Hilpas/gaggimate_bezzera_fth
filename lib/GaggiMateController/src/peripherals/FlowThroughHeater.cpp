#include "FlowThroughHeater.h"
#include <Arduino.h>
#include <algorithm>

constexpr float TUNER_INPUT_SPAN = 160.0f;
constexpr float TUNER_OUTPUT_SPAN = 1000.0f; //-> 1Hz

FlowThroughHeater::FlowThroughHeater(TemperatureSensor *sensor, uint8_t heaterPin, uint8_t overheatPin, const heater_error_callback_t &error_callback)
    : sensor(sensor), heaterPin(heaterPin), overheatPin(overheatPin), taskHandle(nullptr), error_callback(error_callback) {

    simplePid = new SimplePID(&output, &temperature, &setpoint);
}

void FlowThroughHeater::setup() {
    pinMode(heaterPin, OUTPUT);
    pinMode(overheatPin, INPUT_PULLDOWN);
    setupPid();
    xTaskCreate(loopTask, "FlowThroughHeater::loop", configMINIMAL_STACK_SIZE * 4, this, 1, &taskHandle);
}

void FlowThroughHeater::setupPid() {

    simplePid->setSamplingFrequency(TUNER_OUTPUT_SPAN / 1000.0f); 
    simplePid->setCtrlOutputLimits(0.0f, TUNER_OUTPUT_SPAN);
    simplePid->activateSetPointFilter(false);
    simplePid->activateFeedForward(false);
    simplePid->setKp(Kp);
    simplePid->setKi(Ki);
    simplePid->setKd(Kd);
    simplePid->reset();
}

void FlowThroughHeater::loop() {
    if (temperature <= 0.0f || setpoint <= 0.0f || digitalRead(overheatPin) == HIGH) {
        simplePid->setMode(SimplePID::Control::manual);
        digitalWrite(heaterPin, LOW);
        relayStatus = false;
        temperature = sensor->read();
        return;
    }
    simplePid->setMode(SimplePID::Control::automatic);

    loopPid();
}

void FlowThroughHeater::setSetpoint(float setpoint) {
    if (this->setpoint != setpoint) {
        this->setpoint = setpoint;
        ESP_LOGV(LOG_TAG, "Set setpoint %f°C", setpoint);
    }
}

void FlowThroughHeater::setTunings(float Kp, float Ki, float Kd) {
    if (simplePid->getKp() != Kp || simplePid->getKi() != Ki || simplePid->getKd() != Kd) {
        simplePid->setControllerPIDGains(Kp, Ki, Kd, 0.0f);
        simplePid->reset();
        ESP_LOGV(LOG_TAG, "Set tunings to Kp: %f, Ki: %f, Kd: %f", Kp, Ki, Kd);
    }
}

void FlowThroughHeater::loopPid() {
    softPwm(TUNER_OUTPUT_SPAN);
    temperature = sensor->read();
    if (simplePid->update()) {
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
        ESP_LOGI(LOG_TAG, "Setpoint: %.2f, Input: %.2f, Output: %.2f, Kp: %.2f, Ki: %.2f, Kd: %.2f, Filtered Setpoint: %.2f",
                 setpoint, temperature, optimumOutput * outputScale, simplePid->getKp(), simplePid->getKi(), simplePid->getKd(),
                 simplePid->getSetpointFiltered());
    } else
        plotCount++;
}

void FlowThroughHeater::loopTask(void *arg) {
    auto *heater = static_cast<FlowThroughHeater *>(arg);
    while (true) {
        heater->loop();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
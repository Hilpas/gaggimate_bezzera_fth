#ifndef FLOWTHROUGHHEATER_H
#define FLOWTHROUGHHEATER_H

#include "NTCTemperatureSensor.h"
#include "TemperatureSensor.h"
#include "FlowSensor.h"
#include "FlowThroughPID.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

using heater_error_callback_t = std::function<void()>;


constexpr int FLOW_REGULATOR_LOOP_INTERVAL_MS = 20; // 20ms fastest pulses sensor can make are 56ms

class FlowThroughHeater {
  public:
    FlowThroughHeater(TemperatureSensor *sensor_temperature, FlowSensor *sensor_flow, uint8_t heaterPin, uint8_t overheatPin, const heater_error_callback_t &error_callback);
    void setup();
    void loop();

    void setSetpoint(float setpoint);
    void setTunings(float Kp, float Ki, float Kd, float Kf);
    void setRegulatorReset();

  private:
    void setupPid();
    void loopPid();
    float softPwm(uint32_t windowSize);
    void plot(float optimumOutput, float outputScale, uint8_t everyNth);

    TemperatureSensor *sensor_temperature;
    FlowSensor *sensor_flow;
    uint8_t heaterPin;
    uint8_t overheatPin;
    xTaskHandle taskHandle;
    FlowThroughPID *flowThroughPID = nullptr;

    heater_error_callback_t error_callback;

    float temperature = 0.0f;
    float flow = 0.0f;
    float output = 0.0f;
    float setpoint = 0.0f;
    float Kp = 0.15;
    float Ki = 0.0;
    float Kd = 0.0;
    float Kf = 0.0; 
    int plotCount = 0;

    bool relayStatus = false;
    unsigned long windowStartTime = 0;
    unsigned long nextSwitchTime = 0;

    const char *LOG_TAG = "FlowThroughHeater";
    static void loopTask(void *arg);
};

#endif // FLOWTHROUGHHEATER_H
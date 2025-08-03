#ifndef FLOWSENSOR_H
#define FLOWSENSOR_H

#include <Arduino.h>
#include <functional>

constexpr int FLOW_LOOP_INTERVAL_MS = 1000;
constexpr float ML_PER_PULSE = 0.4797f; // according to datasheet 0.5195mL/Pulse, but adjusted due to calibration
using flow_callback_t = std::function<void(float)>;

class FlowSensor {
public:
    FlowSensor(uint8_t pin, const flow_callback_t &callback = nullptr);
    ~FlowSensor() = default;

    float read();
    void setup();
    void loop();
    inline float getFlowRate() const { return _flowRate; }

private:
    static void IRAM_ATTR isrHandlerStatic(void* arg);
    void isrHandler();
    static void loopTask(void *arg);

    uint8_t _pin;
    volatile uint64_t _lastPulseTimeUs = 0;
    volatile uint64_t _pulseDeltaUs = 0;
    volatile float _flowRate = 0.0f;
    flow_callback_t _callback;
    xTaskHandle taskHandle = nullptr;

    const char *LOG_TAG = "FlowSensor";
};

#endif // FLOWSENSOR_H
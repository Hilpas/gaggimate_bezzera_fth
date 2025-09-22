#ifndef FLOWSENSOR_H
#define FLOWSENSOR_H

#include <Arduino.h>
#include <functional>

constexpr int FLOW_LOOP_INTERVAL_MS = 20; // 20ms fastest pulses sensor can make are 56ms
constexpr float ML_PER_PULSE = 0.43900; // according to datasheet 0.5195mL/Pulse, but adjusted due to calibration
using flow_callback_t = std::function<void(float)>;

class FlowSensor {
public:
    FlowSensor(uint8_t pin, const flow_callback_t &callback = nullptr);
    ~FlowSensor() = default;

    float read();
    float read_g_s();
    float read_instantaneous();

    void setup();
    void loop();
    inline float getFlowRate() const { return _flowRate; }

private:
    static void IRAM_ATTR isrHandlerStatic(void* arg);
    void isrHandler();
    static void loopTask(void *arg);

    uint8_t _pin;
    volatile uint64_t _lastPulseTimeUs = 0;
    uint64_t _lastLogTimeUs = 0;
    volatile uint64_t _pulseDeltaUs = 0;
    volatile float _flowRate = 0.0f;
    volatile float _filteredFlowRate = 0.0f;
    volatile float _instantaneousFlowRate = 0.0f;
    flow_callback_t _callback;
    xTaskHandle taskHandle = nullptr;

    // For filtering the flow rate
    const float STABLE_ALPHA = 0.15f;
    const float FAST_ALPHA = 0.60f;    

    QueueHandle_t _pulseQueue; // <-- ADD THIS: Handle for our queue
    const int PULSE_QUEUE_LENGTH = 30; // Max pulses to queue before calculations

    const char *LOG_TAG = "FlowSensor";
};

#endif // FLOWSENSOR_H
#include "FlowSensor.h"
#include <esp_timer.h>

FlowSensor::FlowSensor(uint8_t pin, const flow_callback_t &callback)
    : _pin(pin), _callback(callback) {}

float FlowSensor::read() { return _flowRate; }
float FlowSensor::read_g_s() { return _flowRate / 60.0f; }

void FlowSensor::setup() {
    pinMode(_pin, INPUT_PULLUP);
    attachInterruptArg(_pin, &FlowSensor::isrHandlerStatic, this, RISING);
    ESP_LOGI(LOG_TAG, "Initializing flow sensor on pin: %d", _pin);
    xTaskCreate(loopTask, "FlowSensor::loop", configMINIMAL_STACK_SIZE * 4, this, 1, &taskHandle);
}

void FlowSensor::loop() {
    // Zero flow rate if no pulse for 1 second
    uint64_t nowUs = esp_timer_get_time();
    if ((nowUs - _lastPulseTimeUs) > 1000000 && _flowRate != 0.0f) {
        _flowRate = 0.0f;
        ESP_LOGV(LOG_TAG, "Flow rate zeroed due to timeout");
    }

    // For debugging, log the current flow rate
    ESP_LOGI(LOG_TAG, "Flow rate: %.2f ml/min", _flowRate);
}

void FlowSensor::isrHandlerStatic(void* arg) {
    static_cast<FlowSensor*>(arg)->isrHandler();
}

void FlowSensor::isrHandler() {
    uint64_t now = esp_timer_get_time();
    if (_lastPulseTimeUs != 0) {
        _pulseDeltaUs = now - _lastPulseTimeUs;
        if (_pulseDeltaUs > 0) {
            _flowRate = (1.0 / (double)_pulseDeltaUs) * 1e6f * 60.0f * ML_PER_PULSE;
        }
    }
    _lastPulseTimeUs = now;
}

void FlowSensor::loopTask(void *arg) {
    auto *sensor = static_cast<FlowSensor *>(arg);
    while (true) {
        sensor->loop();
        vTaskDelay(FLOW_LOOP_INTERVAL_MS / portTICK_PERIOD_MS);
    }
}
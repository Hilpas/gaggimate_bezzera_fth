#include "FlowSensor.h"
#include <esp_timer.h>

FlowSensor::FlowSensor(uint8_t pin, const flow_callback_t &callback)
    : _pin(pin), _callback(callback) {}

float FlowSensor::read() { return _filteredFlowRate; }
float FlowSensor::read_g_s() { return _filteredFlowRate / 60.0f; }

void FlowSensor::setup() {
    // Create a queue to store pulse timestamps from the ISR
    _pulseQueue = xQueueCreate(PULSE_QUEUE_LENGTH, sizeof(uint64_t));
    if (_pulseQueue == NULL) {
        ESP_LOGE(LOG_TAG, "Failed to create pulse queue!");
        return; // Don't proceed if queue creation fails
    }

    pinMode(_pin, INPUT_PULLUP);
    attachInterruptArg(_pin, &FlowSensor::isrHandlerStatic, this, RISING);
    ESP_LOGI(LOG_TAG, "Initializing flow sensor on pin: %d", _pin);
    xTaskCreate(loopTask, "FlowSensor::loop", configMINIMAL_STACK_SIZE * 4, this, 1, &taskHandle);
}

void FlowSensor::loop() {
    uint64_t pulseTimestampUs;
    bool newPulseProcessed = false;
    
    // Drain the queue of all pending pulses
    while (xQueueReceive(_pulseQueue, &pulseTimestampUs, 0) == pdPASS) {
        newPulseProcessed = true; // Flag that we processed at least one pulse
        if (_lastPulseTimeUs != 0) {
            uint64_t pulseDeltaUs = pulseTimestampUs - _lastPulseTimeUs;
            if (pulseDeltaUs > 0) {
                // 1. Calculate the raw, instantaneous flow rate
                _flowRate = (1000000.0f / pulseDeltaUs) * 60.0f * ML_PER_PULSE;

                // --- ADDED FILTER LOGIC ---
                // 2. Apply the exponential moving average filter
                // This gives us a much smoother, more stable reading.
                _filteredFlowRate = (ALPHA * _flowRate) + (1.0f - ALPHA) * _filteredFlowRate;
                // --- END FILTER LOGIC ---
            }
        } else {
            // Handle the very first pulse to initialize the filter
            _flowRate = 0; // Can't calculate a rate with only one pulse
            _filteredFlowRate = 0; // So the filtered rate is also zero
        }
        _lastPulseTimeUs = pulseTimestampUs;
    }

    // Zero the flow rate if no new pulses have arrived for a while
    uint64_t nowUs = esp_timer_get_time();
    if ((nowUs - _lastPulseTimeUs) > 1000000) {
        if (_flowRate != 0.0f) {
            ESP_LOGV(LOG_TAG, "Flow rate zeroed due to timeout");
        }
        _flowRate = 0.0f;
        _filteredFlowRate = 0.0f; // Also zero the filtered rate on timeout
    }

    // Only print the log message if at least 1 second (1,000,000 µs) has passed
    if (nowUs - _lastLogTimeUs > 1000000) {
        ESP_LOGV(LOG_TAG, "Raw Rate: %.2f ml/min, Filtered Rate: %.2f ml/min", _flowRate, _filteredFlowRate);
        _lastLogTimeUs = nowUs; // Update the last log time
    }
    
    // Optional: You might want the callback to send the filtered value now
    if(_callback) {
        _callback(_filteredFlowRate);
    }
}

void FlowSensor::isrHandlerStatic(void* arg) {
    static_cast<FlowSensor*>(arg)->isrHandler();
}

void FlowSensor::isrHandler() {
    // Get the timestamp and immediately send it to the queue.
    // This is the ONLY thing the ISR should do. It's extremely fast.
    uint64_t now = esp_timer_get_time();
    xQueueSendFromISR(_pulseQueue, &now, NULL);
}

void FlowSensor::loopTask(void *arg) {
    auto *sensor = static_cast<FlowSensor *>(arg);
    while (true) {
        sensor->loop();
        vTaskDelay(FLOW_LOOP_INTERVAL_MS / portTICK_PERIOD_MS);
    }
}
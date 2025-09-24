#include "NTCDifferentialTemperatureSensor.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include "../GaggiMateController.h" 
#include "PressureSensor.h" 

const char* NTCDifferentialTemperatureSensor::LOG_TAG = "NTCDiff";

NTCDifferentialTemperatureSensor::NTCDifferentialTemperatureSensor(uint8_t sda_pin, uint8_t scl_pin,
                                            const temperature_callback_t &callback, const temperature_error_callback_t &error_callback)
                                           {
    this->callback = callback;
    this->error_callback = error_callback;
}

float NTCDifferentialTemperatureSensor::read() { return _temperature; }

bool NTCDifferentialTemperatureSensor::isErrorState() { return _errors >= NTC_DIFF_MAX_ERRORS; }

void NTCDifferentialTemperatureSensor::setup() {
    xTaskCreate(monitorTask, "NTCTemperatureSensor::monitor", configMINIMAL_STACK_SIZE * 4, this, TASK_PRIO_SENSORS, &taskHandle);
}

float NTCDifferentialTemperatureSensor::getNTCResistance(float Vntc) {
  return (Vntc * R_SERIES) / (VCC - Vntc);
}

float NTCDifferentialTemperatureSensor::ntcToTemperature(float R) {
  float invT = (1.0 / T0) + (1.0 / BETA) * log(R / R25);
  return (1.0 / invT) - 273.15; // in °C
}

void NTCDifferentialTemperatureSensor::loop() {
    // This function now only reads the sensor, assuming it's initialized.
    // The old logic is moved to monitorTask.

    int16_t adcRaw = _ads.readADC_Differential_0_1();
    float voltsDiff = adcRaw * 0.0000625; // [V]

    float Vref = VCC * (R_REF_BOT / (R_REF_TOP + R_REF_BOT));
    float Vntc = voltsDiff + Vref;
    float Rntc = getNTCResistance(Vntc);
    float tempC = ntcToTemperature(Rntc);

    _errors = 0;
    if (tempC > 0) {
        _temperature_filtered = _alpha * tempC + (1 - _alpha) * _temperature_filtered;
        _temperature = _temperature_filtered + _calib_offset;
    } else {
        _errors++;
    }

    if (_errors >= NTC_DIFF_MAX_ERRORS || _temperature > NTC_DIFF_MAX_SAFE_TEMP) {
        error_callback();
        return;
    }

    ESP_LOGV(LOG_TAG, "Updated temperature: %2f\n", _temperature);
    callback(_temperature);
}

[[noreturn]] void NTCDifferentialTemperatureSensor::monitorTask(void *arg) {
    auto *sensor = static_cast<NTCDifferentialTemperatureSensor *>(arg);
    
    while (true) {
        if (!sensor->_isInitialized) {
            // --- INITIALIZATION LOGIC ---
            // Try to begin communication with the ADS1115
            if (sensor->_ads.begin(0x49, PressureSensor::i2c_bus)) {
                sensor->_ads.setGain(GAIN_TWO); // Set gain after successful init
                sensor->_isInitialized = true;
                ESP_LOGV(LOG_TAG, "ADS1115 @0x49 found and configured.");
            } else {
                // If it fails, print an error and wait before retrying.
                ESP_LOGV(LOG_TAG, "ADS1115 @0x49 not found, retrying in 2s...");
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                continue; // Skip the rest of the loop and try to init again
            }
        }

        // --- NORMAL OPERATION ---
        // If initialized, run the sensor reading loop.
        sensor->loop();
        
        // Wait for the next update cycle.
        vTaskDelay(NTC_UPDATE_INTERVAL / portTICK_PERIOD_MS);
    }
}

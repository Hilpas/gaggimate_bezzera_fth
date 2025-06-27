#include "NTCTemperatureSensor.h"
#include <Arduino.h>
#include <freertos/FreeRTOS.h>

NTCTemperatureSensor::NTCTemperatureSensor(const int pin, const float seriesResistor, const float nominalResistance,
                                           const float nominalTempC, const float bCoefficient,
                                           const temperature_error_callback_t &error_callback, const temperature_callback_t &callback)
                                           : pin(pin), seriesResistor(seriesResistor), nominalResistance(nominalResistance),
                                             nominalTempC(nominalTempC), bCoefficient(bCoefficient) {
    this->callback = callback;
    this->error_callback = error_callback;
}

float NTCTemperatureSensor::read() { return temperature; }

bool NTCTemperatureSensor::hasError() { return errors >= NTC_MAX_ERRORS; }

void NTCTemperatureSensor::setup() {
    pinMode(pin, INPUT);

    xTaskCreate(monitorTask, "NTCTemperatureSensor::monitor", configMINIMAL_STACK_SIZE * 4, this, 1, &taskHandle);
}

void NTCTemperatureSensor::loop() {
    errors = max(0.0f, errors - 0.1f);
    int adc = analogRead(pin);
    float resistance = seriesResistor / ((4095.0 / adc) - 1.0);
    float temp = 0.0f;

    temp = resistance / nominalResistance;          // (R/Ro)
    temp = log(temp);                        // ln(R/Ro)
    temp /= bCoefficient;                           // 1/B * ln(R/Ro)
    temp += 1.0 / (nominalTempC + 273.15);          // + (1/To)
    temp = 1.0 / temp;                       // Invert
    temp -= 273.15;

    errors = 0;// convert to C
    if (temp > 0) {
        temperature = 0.2 * temp + 0.8 * temperature;
    } else {
        errors++;
    }

    if (errors >= NTC_MAX_ERRORS || temperature > NTC_MAX_SAFE_TEMP) {
        error_callback();
        return;
    }

    ESP_LOGV(LOG_TAG, "Updated temperature: %2f\n", temperature);
    callback(temperature);
}

[[noreturn]] void NTCTemperatureSensor::monitorTask(void *arg) {
    auto *sensor = static_cast<NTCTemperatureSensor *>(arg);
    while (true) {
        sensor->loop();
        vTaskDelay(NTC_UPDATE_INTERVAL / portTICK_PERIOD_MS);
    }
}

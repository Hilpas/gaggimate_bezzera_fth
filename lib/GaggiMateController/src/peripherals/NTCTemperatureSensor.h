#ifndef NTCTEMPERATURESENSOR_H
#define NTCTEMPERATURESENSOR_H

#include "TemperatureSensor.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <functional>

constexpr int NTC_UPDATE_INTERVAL = 250;
constexpr int NTC_MAX_ERRORS = 20;
constexpr double NTC_MAX_SAFE_TEMP = 170.0;

using temperature_callback_t = std::function<void(float)>;
using temperature_error_callback_t = std::function<void()>;

class NTCTemperatureSensor : public TemperatureSensor{
    public:
        NTCTemperatureSensor(int pin, float seriesResistor, float nominalResistance, float nominalTempC, float bCoefficient, 
            const temperature_error_callback_t &error_callback, const temperature_callback_t &callback);
        float read() override;
        bool hasError() override;

        void setup();
        void loop();

    private:
        xTaskHandle taskHandle;

        float errors = .0f;
        float temperature = .0f;

        int pin;
        float seriesResistor;
        float nominalResistance;
        float nominalTempC;
        float bCoefficient;

        temperature_callback_t callback;
        temperature_error_callback_t error_callback;
        
        const char *LOG_TAG = "NTCTemperatureSensor";
        static void monitorTask(void *arg);
    };

#endif // NTCTEMPERATURESENSOR_H
#ifndef NTCDIFFTEMPERATURESENSOR_H
#define NTCDIFFTEMPERATURESENSOR_H

#include "TemperatureSensor.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <functional>
#include <Adafruit_ADS1X15.h>

constexpr int NTC_DIFF_UPDATE_INTERVAL = 20; // 20ms same as flow
constexpr int NTC_DIFF_MAX_ERRORS = 20;
constexpr double NTC_DIFF_MAX_SAFE_TEMP = 170.0;

// === NTC / Schaltung Parameter ===
const float VCC = 3.3;                // Versorgungsspannung [V]
const float R25 = 50000.0;            // NTC Nennwiderstand @25°C [Ohm]
const float BETA = 3976.0;            // NTC B-Parameter [K]
const float T0 = 298.15;              // Referenztemperatur 25°C in Kelvin

// Deine gemessenen Widerstände hier eintragen:
const float R_SERIES = 4625.0;        // Serienwiderstand in Ohm (statt nominell 4k7)
const float R_REF_TOP = 46960.0;      // oberer Referenzwiderstand (VCC -> Ref)
const float R_REF_BOT = 46920.0;      // unterer Referenzwiderstand (Ref -> GND)

using temperature_callback_t = std::function<void(float)>;
using temperature_error_callback_t = std::function<void()>;

class NTCDifferentialTemperatureSensor : public TemperatureSensor{
    public:
        NTCDifferentialTemperatureSensor(uint8_t sda_pin, uint8_t scl_pin, const temperature_callback_t &callback, const temperature_error_callback_t &error_callback);
        float read() override;
        bool isErrorState() override;

        void setup();
        void loop();

        float getNTCResistance(float Vntc);
        float ntcToTemperature(float R);

    private:
        Adafruit_ADS1115 _ads;
        bool _isInitialized = false;
        uint8_t _sda_pin;
        uint8_t _scl_pin;

        xTaskHandle taskHandle;

        float _errors = .0f;
        float _temperature = .0f;
        float _temperature_filtered = .0f;
        float _alpha = 0.3f;
        float _calib_offset = -0.846f; // Einpunktkalibrierung bei Raumtemperatur (24.7°C gemessen, 25.54°C berechnet)

        temperature_callback_t callback;
        temperature_error_callback_t error_callback;

        static const char *LOG_TAG;
        static void monitorTask(void *arg);
    };

#endif // NTCDIFFTEMPERATURESENSOR_H
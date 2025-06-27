#ifndef NTCTEMPERATURESENSOR_H
#define NTCTEMPERATURESENSOR_H
#include <Arduino.h>

class NTCThermistor {
public:
    NTCThermistor(uint8_t pin, float seriesResistor, float nominalResistance, float nominalTempC, float bCoefficient)
        : _pin(pin), _seriesResistor(seriesResistor), _nominalResistance(nominalResistance),
          _nominalTempC(nominalTempC), _bCoefficient(bCoefficient) {}

    float readTemperatureC() {
        int adc = analogRead(_pin);
        float resistance = _seriesResistor / ((4095.0 / adc) - 1.0);
        float steinhart;
        steinhart = resistance / _nominalResistance;          // (R/Ro)
        steinhart = log(steinhart);                           // ln(R/Ro)
        steinhart /= _bCoefficient;                           // 1/B * ln(R/Ro)
        steinhart += 1.0 / (_nominalTempC + 273.15);          // + (1/To)
        steinhart = 1.0 / steinhart;                          // Invert
        steinhart -= 273.15;                                  // convert to C
        return steinhart;
    }

    void setup() {
        pinMode(_pin, INPUT);
    }

private:
    uint8_t _pin;
    float _seriesResistor, _nominalResistance, _nominalTempC, _bCoefficient;
};

#endif // NTCTEMPERATURESENSOR_H
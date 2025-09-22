#ifndef PSM_H
#define PSM_H

#include <Arduino.h>

class PSM {
public:
    // --- NEU: Erweiterter Konstruktor, der dem alten entspricht ---
    PSM(uint8_t zeroCrossPin, uint8_t triacPin, uint16_t range = 1000, int mode = FALLING, uint8_t divider = 1, uint8_t interruptMinTimeDiff = 0);

    void begin();

    // --- NEU: set() anstelle von setPower(), um der alten Lib zu entsprechen ---
    void set(uint16_t value);

    // --- NEU: Alle zusätzlichen Funktionen aus der alten Bibliothek ---
    long getCounter();
    void resetCounter();
    void stopAfter(long counter);
    uint8_t getDivider();
    void setDivider(uint8_t divider);

private:
    static void IRAM_ATTR onZeroCrossISR(void* arg);
    void IRAM_ATTR handleZeroCross();

    // Pins
    const uint8_t m_zeroCrossPin;
    const uint8_t m_triacPin;
    const int m_interruptMode;

    // --- NEU: Alle Zustandsvariablen aus der alten Bibliothek ---
    uint16_t m_range;
    volatile uint16_t m_value = 0;
    volatile uint16_t m_accumulator = 0;
    volatile bool m_skipPulse = true;

    // Divider-Logik
    uint8_t m_divider = 1;
    volatile uint8_t m_dividerCounter = 0;

    // Counter-Logik
    volatile long m_counter = 0;
    volatile long m_stopAfter = 0;

    // Sicherer Ersatz für millis() im Interrupt
    uint32_t m_interruptMinTimeDiffCycles;
    volatile uint32_t m_lastCycleCount = 0;
};

#endif // PSM_H
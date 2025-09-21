#ifndef PSM_H
#define PSM_H

#include <Arduino.h>

class PSM {
public:
    PSM(uint8_t zeroCrossPin, uint8_t triacPin);
    void begin();
    void setPower(float level);

private:
    // --- FIX: Change ISR to accept an argument ---
    // The ISR will now be passed a pointer to the PSM object instance.
    static void IRAM_ATTR onZeroCrossISR(void* arg);

    void IRAM_ATTR handleZeroCross();

    // Pins
    const uint8_t m_zeroCrossPin;
    const uint8_t m_triacPin;

    // Dimming Algorithm State
    volatile uint16_t m_dimmingValue = 0;
    const uint16_t m_dimmingRange = 1000;
    volatile uint16_t m_accumulator = 0;
    volatile bool m_skipPulse = true;

    // The static pointer is no longer needed.
    // static PSM* s_instance; 
};


#endif // PSM_H
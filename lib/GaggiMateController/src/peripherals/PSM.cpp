#include "PSM.h"

// The static pointer is no longer needed.
// PSM* PSM::s_instance = nullptr;

PSM::PSM(uint8_t zeroCrossPin, uint8_t triacPin)
    : m_zeroCrossPin(zeroCrossPin), m_triacPin(triacPin) {
    // The constructor no longer needs to set the static pointer.
}

void PSM::begin() {
    pinMode(m_zeroCrossPin, INPUT_PULLUP);
    pinMode(m_triacPin, OUTPUT);
    digitalWrite(m_triacPin, LOW);

    // --- FIX: Use attachInterruptArg ---
    // This function allows us to pass a pointer to 'this' object as the argument,
    // and it accepts the ESP_INTR_FLAG_IRAM flag.
    attachInterruptArg(digitalPinToInterrupt(m_zeroCrossPin), onZeroCrossISR, this, FALLING);
}

void PSM::setPower(float level) {
    // Clamp the input level to a safe range of 0.0 to 100.0
    if (level < 0.0f) level = 0.0f;
    if (level > 100.0f) level = 100.0f;

    // Convert the 0-100 float to a 0-1000 integer for our dimming algorithm.
    m_dimmingValue = static_cast<uint16_t>(level * 10.0f);
}

// --- FIX: Update ISR to handle the argument ---
// The 'arg' parameter will be the 'this' pointer we passed in begin().
void IRAM_ATTR PSM::onZeroCrossISR(void* arg) {
    // Cast the void pointer back to a PSM object pointer and call the handler.
    PSM* instance = static_cast<PSM*>(arg);
    instance->handleZeroCross();
}

// The actual interrupt handler logic remains the same.
void IRAM_ATTR PSM::handleZeroCross() {
    // This is the core of the dimming algorithm (a Digital Differential Analyzer).
    // It decides if we should skip this AC pulse to achieve a fractional power level.
    m_accumulator += m_dimmingValue;
    if (m_accumulator >= m_dimmingRange) {
        m_accumulator -= m_dimmingRange;
        m_skipPulse = false; // Don't skip this pulse
    } else {
        m_skipPulse = true;  // Skip this pulse
    }

    // If we are not skipping this pulse, fire the TRIAC.
    if (!m_skipPulse) {
        // A small delay to ensure we are past the zero-cross point.
        // delayMicroseconds() is one of the few delay functions safe to use in an ISR.
        delayMicroseconds(10);

        // Use fast, direct GPIO writes instead of the slow digitalWrite().
        gpio_set_level((gpio_num_t)m_triacPin, 1);
        delayMicroseconds(10); // Keep the TRIAC gate high for a moment to ensure it latches.
        gpio_set_level((gpio_num_t)m_triacPin, 0);
    }
}
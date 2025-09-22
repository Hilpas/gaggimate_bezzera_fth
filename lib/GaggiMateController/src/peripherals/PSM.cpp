#include "PSM.h"

PSM::PSM(uint8_t zeroCrossPin, uint8_t triacPin, uint16_t range, int mode, uint8_t divider, uint8_t interruptMinTimeDiff)
    : m_zeroCrossPin(zeroCrossPin), 
      m_triacPin(triacPin), 
      m_interruptMode(mode),
      m_range(range > 0 ? range : 1) 
{
    setDivider(divider);
    m_interruptMinTimeDiffCycles = interruptMinTimeDiff * 240000; // 240MHz CPU clock
}

void PSM::begin() {
    pinMode(m_zeroCrossPin, INPUT_PULLUP);
    pinMode(m_triacPin, OUTPUT);
    digitalWrite(m_triacPin, LOW); // Start with pump off

    attachInterruptArg(digitalPinToInterrupt(m_zeroCrossPin), onZeroCrossISR, this, m_interruptMode);
}

void PSM::set(uint16_t value) {
    if (value <= m_range) {
        m_value = value;
    } else {
        m_value = m_range;
    }
}

long PSM::getCounter() {
    return m_counter;
}

void PSM::resetCounter() {
    m_counter = 0;
}

void PSM::stopAfter(long counter) {
    m_stopAfter = counter;
}

uint8_t PSM::getDivider() {
    return m_divider;
}

void PSM::setDivider(uint8_t divider) {
    m_divider = divider > 0 ? divider : 1;
}

void IRAM_ATTR PSM::onZeroCrossISR(void* arg) {
    static_cast<PSM*>(arg)->handleZeroCross();
}

void IRAM_ATTR PSM::handleZeroCross() {
    // --- 1:1 NACHBILDUNG DER ALTEN LOGIK ---

    // Debounce-Logik (Sicherheitsfeature beibehalten)
    if (m_interruptMinTimeDiffCycles > 0) {
        uint32_t now = xthal_get_ccount();
        if ((now - m_lastCycleCount) < m_interruptMinTimeDiffCycles) {
            return;
        }
        m_lastCycleCount = now;
    }

    // Divider-Logik
    if (m_dividerCounter >= m_divider - 1) {
        m_dividerCounter = 0;
        
        // --- calculateSkip()-Logik ---
        m_accumulator += m_value;
        if (m_accumulator >= m_range) {
            m_accumulator -= m_range;
            m_skipPulse = false;
        } else {
            m_skipPulse = true;
        }

        // Seltsame Bugfix-Logik aus der alten Lib, für 100% Kompatibilität übernommen
        if (m_accumulator > m_range) {
            m_accumulator = 0;
            m_skipPulse = false;
        }

        // Counter- und stopAfter-Logik
        if (!m_skipPulse) {
            m_counter++;
            if (m_stopAfter > 0 && m_counter > m_stopAfter) {
                m_skipPulse = true;
            }
        }
    } else {
        m_dividerCounter++;
        // WICHTIG: Im `else`-Fall wird `m_skipPulse` NICHT geändert, genau wie in der alten Lib.
        // Der Zustand vom letzten Zyklus bleibt erhalten.
    }

    // --- updateControl()-Logik ---
    // Dies bildet das Verhalten exakt nach: Pin bleibt HIGH, wenn nicht geskippt wird.
    if (m_skipPulse) {
        gpio_set_level((gpio_num_t)m_triacPin, 0); // LOW
    } else {
        gpio_set_level((gpio_num_t)m_triacPin, 1); // HIGH
    }
}
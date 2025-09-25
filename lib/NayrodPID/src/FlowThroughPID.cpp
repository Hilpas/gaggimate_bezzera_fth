#include "FlowThroughPID.h"
#include <Arduino.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <numeric>

// based on SimpePID modified for flow through heater control
FlowThroughPID::FlowThroughPID(float *controlerOutputPtr, float *sensorTemperatureOutputPtr, float *sensorFlowOutputPtr, float *setpointTargetPtr) {
    this->controlerOutput = controlerOutputPtr;
    this->sensorTemperatureOutput = sensorTemperatureOutputPtr;
    this->sensorFlowOutput = sensorFlowOutputPtr;
    this->setpointTarget = setpointTargetPtr;
}

bool FlowThroughPID::update() {
    if (mode == Control::manual) {
        return false;
    }
    uint32_t now = millis();
    uint32_t timeChange = (now - lastTime);
    // CORRECTED LOGIC: This correctly uses the variable as a frequency in Hz
    if (timeChange < (1000 / ctrl_freq_sampling)) {
        return false;
    }
    lastTime = now;

    if (!isInitialized) {
        resetFeedbackController();
        if (gainFF != 0.0f)
            isFeedForwardActive = true; // Activate the feedforward control if gainFF is not zero
        isInitialized = true;
    }

    // Use the ACTUAL elapsed time for calculations, not a theoretical one.
    float deltaTimeInSeconds = timeChange / 1000.0f;

    // Feeback terms
    float error = *setpointTarget - *sensorTemperatureOutput;

    float FFoutput = 0.0f;
    if (isFeedForwardActive && sensorFlowOutput != nullptr && *sensorFlowOutput > 0.0f) 
    {
        // ml/min * J/(g°C) * (°C) / 60s = W
        FFoutput = (*sensorFlowOutput * SPECIFIC_HEAT_WATER * (*setpointTarget - WATER_INLET_TEMP)) / 60.0f; // in Watt

        // Scale to output range (e.g., if ctrlOutputLimits are in percent of max power)
        // Example: if FFoutput is in Watt and ctrlOutputLimits are in percent:
        FFoutput = (FFoutput / HEATER_MAX_POWER) * ctrlOutputLimits[1]; // Scale to the output range
        FFoutput *= gainFF; // add gain factor

        // Limit/clamp to allowed range
        FFoutput = fmaxf(ctrlOutputLimits[0], fminf(FFoutput, ctrlOutputLimits[1]));
    }
    else {
        FFoutput = 0.0f; // No feedforward if not active or sensorFlowOutput is zero
    }

    float Pout = gainKp * error;

    feedback_integralState += error * deltaTimeInSeconds;
    float Iout = gainKi * feedback_integralState;

    float derivative = (error - prevError) / deltaTimeInSeconds;
    float Dout = gainKd * derivative;

    // Calculate the output before antiwindup clamping
    float sumPID = Pout + Iout + Dout + FFoutput;
    float sumPIDsat = constrain(sumPID, ctrlOutputLimits[0], ctrlOutputLimits[1]);

    prevError = error;
    prevOutput = sumPIDsat;

    *controlerOutput = sumPIDsat;

    return true;
}

void FlowThroughPID::resetFeedbackController() {
    feedback_integralState = 0.0f; // Reset the integral state
    prevError = 0.0f;              // Reset the previous error for derivative calculation
    prevOutput = 0.0f;             // Reset the previous output for derivative calculation
}

void FlowThroughPID::reset() {
    resetFeedbackController();
    isInitialized = false;
}

// GETTER-SETTER FUNCTIONS
// Feedback controller
void FlowThroughPID::setControllerPIDGains(float Kp, float Ki, float Kd, float Kf) {
    this->gainKp = Kp;
    this->gainKi = Ki;
    this->gainKd = Kd;
    this->gainFF = Kf;
}

void FlowThroughPID::setSamplingFrequency(float freq) { ctrl_freq_sampling = freq; }
void FlowThroughPID::setCtrlOutputLimits(float minOutput, float maxOutput) {
    ctrlOutputLimits[0] = minOutput;
    ctrlOutputLimits[1] = maxOutput;
}

void FlowThroughPID::setMode(Control modeCMD) {
    if (modeCMD == Control::automatic && this->mode == Control::manual) {
        reset();
    }
    this->mode = modeCMD;
}

void FlowThroughPID::setManualOutput(float output) {
    if (this->mode == Control::automatic)
        setMode(Control::manual);
    manualOutput = output;
}

void FlowThroughPID::activateFeedForward(bool flag) {
    isFeedForwardActive = flag;
}

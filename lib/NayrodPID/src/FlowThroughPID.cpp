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
    if (timeChange < ctrl_freq_sampling * 1000) {
        return false;
    }
    lastTime = now;

    if (!isInitialized) {
        resetFeedbackController();
        if (gainFF != 0.0f)
            isFeedForwardActive = true; // Activate the feedforward control if gainFF is not zero
        isInitialized = true;
    }

    float deltaTime = 1.0f / ctrl_freq_sampling; // Time step in seconds

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

        // Limit/clamp to allowed range
        FFoutput = fmaxf(ctrlOutputLimits[0], fminf(FFoutput, ctrlOutputLimits[1]));
    }
    else {
        FFoutput = 0.0f; // No feedforward if not active or sensorFlowOutput is zero
    }

    float Pout = gainKp * error;

    feedback_integralState += error * deltaTime;
    float Iout = gainKi * feedback_integralState;

    float derivative = (error - prevError) / deltaTime;
    float Dout = gainKd * derivative;

    // Calculate the output before antiwindup clamping
    float sumPID = Pout + Iout + Dout + FFoutput;
    float sumPIDsat = constrain(sumPID, ctrlOutputLimits[0], ctrlOutputLimits[1]);

    Serial.printf("FeedForward: %.2f, PID: %.2f\n", FFoutput, sumPIDsat);

    // Antiwindup clamping
    bool isSaturated = (sumPID < ctrlOutputLimits[0] || sumPID > ctrlOutputLimits[1]); // Check if the output is saturated
    bool isSameSign =
        ((error > 0 && sumPID > 0) || (error < 0 && sumPID < 0)); // Check if the error and output have the same sign
    // Serial.printf("OutputPID: %.2f, Integ out: %.2f\n", sumPIDsat, Iout);
    if (isSaturated && isSameSign) {
        // Serial.printf("Antiwindup clamping: %.2f\n", feedback_integralState);
        feedback_integralState -=
            error * deltaTime; // Forbide the integration to happen when the output is saturated and the error is in the same
                               // direction as the output (i.e. the system is not able to follow the setpoint)
        Iout = gainKi * feedback_integralState; // Recompute the integral term with the new state
        sumPID = Pout + Iout + Dout + FFoutput;    // Recompute the output with the new integral state
        sumPIDsat = constrain(sumPID, ctrlOutputLimits[0], ctrlOutputLimits[1]);
    }

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
void FlowThroughPID::setControllerPIDGains(float Kp, float Ki, float Kd, float FF) {
    this->gainKp = Kp;
    this->gainKi = Ki;
    this->gainFF = FF;
    this->gainKd = Kd;
}

void FlowThroughPID::setSamplingFrequency(float freq) { ctrl_freq_sampling = freq; }
void FlowThroughPID::setCtrlOutputLimits(float minOutput, float maxOutput) {
    ctrlOutputLimits[0] = minOutput;
    ctrlOutputLimits[1] = maxOutput;
}

void FlowThroughPID::setMode(Control modeCMD) {
    if (modeCMD == Control::automatic && this->mode == Control::manual) {
        isInitialized = false; // Reset the controller when switching to automatic mode
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

#include "FlowController.h"
#include <math.h>
#include <algorithm>
#include <cstdio>

FlowController::FlowController(float dt, float *rawSetpoint, float *sensorOutput, float *controllerOutput,
                                       int *ValveStatus) {
    this->_rawSetpoint = rawSetpoint;
    this->_rawFlow = sensorOutput;
    this->_ctrlOutput = controllerOutput;
    this->_ValveStatus = ValveStatus;
    this->_dt = dt;
}

void FlowController::update() {
    computePumpDutyCycle();
}

void FlowController::setTunings(float Kp, float Ki, float Kd) {
    _Kp = Kp;
    _Ki = Ki;
    _Kd = Kd;    
}

void FlowController::computePumpDutyCycle() {

    if (*_rawSetpoint == 0.0f) {
        *_ctrlOutput = 0.0f;
        // Reset PID state when pump is off
        this->_integral = 0.0f;
        this->_lastError = 0.0f;
        return;
    }

    // Proportional term
    float error = *_rawSetpoint - *_rawFlow;
    float proportionalTerm = _Kp * error;

    // Integral term
    _integral += error * _dt;
    float integralTerm = _Ki * _integral;

    // Derivative term
    float derivative = (error - _lastError) / _dt;
    float derivativeTerm = _Kd * derivative;

    // Combine terms to get the new output
    float output = proportionalTerm + integralTerm + derivativeTerm;

    // Clamp output to 0..100%
    output = std::clamp(output, 0.0f, 100.0f);

    *_ctrlOutput = output;

    // Update last error for the next iteration
    _lastError = error;

    //printf("Setpoint:%.1f%%, Sensor:%.1f%%, Output:%.1f%%\n", *_rawSetpoint, *_rawFlow, *_ctrlOutput);

}

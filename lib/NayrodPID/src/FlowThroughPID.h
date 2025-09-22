#ifndef FLOW_THROUGH_PID_H
#define FLOW_THROUGH_PID_H
#include <cmath>
#include <deque>
#include <vector>
// #define PI 3.14159265358979323846
#define SPECIFIC_HEAT_WATER 4.186f // J/(g°C)
#define HEATER_MAX_POWER 1500.0f // Maximum heater power in Watts
#define WATER_INLET_TEMP 20.0f // Assumed water inlet temperature in °C

// based on SimpePID modified for flow through heater control
class FlowThroughPID {
  public:
    FlowThroughPID(float *controlerOutput = nullptr, float *sensorTemperatureOutput = nullptr, float *sensorFlowOutput = nullptr, float *setpointTargetPtr = nullptr);
    bool update();
    void setControllerPIDGains(float Kp, float Ki, float Kd);
    void resetFeedbackController();
    void setSamplingFrequency(float freq);
    void setCtrlOutputLimits(float minOutput, float maxOutput);

    void activateSetPointFilter(bool flag);

    void reset();

    void setManualOutput(float output = 0.0f);
    void activateFeedForward(bool flag);

    enum class Control : uint8_t { manual, automatic }; // controller mode
    void setMode(Control mode);

    float getCtrlSamplingFrequency() { return ctrl_freq_sampling; };
    float getKp() { return gainKp; };
    float getKi() { return gainKi; };
    float getKd() { return gainKd; };
    float getKFF() { return gainFF; };
    float getSetpointValue() const { return *setpointTarget; };
    float getInputValue() const { return *sensorTemperatureOutput; };

    void setKp(float val) { gainKp = val; };
    void setKi(float val) { gainKi = val; };
    void setKd(float val) { gainKd = val; };
    void setKFF(float val) { gainFF = val; };

  private:
    // feed forward control
    bool isFeedForwardActive = true;                  // Flag to activate/deactivate the feedforward control

    // feedback controler
    float ctrlOutputLimits[2] = {-INFINITY, INFINITY}; // Control output limits {lower, upper}
    float ctrl_freq_sampling = 0.1f;                   // Control frequency (Hz)
    bool isInitialized = false;                           // Flag to check if the controller is initialized
    float gainKp = 0.0f;                                 // Proportional gain
    float gainKi = 0.0f;                                  // Integral gain (multiplies by Kp if Kp,Ki,Kd are strictly parallèle (no factoring by Kp))
    float gainKd = 0.0f;                                  // Derivative gain (by default no derivative term)
    float gainFF = 1.2f;                                 // Feedforward gain calibrated by hand
    float feedback_integralState = 0.0f;                  // Integral state
    float prevError = 0.0f;                             // Previous error for derivative calculation
    float prevOutput = 0.0f;                            // Previous output for derivative calculation
    Control mode = Control::manual;
    float manualOutput = 0.0f;
    unsigned long lastTime = 0;

    float *controlerOutput = nullptr; // Pointer to the control output variable
    float *sensorTemperatureOutput = nullptr;    // Pointer to the temperature sensor output variable
    float *sensorFlowOutput = nullptr; // Pointer to the flow sensor output variable
    float *setpointTarget = nullptr;  // System current target setpoint;
};

#endif
//

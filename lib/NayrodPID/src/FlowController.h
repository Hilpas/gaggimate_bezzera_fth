// FlowController.h
#ifndef FLOW_CONTROLLER_H
#define FLOW_CONTROLLER_H

class FlowController {
  public:
    FlowController(float dt, float *rawSetpoint, float *sensorOutput, float *controllerOutput, int *valveStatus);

    void update();
    void setTunings(float Kp, float Ki, float Kd);
    void filterSensor();
    void computePumpDutyCycle();
    float getFlowPerSecond() { return _flowPerSecond; };

  private:
    float _dt = 1; // Controler frequency sampling

    float *_rawSetpoint = nullptr;  // pointer to the Pressure profile current setpoint
    float *_rawFlow = nullptr;      // pointer to the flow measurement ,raw output from sensor
    float *_ctrlOutput = nullptr;   // pointer to controller output value of power ratio 0-100%
    int *_ValveStatus = nullptr;    // pointer to valve status regarding group head canal open/closed
    int old_ValveStatus = 0;

    // === Parameters Controller ===
    float _Kp = 5.0f;         
    float _Ki = 20.0f;
    float _Kd = 2.0f;    

    float _flowPerSecond = 0.0f;

     // PID state variables
    float _integral = 0.0f; // Stores the accumulated error
    float _lastError = 0.0f; // Stores the error from the previous time step

};

#endif // FLOW_CONTROLLER_H

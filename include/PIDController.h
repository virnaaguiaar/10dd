#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PIDController {
private:
  float kp, ki, kd;
  float integral, prevError;
  unsigned long lastTime;
  float outputMin, outputMax;
  float target;
  bool enabled;
  
public:
  PIDController(float p = 1.0f, float i = 0.0f, float d = 0.0f, 
                float minOut = -9.0f, float maxOut = 9.0f) 
    : kp(p), ki(i), kd(d), integral(0), prevError(0),
      outputMin(minOut), outputMax(maxOut), target(0), enabled(false) {
    lastTime = millis();
  }
  
  void setGains(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
  }
  
  void setOutputLimits(float minOut, float maxOut) {
    outputMin = minOut;
    outputMax = maxOut;
  }
  
  void setTarget(float t) {
    target = t;
    enabled = true;
  }
  
  void enable() { enabled = true; reset(); }
  void disable() { enabled = false; reset(); }
  bool isEnabled() const { return enabled; }
  
  void reset() {
    integral = 0;
    prevError = 0;
    lastTime = millis();
  }
  
  float update(float measured) {
    if (!enabled) return 0;
    
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0;
    if (dt <= 0) dt = 0.01;
    lastTime = now;
    
    float error = target - measured;
    integral += error * dt;
    
    // Anti-windup
    integral = constrain(integral, -10.0f, 10.0f);
    
    float derivative = (error - prevError) / dt;
    prevError = error;
    
    float output = kp * error + ki * integral + kd * derivative;
    return constrain(output, outputMin, outputMax);
  }
  
  float update(float setpoint, float measured) {
    target = setpoint;
    return update(measured);
  }
  
  float getError() const { return target - prevError; }
  float getIntegral() const { return integral; }
  float getTarget() const { return target; }
};

#endif
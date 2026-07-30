#ifndef ACS_CONTROLLER_HPP
#define ACS_CONTROLLER_HPP

#include <Arduino.h>
#include <ESP32Servo.h>

class ACSController {
private:
    bool initialized;
    Servo rollMotor;
    Servo pitchMotor;

public:
    ACSController();
    void begin(int rollPin, int pitchPin,int minPulse = 1000,int maxPulse =2000);
    void setRollOutput(int pulseWidth);
    void setPitchOutput(int pulseWidth);

    void stop();
};

#endif

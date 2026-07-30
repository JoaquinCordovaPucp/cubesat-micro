#ifndef EJECTION_SYSTEM_HPP
#define EJECTION_SYSTEM_HPP

#include <Arduino.h>

class EjectionSystem {
private:
    int motorPin;
    bool active;

public:
    EjectionSystem();

    void begin(int pin);
    void activate();
    void deactivate();

    bool isActive();
};

#endif

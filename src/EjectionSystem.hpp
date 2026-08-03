#ifndef EJECTION_SYSTEM_HPP
#define EJECTION_SYSTEM_HPP

#include <Arduino.h>

class EjectionSystem {
private:
    int motorPin;
    bool active;

    unsigned long activationStartMillis;
    unsigned long activationDurationMillis;

public:
    EjectionSystem();

    void begin(int pin);

    void activate(
        unsigned long durationMillis
    );

    void update();
    void deactivate();

    bool isActive();
};

#endif

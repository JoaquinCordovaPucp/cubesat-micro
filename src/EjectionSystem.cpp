#include "EjectionSystem.hpp"

EjectionSystem::EjectionSystem() {
    motorPin = -1;
    active = false;

    activationStartMillis = 0;
    activationDurationMillis = 0;
}

void EjectionSystem::begin(int pin) {
    motorPin = pin;

    pinMode(motorPin, OUTPUT);

    deactivate();
}

void EjectionSystem::activate(
    unsigned long durationMillis
) {
    digitalWrite(motorPin, HIGH);

    active = true;

    activationStartMillis = millis();
    activationDurationMillis =
        durationMillis;
}

void EjectionSystem::update() {
    if (active == false) {
        return;
    }

    if (
        millis() -
        activationStartMillis >=
        activationDurationMillis
    ) {
        deactivate();
    }
}

void EjectionSystem::deactivate() {
    if (motorPin >= 0) {
        digitalWrite(
            motorPin,
            LOW
        );
    }

    active = false;
}

bool EjectionSystem::isActive() {
    return active;
}
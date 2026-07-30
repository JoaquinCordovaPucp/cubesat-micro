#include "EjectionSystem.hpp"

EjectionSystem::EjectionSystem() {
    motorPin = -1;
    active = false;
}

void EjectionSystem::begin(int pin) {
    motorPin = pin;

    pinMode(motorPin, OUTPUT);

    deactivate();
}

void EjectionSystem::activate() {
    digitalWrite(motorPin, HIGH);

    active = true;
}

void EjectionSystem::deactivate() {
    digitalWrite(motorPin, LOW);

    active = false;
}

bool EjectionSystem::isActive() {
    return active;
}

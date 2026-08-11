#include "ACSController.hpp"

ACSController::ACSController() {
    initialized = false;
}

void ACSController::begin(int rollPin, int pitchPin, int minPulse, int maxPulse) {
    rollMotor.setPeriodHertz(50);
    rollMotor.attach(rollPin, minPulse, maxPulse);

    pitchMotor.setPeriodHertz(50);
    pitchMotor.attach(pitchPin, minPulse, maxPulse);

    initialized = true;

    stop();
}

void ACSController::setRollOutput(int microseconds) {
    if (initialized == false) {
        return;
    }

    rollMotor.writeMicroseconds(microseconds);
}

void ACSController::setPitchOutput(int microseconds) {
    if (initialized == false) {
        return;
    }

    pitchMotor.writeMicroseconds(microseconds);
}

void ACSController::stop() {
    if (initialized == false) {
        return;
    }

    setRollOutput(1000);
    setPitchOutput(1000);
}

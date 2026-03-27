#include "acs.hpp"

ACSController::ACSController() : minPulse(1000), maxPulse(2000), neutralPulse(1500), initialized(false) {} // Incializa los valores de pin e initialized, el pin 255 es un valor invalido, para indicar que no se ha inicializado aun.

void ACSController::begin(uint8_t rollPinNum, uint8_t pitchPinNum, int minUs, int maxUs) {
	rollEsc.setPeriodHertz(50);
	rollEsc.attach(rollPinNum, minUs, maxUs);
	pitchEsc.setPeriodHertz(50);
	pitchEsc.attach(pitchPinNum, minUs, maxUs);
	initialized = true;
    stop();
}

void ACSController::stop() {
    if (!initialized) {
        return;
    }
    setRollOutput(neutralPulse);
	setPitchOutput(neutralPulse);
}


void ACSController::setRollOutput(int microseconds) {
	if (!initialized) {
		return;
	}
	rollEsc.writeMicroseconds(microseconds);
}

void ACSController::setPitchOutput(int microseconds) {
	if (!initialized) {
		return;
	}
	pitchEsc.writeMicroseconds(microseconds);
}
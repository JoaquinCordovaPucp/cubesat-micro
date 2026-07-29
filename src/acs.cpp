#include "acs.hpp"

ACSController::ACSController() : initialized(false) {}

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
	setRollOutput(1000);
	setPitchOutput(1000);
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
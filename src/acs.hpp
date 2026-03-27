#ifndef ACS_HPP
#define ACS_HPP

#include <Arduino.h>
#include <ESP32Servo.h>

class ACSController {
public:
	ACSController();
	// Basic setup for one actuator channel.
	void begin(uint8_t rollPinNum, uint8_t pitchPinNum, int minUs = 1000, int maxUs = 2000);
	// Motor output command in microseconds [minUs..maxUs].
	void setRollOutput(int microseconds);
	void setPitchOutput(int microseconds);
    void stop(); // Detiene los reaction wheels, enviando el pulso neutro (1500 microsegundos)

private:
	Servo rollEsc;
	Servo pitchEsc;
	bool initialized;
    int minPulse;
    int maxPulse;
    int neutralPulse;
};





#endif // ACS_HPP
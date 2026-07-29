#ifndef ACS_HPP
#define ACS_HPP

#include <Arduino.h>
#include <ESP32Servo.h>

class ACSController {
public:
	ACSController();
	void begin(uint8_t rollPinNum, uint8_t pitchPinNum, int minUs = 1000, int maxUs = 2000);
	void setRollOutput(int microseconds);
	void setPitchOutput(int microseconds);
	void stop();

private:
	bool initialized;
	Servo rollEsc;
	Servo pitchEsc;
};





#endif // ACS_HPP
/*
Motor class for driving a motor fwd, back and stopping with braking
Uses PWM on one pin with with two digital pins for controlling
direction, ie L298N motor driver style
Sets pin modes, takes esp8266 pins as eg D6, hence char declaration
*/
#include "Arduino.h"
#ifndef Motor_h
#define Motor_h

class Motor {
	public:
		Motor(char enablePin, char motorPin1, char motorPin2, int speed);
		void m_fwd();
		void m_back();
		void m_stop();
	private:
		char _enablePin;
		char _motorPin1;
		char _motorPin2;
		int _speed;
};

#endif

/*
Motor class for driving a motor fwd, back and stopping with braking
Uses PWM on one pin with with two digital pins for controlling
direction, ie L298N motor driver style
Sets pin modes, takes esp8266 pins as eg D6, hence char declaration
*/
#include "Arduino.h"
#include "Motor.h"

// Class constructor
Motor::Motor(char enablePin, char motorPin1, char motorPin2, int speed) {
	// Set up pins
	pinMode(motorPin1, OUTPUT);
	pinMode(motorPin2, OUTPUT);
	pinMode(enablePin, OUTPUT);

	// Initialise private variables
	_motorPin1 = motorPin1;
	_motorPin2 = motorPin2;
	_enablePin = enablePin;
	_speed = speed;
};

// functions to drive motor
void Motor::m_fwd(void) {
	analogWrite(_enablePin, _speed);
	digitalWrite(_motorPin1, LOW);
	digitalWrite(_motorPin2, HIGH); 
};

void Motor::m_back(void) {
	analogWrite(_enablePin, _speed);
	digitalWrite(_motorPin1, HIGH);
	digitalWrite(_motorPin2, LOW); 
};

void Motor::m_stop(void) {
	// Motor braking
	analogWrite(_enablePin, _speed);
	digitalWrite(_motorPin1, HIGH);
	digitalWrite(_motorPin2, HIGH);
	delay(200);
  
	// When motor is stopped, turn off all pins
	analogWrite(_enablePin, 0);
	digitalWrite(_motorPin1, LOW);
	digitalWrite(_motorPin2, LOW);
};

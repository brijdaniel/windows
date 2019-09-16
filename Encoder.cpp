#include "Arduino.h"
#include "Encoder.h"

// Class constructur
Encoder::Encoder(char encoderPin, void encoder_dispatch(void)) {
	// Set up pin
	pinMode(encoderPin, INPUT_PULLUP);

	// Initialise private variables
	_encoderPin = encoderPin;
	_encoder_value = 0;
	_stall_time = 100; // 100ms

	// Declare update_encoder as ISR
	void ICACHE_RAM_ATTR update_encoder();
	
	// Set encoder interrupt to pin
	attachInterrupt(digitalPinToInterrupt(encoderPin), encoder_dispatch, FALLING);
};

/* 
Function to read encoder
Checks encoder value is less than the desired number of motor rotations
Also checks time since last encoder tick, to make sure motor hasn't stalled
This function is essentially just a while loop that does nothing, as when this
function ends, the next function to be called is motor_stop
*/
void Encoder::encoder_count(float rotations) {
	_rotations = 150*11*rotations; // 150:1 gearbox reduction and 11 ticks per rotation
	_encoder_value = 0;
	while (_encoder_value < _rotations) {
		// Motor stall monitoring
		if (_encoder_value > 50) { // make sure the motor has spun up to speed before we check if its stalled
			_current_time = millis();
			// Using timer variable to fix overflow problem of 32-bit unsigned long
			// int overflows more often, but returns -1 when it does, so just added 2
			// ensures its always positive... pretty dodgy but works
			unsigned int _timer = _current_time - _previous_time + 2;
			/* 
			Check if motor has stalled by making sure time between encoder ticks
			doesnt exceed global stall_time variable (currently set to 100ms)
			*/
			if (_timer >= _stall_time) {
				Serial.println("Motor stalled");
				Serial.println(_timer);
				break;
			};
		};
	ESP.wdtFeed(); // Feed the watchdog so it doesnt reboot the controller
	};
};

/* 
Increment encoder value and set time of tick (inputs to encoder_count)
Must be set as ISR in main code, ie:
void ICACHE_RAM_ATTR encoder.update_encoder();
Then interrupt must be attached to encoder pin, ie:
attachInterrupt(digitalPinToInterrupt(encoderPin), encoder.update_encoder, FALLING);
*/
void Encoder::update_encoder() {
	_encoder_value++;
	_previous_time = millis();
};

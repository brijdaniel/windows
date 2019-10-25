#include "Arduino.h"
#ifndef Encoder_h
#define Encoder_h

class Encoder {
	public:
		Encoder(char encoderPin, void encoder_dispatch(void));
		void encoder_count(float rotations);
		void update_encoder();
		
	private:
		char _encoderPin;
		float _rotations;
		volatile long _encoder_value;
		volatile unsigned long _current_time;
		volatile unsigned long _previous_time;
		unsigned int _timer;
		unsigned long _stall_time;
};

#endif

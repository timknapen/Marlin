#ifndef libServo_Teensy_h
#define libServo_Teensy_h

#include <Servo.h>

// Inherit and expand on the official library
class libServo : public Servo {
	public:
		void move(int value);
		int read();
	private:
	   uint16_t min_ticks;
	   uint16_t max_ticks;
	   uint8_t servoIndex;               // index into the channel data for this servo
};

#endif


#include "libServo_Teensy.h"
#include "../../../MarlinConfig.h"


static uint16_t servo_ticks[MAX_SERVOS];


void libServo::move(int value) {
  if (this->attach(0) >= 0) {
    this->write(value);
    delay(SERVO_DELAY);
    #if ENABLED(DEACTIVATE_SERVOS_AFTER_MOVE)
      this->detach();
    #endif
  }
}

int libServo::read() // return the value as degrees
{
	if (servoIndex >= MAX_SERVOS) return 0;
	return map(servo_ticks[servoIndex], min_ticks, max_ticks, 0, 180);     
}

#if defined(__MK64FX512__) || defined(__MK66FX1M0__)

#include "HAL_Servo_Teensy.h"
#include "../../inc/MarlinConfig.h"

int8_t libServo::attach(const int pin) {
  this->pin = pin;
  if (this->servoIndex >= MAX_SERVOS) return -1;
  return Servo::attach(pin);
}

int8_t libServo::attach(const int pin, const int min, const int max) {
	this->pin = pin;
  return Servo::attach(pin, min, max);
}

void libServo::move(const int value) {
  if (this->attach(this->pin) >= 0) {
    this->write(value);
    delay(SERVO_DELAY);
  }
}

#endif // __MK64FX512__ || __MK66FX1M0__

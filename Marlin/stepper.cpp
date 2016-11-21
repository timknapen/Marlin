/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * stepper.cpp - A singleton object to execute motion plans using stepper motors
 * Marlin Firmware
 *
 * Derived from Grbl
 * Copyright (c) 2009-2011 Simen Svale Skogsrud
 *
 * Grbl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Grbl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
 */

/* The timer calculations of this module informed by the 'RepRap cartesian firmware' by Zack Smith
   and Philipp Tiefenbacher. */

#include "Marlin.h"
#include "stepper.h"
#include "endstops.h"
#include "planner.h"
#include "temperature.h"
#include "ultralcd.h"
#include "language.h"
#include "cardreader.h"
#include "speed_lookuptable.h"

Stepper stepper; // Singleton

// public:

block_t* Stepper::current_block = NULL;  // A pointer to the block currently being traced

// private:

unsigned char Stepper::last_direction_bits = 0;        // The next stepping-bits to be output
unsigned int Stepper::cleaning_buffer_counter = 0;

long Stepper::counter_X = 0,
     Stepper::counter_Y = 0,
     Stepper::counter_Z = 0,
     Stepper::counter_E = 0;

volatile uint32_t Stepper::step_events_completed = 0; // The number of step events executed in the current block

long Stepper::acceleration_time, Stepper::deceleration_time;

volatile long Stepper::count_position[NUM_AXIS] = { 0 };
volatile signed char Stepper::count_direction[NUM_AXIS] = { 1, 1, 1, 1 };

unsigned short Stepper::acc_step_rate; // needed for deceleration start point
uint8_t Stepper::step_loops, Stepper::step_loops_nominal;
unsigned short Stepper::OCR1A_nominal;

volatile long Stepper::endstops_trigsteps[XYZ];

#define X_APPLY_DIR(v,Q) X_DIR_WRITE(v)
#define X_APPLY_STEP(v,Q) X_STEP_WRITE(v)

#define Y_APPLY_DIR(v,Q) Y_DIR_WRITE(v)
#define Y_APPLY_STEP(v,Q) Y_STEP_WRITE(v)

#define Z_APPLY_DIR(v,Q) Z_DIR_WRITE(v)
#define Z_APPLY_STEP(v,Q) Z_STEP_WRITE(v)

#define E_APPLY_STEP(v,Q) E_STEP_WRITE(v)

	// intRes = longIn1 * longIn2 >> 24
#if defined(__AVR__)
	// uses:
	// r26 to store 0
	// r27 to store bits 16-23 of the 48bit result. The top bit is used to round the two byte result.
	// note that the lower two bytes and the upper byte of the 48bit result are not calculated.
	// this can cause the result to be out by one as the lower bytes may cause carries into the upper ones.
	// B0 A0 are bits 24-39 and are the returned value
	// C1 B1 A1 is longIn1
	// D2 C2 B2 A2 is longIn2
	//
	#define MultiU24X32toH16(intRes, longIn1, longIn2) \
	  asm volatile ( \
	                 "clr r26 \n\t" \
	                 "mul %A1, %B2 \n\t" \
	                 "mov r27, r1 \n\t" \
	                 "mul %B1, %C2 \n\t" \
	                 "movw %A0, r0 \n\t" \
	                 "mul %C1, %C2 \n\t" \
	                 "add %B0, r0 \n\t" \
	                 "mul %C1, %B2 \n\t" \
	                 "add %A0, r0 \n\t" \
	                 "adc %B0, r1 \n\t" \
	                 "mul %A1, %C2 \n\t" \
	                 "add r27, r0 \n\t" \
	                 "adc %A0, r1 \n\t" \
	                 "adc %B0, r26 \n\t" \
	                 "mul %B1, %B2 \n\t" \
	                 "add r27, r0 \n\t" \
	                 "adc %A0, r1 \n\t" \
	                 "adc %B0, r26 \n\t" \
	                 "mul %C1, %A2 \n\t" \
	                 "add r27, r0 \n\t" \
	                 "adc %A0, r1 \n\t" \
	                 "adc %B0, r26 \n\t" \
	                 "mul %B1, %A2 \n\t" \
	                 "add r27, r1 \n\t" \
	                 "adc %A0, r26 \n\t" \
	                 "adc %B0, r26 \n\t" \
	                 "lsr r27 \n\t" \
	                 "adc %A0, r26 \n\t" \
	                 "adc %B0, r26 \n\t" \
	                 "mul %D2, %A1 \n\t" \
	                 "add %A0, r0 \n\t" \
	                 "adc %B0, r1 \n\t" \
	                 "mul %D2, %B1 \n\t" \
	                 "add %B0, r0 \n\t" \
	                 "clr r1 \n\t" \
	                 : \
	                 "=&r" (intRes) \
	                 : \
	                 "d" (longIn1), \
	                 "d" (longIn2) \
	                 : \
	                 "r26" , "r27" \
	               )
#elif defined(__MK64FX512__)
	#define MultiU24X32toH16(intRes, longIn1, longIn2) (intRes = ((uint64_t)longIn1 * longIn2) >> 24)
#endif

#if defined(__AVR__)
	// Some useful constants
	#define ENABLE_STEPPER_DRIVER_INTERRUPT()  SBI(TIMSK1, OCIE1A)
	#define DISABLE_STEPPER_DRIVER_INTERRUPT() CBI(TIMSK1, OCIE1A)
#elif defined(__MK64FX512__) // && defined(IRQ_FTM2)
	#define ENABLE_STEPPER_DRIVER_INTERRUPT()  NVIC_ENABLE_IRQ(IRQ_FTM2)
	#define DISABLE_STEPPER_DRIVER_INTERRUPT() NVIC_DISABLE_IRQ(IRQ_FTM2)
	#define ISR(func) static void func (void)
#endif

/**
 *         __________________________
 *        /|                        |\     _________________         ^
 *       / |                        | \   /|               |\        |
 *      /  |                        |  \ / |               | \       s
 *     /   |                        |   |  |               |  \      p
 *    /    |                        |   |  |               |   \     e
 *   +-----+------------------------+---+--+---------------+----+    e
 *   |               BLOCK 1            |      BLOCK 2          |    d
 *
 *                           time ----->
 *
 *  The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates
 *  first block->accelerate_until step_events_completed, then keeps going at constant speed until
 *  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
 *  The slope of acceleration is calculated using v = u + at where t is the accumulated timer values of the steps so far.
 */
void Stepper::wake_up() {
  //  TCNT1 = 0;
  #if defined(__AVR__)
  	ENABLE_STEPPER_DRIVER_INTERRUPT();
  #endif
}

void Stepper::set_directions() {

  #define SET_STEP_DIR(AXIS) \
    if (motor_direction(AXIS ##_AXIS)) { \
      AXIS ##_APPLY_DIR(INVERT_## AXIS ##_DIR, false); \
      count_direction[AXIS ##_AXIS] = -1; \
    } \
    else { \
      AXIS ##_APPLY_DIR(!INVERT_## AXIS ##_DIR, false); \
      count_direction[AXIS ##_AXIS] = 1; \
    }

  SET_STEP_DIR(X); // A
  SET_STEP_DIR(Y); // B
  SET_STEP_DIR(Z); // C

  if (motor_direction(E_AXIS)) {
    REV_E_DIR();
    count_direction[E_AXIS] = -1;
  }
  else {
    NORM_E_DIR();
    count_direction[E_AXIS] = 1;
  }
}

/**
 * Stepper Driver Interrupt
 *
 * Directly pulses the stepper motors at high frequency.
 * Timer 1 runs at a base frequency of 2MHz, with this ISR using OCR1A compare mode.
 *
 * OCR1A   Frequency
 *     1     2 MHz
 *    50    40 KHz
 *   100    20 KHz - capped max rate
 *   200    10 KHz - nominal max rate
 *  2000     1 KHz - sleep rate
 *  4000   500  Hz - init rate
 */
#if defined(__AVR__)
	ISR(TIMER1_COMPA_vect) { Stepper::isr(); }
#elif defined(__MK64FX512__) && defined(IRQ_FTM2)
void ftm2_isr(void) {
  int flags = FTM2_STATUS;
  FTM2_STATUS = 0;
  if (flags & 0x01) {
    FTM2_C0V += OCR1Aval;
    TIMER1_COMPA_vect();
  }
}
#endif

void Stepper::isr() {
  if (cleaning_buffer_counter) {
    current_block = NULL;
    planner.discard_current_block();
    cleaning_buffer_counter--;
    //COMPARE1A = 200; // Run at max speed - 10 KHz
    setOCR1A(200);
    return;
  }

  // If there is no current block, attempt to pop one from the buffer
  if (!current_block) {
    // Anything in the buffer?
    current_block = planner.get_current_block();
    if (current_block) {
      SBI(current_block->flag, BLOCK_BIT_BUSY);
      trapezoid_generator_reset();

      // Initialize Bresenham counters to 1/2 the ceiling
      counter_X = counter_Y = counter_Z = counter_E = -(current_block->step_event_count >> 1);

      step_events_completed = 0;
    }
    else {
      //COMPARE1A = 2000; // Run at slow speed - 1 KHz
      setOCR1A(2000);
      return;
    }
  }

  // Update endstops state, if enabled
  if (endstops.enabled ) endstops.update();

  // Take multiple steps per interrupt (For high speed moves)
  bool all_steps_done = false;
  for (int8_t i = 0; i < step_loops; i++) {
    #ifndef USBCON
      customizedSerial.checkRx(); // Check for serial chars.
    #endif

    #define _COUNTER(AXIS) counter_## AXIS
    #define _APPLY_STEP(AXIS) AXIS ##_APPLY_STEP
    #define _INVERT_STEP_PIN(AXIS) INVERT_## AXIS ##_STEP_PIN

    // Advance the Bresenham counter; start a pulse if the axis needs a step
    #define PULSE_START(AXIS) \
      _COUNTER(AXIS) += current_block->steps[_AXIS(AXIS)]; \
      if (_COUNTER(AXIS) > 0) { _APPLY_STEP(AXIS)(!_INVERT_STEP_PIN(AXIS),0); }

    // Stop an active pulse, reset the Bresenham counter, update the position
    #define PULSE_STOP(AXIS) \
      if (_COUNTER(AXIS) > 0) { \
        _COUNTER(AXIS) -= current_block->step_event_count; \
        count_position[_AXIS(AXIS)] += count_direction[_AXIS(AXIS)]; \
        _APPLY_STEP(AXIS)(_INVERT_STEP_PIN(AXIS),0); \
      }

    PULSE_START(X);
    PULSE_START(Y);
    PULSE_START(Z);
    // For non-advance use linear interpolation for E also
    PULSE_START(E);

    PULSE_STOP(X);
    PULSE_STOP(Y);
    PULSE_STOP(Z);
    PULSE_STOP(E);

    if (++step_events_completed >= current_block->step_event_count) {
      all_steps_done = true;
      break;
    }
  }

  // Calculate new timer value
  uint16_t timer, step_rate;
  if (step_events_completed <= (uint32_t)current_block->accelerate_until) {

  	#if defined(__AVR__)
    	MultiU24X32toH16(acc_step_rate, acceleration_time, current_block->acceleration_rate);
    #elif defined(__MK64FX512__)
    	acc_step_rate = acceleration_time * current_block->acceleration_rate;
    #endif
    acc_step_rate += current_block->initial_rate;

    // upper limit
    NOMORE(acc_step_rate, current_block->nominal_rate);

    // step_rate to timer interval
	  timer = calc_timer(acc_step_rate);
    setOCR1A(timer);
    acceleration_time += timer;
  }
  else if (step_events_completed > (uint32_t)current_block->decelerate_after) {
  	#if defined(__AVR__)
    	MultiU24X32toH16(step_rate, deceleration_time, current_block->acceleration_rate);
  	#elif defined(__MK64FX512__)
  		step_rate = deceleration_time * current_block->acceleration_rate;
		#endif

    if (step_rate < acc_step_rate) { // Still decelerating?
      step_rate = acc_step_rate - step_rate;
      NOLESS(step_rate, current_block->final_rate);
    }
    else
      step_rate = current_block->final_rate;

  	// step_rate to timer interval
  	timer = calc_timer(step_rate);
    setOCR1A(timer);
    deceleration_time += timer;
  }
  else {
    setOCR1A(OCR1A_nominal);
    // ensure we're running at the correct step rate, even if we just came off an acceleration
    step_loops = step_loops_nominal;
  }
	//NOLESS(COMPARE1A, TCNT1 + 16);
  // If current block is finished, reset pointer
  if (all_steps_done) {
    current_block = NULL;
    planner.discard_current_block();
  }
}

void Stepper::init() {  
  // Init TMC2130 Steppers
  #if ENABLED(HAVE_TMC2130)
    tmc2130_init();
  #endif

  // Init Dir Pins
  X_DIR_INIT;
  Y_DIR_INIT;
  Z_DIR_INIT;
  E0_DIR_INIT;

  // Init Enable Pins - steppers default to disabled.
  X_ENABLE_INIT;
  if (!X_ENABLE_ON) X_ENABLE_WRITE(HIGH);
  Y_ENABLE_INIT;
  if (!Y_ENABLE_ON) Y_ENABLE_WRITE(HIGH);
  Z_ENABLE_INIT;
  if (!Z_ENABLE_ON) Z_ENABLE_WRITE(HIGH);
  E0_ENABLE_INIT;
  if (!E_ENABLE_ON) E0_ENABLE_WRITE(HIGH);

  // Init endstops and pullups
  endstops.init();

  #define _STEP_INIT(AXIS) AXIS ##_STEP_INIT
  #define _WRITE_STEP(AXIS, HIGHLOW) AXIS ##_STEP_WRITE(HIGHLOW)
  #define _DISABLE(axis) disable_## axis()

  #define AXIS_INIT(axis, AXIS, PIN) \
    _STEP_INIT(AXIS); \
    _WRITE_STEP(AXIS, _INVERT_STEP_PIN(PIN)); \
    _DISABLE(axis)

  #define E_AXIS_INIT(NUM) AXIS_INIT(e## NUM, E## NUM, E)

  // Init Step Pins
  AXIS_INIT(x, X, X);
  AXIS_INIT(y, Y, Y);
  AXIS_INIT(z, Z, Z);
  E_AXIS_INIT(0);

  #if defined(__AVR__)
	  // waveform generation = 0100 = CTC
	  CBI(TCCR1B, WGM13);
	  SBI(TCCR1B, WGM12);
	  CBI(TCCR1A, WGM11);
	  CBI(TCCR1A, WGM10);

	  // output mode = 00 (disconnected)
	  TCCR1A &= ~(3 << COM1A0);
	  TCCR1A &= ~(3 << COM1B0);

	  // Set the timer pre-scaler
	  // Generally we use a divider of 8, resulting in a 2MHz timer
	  // frequency on a 16MHz MCU. If you are going to change this, be
	  // sure to regenerate speed_lookuptable.h with
	  // create_speed_lookuptable.py
	  TCCR1B = (TCCR1B & ~(0x07 << CS10)) | (2 << CS10);

	  // Init Stepper ISR to 122 Hz for quick starting
    setOCR1A(0x4000);
	  TCNT1 = 0;
	#elif defined(__MK20DX256__)
	  FTM2_SC = 0; // Init FlexTimer2 Status and Control register
    FTM2_CNT = 0; // Init counter value to 0
	  FTM2_MOD = 0xFFFF; // Set top value for FTM2: 65335
    setOCR1A(0x4000);
	  FTM2_C0SC = 0x68; // b0110 1000 // CHF=0, CHIE=1, MSB=1, MSA=0, ELSB=1, ELSA=0, DMA=0
	  #if F_BUS >= 32000000
		  FTM2_SC = FTM_SC_CLKS(4) | FTM_SC_CLKS(1);
	  #elif F_BUS >= 16000000
		  FTM2_SC = FTM_SC_CLKS(3) | FTM_SC_CLKS(1);
	  #else
		  #error "Clock must be at least 16 MHz"
	  #endif
	#endif

	ENABLE_STEPPER_DRIVER_INTERRUPT();

  endstops.enable(true); // Start with endstops active. After homing they can be disabled
  sei();

  set_directions(); // Init directions to last_direction_bits = 0
}


/**
 * Block until all buffered steps are executed
 */
void Stepper::synchronize() { while (planner.blocks_queued()) idle(); }

/**
 * Set the stepper positions directly in steps
 *
 * The input is based on the typical per-axis XYZ steps.
 * For CORE machines XYZ needs to be translated to ABC.
 *
 * This allows get_axis_position_mm to correctly
 * derive the current XYZ position later on.
 */
void Stepper::set_position(const long &a, const long &b, const long &c, const long &e) {

  synchronize(); // Bad to set stepper counts in the middle of a move

  CRITICAL_SECTION_START;

  // default non-h-bot planning
  count_position[X_AXIS] = a;
  count_position[Y_AXIS] = b;
  count_position[Z_AXIS] = c;

  count_position[E_AXIS] = e;
  CRITICAL_SECTION_END;
}

void Stepper::set_position(const AxisEnum &axis, const long &v) {
  CRITICAL_SECTION_START;
  count_position[axis] = v;
  CRITICAL_SECTION_END;
}

void Stepper::set_e_position(const long &e) {
  CRITICAL_SECTION_START;
  count_position[E_AXIS] = e;
  CRITICAL_SECTION_END;
}

/**
 * Get a stepper's position in steps.
 */
long Stepper::position(AxisEnum axis) {
  CRITICAL_SECTION_START;
  long count_pos = count_position[axis];
  CRITICAL_SECTION_END;
  return count_pos;
}

/**
 * Get an axis position according to stepper position(s)
 * For CORE machines apply translation from ABC to XYZ.
 */
float Stepper::get_axis_position_mm(AxisEnum axis) {
  float axis_steps;
  axis_steps = position(axis);
  return axis_steps * planner.steps_to_mm[axis];
}

void Stepper::finish_and_disable() {
  synchronize();
  disable_all_steppers();
}

void Stepper::quick_stop() {
  cleaning_buffer_counter = 5000;
  #if defined(__AVR__)
  	DISABLE_STEPPER_DRIVER_INTERRUPT();
  #endif
  while (planner.blocks_queued()) planner.discard_current_block();
  current_block = NULL;
  #if defined(__AVR__)
  	ENABLE_STEPPER_DRIVER_INTERRUPT();
  #endif
}

void Stepper::endstop_triggered(AxisEnum axis) {
  endstops_trigsteps[axis] = count_position[axis];
  kill_current_block();
}

void Stepper::report_positions() {
  CRITICAL_SECTION_START;
  long xpos = count_position[X_AXIS],
       ypos = count_position[Y_AXIS],
       zpos = count_position[Z_AXIS];
  CRITICAL_SECTION_END;

  SERIAL_PROTOCOLPGM(MSG_COUNT_X);
  SERIAL_PROTOCOL(xpos);

  SERIAL_PROTOCOLPGM(" Y:");
  SERIAL_PROTOCOL(ypos);

  SERIAL_PROTOCOLPGM(" Z:");
  SERIAL_PROTOCOL(zpos);

  SERIAL_EOL;
}

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

#include "stepper.h"

#ifdef __AVR__
  #include "speed_lookuptable.h"
#endif

#include "endstops.h"
#include "planner.h"
#include "motion.h"

#include "../core/language.h"
#include "../gcode/queue.h"
#include "../sd/cardreader.h"
#include "../Marlin.h"



Stepper stepper; // Singleton

// public:

block_t* Stepper::current_block = NULL;  // A pointer to the block currently being traced

#if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
  bool Stepper::abort_on_endstop_hit = false;
#endif

#if ENABLED(X_DUAL_ENDSTOPS) || ENABLED(Y_DUAL_ENDSTOPS)
  bool Stepper::performing_homing = false;
#endif

#if HAS_MOTOR_CURRENT_PWM
  uint32_t Stepper::motor_current_setting[3]; // Initialized by settings.load()
#endif

// private:

uint8_t Stepper::last_direction_bits = 0;        // The next stepping-bits to be output
int16_t Stepper::cleaning_buffer_counter = 0;

#if ENABLED(X_DUAL_ENDSTOPS)
  bool Stepper::locked_x_motor = false, Stepper::locked_x2_motor = false;
#endif
#if ENABLED(Y_DUAL_ENDSTOPS)
  bool Stepper::locked_y_motor = false, Stepper::locked_y2_motor = false;
#endif

long Stepper::counter_X = 0,
     Stepper::counter_Y = 0;

volatile uint32_t Stepper::step_events_completed = 0; // The number of step events executed in the current block


long Stepper::acceleration_time, Stepper::deceleration_time;

volatile long Stepper::count_position[NUM_AXIS] = { 0 };
volatile signed char Stepper::count_direction[NUM_AXIS] = { 1, 1 };


uint8_t Stepper::step_loops, Stepper::step_loops_nominal;

hal_timer_t Stepper::OCR1A_nominal,
            Stepper::acc_step_rate; // needed for deceleration start point

volatile long Stepper::endstops_trigsteps[XY];

#if ENABLED(X_DUAL_ENDSTOPS) || ENABLED(Y_DUAL_ENDSTOPS)
  #define LOCKED_X_MOTOR  locked_x_motor
  #define LOCKED_Y_MOTOR  locked_y_motor

  #define DUAL_ENDSTOP_APPLY_STEP(AXIS,v)                                                                                                             \
    if (performing_homing) {                                                                                                                          \
      if (AXIS##_HOME_DIR < 0) {                                                                                                                      \
        if (!(TEST(endstops.old_endstop_bits, AXIS##_MIN) && (count_direction[AXIS##_AXIS] < 0)) && !LOCKED_##AXIS##_MOTOR) AXIS##_STEP_WRITE(v);     \
        if (!(TEST(endstops.old_endstop_bits, AXIS##2_MIN) && (count_direction[AXIS##_AXIS] < 0)) && !LOCKED_##AXIS##2_MOTOR) AXIS##2_STEP_WRITE(v);  \
      }                                                                                                                                               \
      else {                                                                                                                                          \
        if (!(TEST(endstops.old_endstop_bits, AXIS##_MAX) && (count_direction[AXIS##_AXIS] > 0)) && !LOCKED_##AXIS##_MOTOR) AXIS##_STEP_WRITE(v);     \
        if (!(TEST(endstops.old_endstop_bits, AXIS##2_MAX) && (count_direction[AXIS##_AXIS] > 0)) && !LOCKED_##AXIS##2_MOTOR) AXIS##2_STEP_WRITE(v);  \
      }                                                                                                                                               \
    }                                                                                                                                                 \
    else {                                                                                                                                            \
      AXIS##_STEP_WRITE(v);                                                                                                                           \
      AXIS##2_STEP_WRITE(v);                                                                                                                          \
    }
#endif

  #define X_APPLY_DIR(v,Q) X_DIR_WRITE(v)
  #define X_APPLY_STEP(v,Q) X_STEP_WRITE(v)


  #define Y_APPLY_DIR(v,Q) Y_DIR_WRITE(v)
  #define Y_APPLY_STEP(v,Q) Y_STEP_WRITE(v)




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
  // TCNT1 = 0;
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}

/**
 * Set the stepper direction of each axis
 *
 *   COREXY: X_AXIS=A_AXIS and Y_AXIS=B_AXIS
 */
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

  #if HAS_X_DIR
    SET_STEP_DIR(X); // A
  #endif
  #if HAS_Y_DIR
    SET_STEP_DIR(Y); // B
  #endif

}

#if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
  extern volatile uint8_t e_hit;
#endif

/**
 * Stepper Driver Interrupt
 *
 * Directly pulses the stepper motors at high frequency.
 *
 * AVR :
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

HAL_STEP_TIMER_ISR {
	HAL_timer_isr_prologue(STEP_TIMER_NUM);
	Stepper::isr();
}

void Stepper::isr() {

  #define ENDSTOP_NOMINAL_OCR_VAL 1500 * HAL_TICKS_PER_US // Check endstops every 1.5ms to guarantee two stepper ISRs within 5ms for BLTouch
  #define OCR_VAL_TOLERANCE        500 * HAL_TICKS_PER_US // First max delay is 2.0ms, last min delay is 0.5ms, all others 1.5ms

  #if DISABLED(LIN_ADVANCE)
    // Disable Timer0 ISRs and enable global ISR again to capture UART events (incoming chars)
    DISABLE_STEPPER_DRIVER_INTERRUPT();
    #ifndef CPU_32_BIT
      sei();
    #endif
  #endif

  hal_timer_t ocr_val;
  static uint32_t step_remaining = 0;  // SPLIT function always runs.  This allows 16 bit timers to be
                                       // used to generate the stepper ISR.
  #define SPLIT(L) do { \
    if (L > ENDSTOP_NOMINAL_OCR_VAL) { \
      const uint32_t remainder = (uint32_t)L % (ENDSTOP_NOMINAL_OCR_VAL); \
      ocr_val = (remainder < OCR_VAL_TOLERANCE) ? ENDSTOP_NOMINAL_OCR_VAL + remainder : ENDSTOP_NOMINAL_OCR_VAL; \
      step_remaining = (uint32_t)L - ocr_val; \
    } \
    else \
      ocr_val = L;\
  }while(0)

  // Time remaining before the next step?
  if (step_remaining) {

    // Make sure endstops are updated
    if (ENDSTOPS_ENABLED) endstops.update();

    // Next ISR either for endstops or stepping
    ocr_val = step_remaining <= ENDSTOP_NOMINAL_OCR_VAL ? step_remaining : ENDSTOP_NOMINAL_OCR_VAL;
    step_remaining -= ocr_val;
    _NEXT_ISR(ocr_val);

    #if DISABLED(LIN_ADVANCE)
      #ifdef CPU_32_BIT
        HAL_timer_set_count(STEP_TIMER_NUM, ocr_val);
      #else
        NOLESS(OCR1A, TCNT1 + 16);
      #endif
      HAL_ENABLE_ISRs(); // re-enable ISRs
    #endif

    return;
  }

  //
  // When cleaning, discard the current block and run fast
  //
  if (cleaning_buffer_counter) {
    if (cleaning_buffer_counter < 0) {          // Count up for endstop hit
      if (current_block) planner.discard_current_block(); // Discard the active block that led to the trigger
      if (!planner.discard_continued_block())   // Discard next CONTINUED block
        cleaning_buffer_counter = 0;            // Keep discarding until non-CONTINUED
    }
    else {
      planner.discard_current_block();
      --cleaning_buffer_counter;                // Count down for abort print
      #ifdef SD_FINISHED_RELEASECOMMAND
        if (!cleaning_buffer_counter && (SD_FINISHED_STEPPERRELEASE)) enqueue_and_echo_commands_P(PSTR(SD_FINISHED_RELEASECOMMAND));
      #endif
    }
    current_block = NULL;                       // Prep to get a new block after cleaning
    _NEXT_ISR(HAL_STEPPER_TIMER_RATE / 10000);  // Run at max speed - 10 KHz
    HAL_ENABLE_ISRs();
    return;
  }

  // If there is no current block, attempt to pop one from the buffer
  bool first_step = false;
  if (!current_block) {
    // Anything in the buffer?
    if ((current_block = planner.get_current_block())) {
      trapezoid_generator_reset();
      HAL_timer_set_current_count(STEP_TIMER_NUM, 0);
      first_step = true;

      // Initialize Bresenham counters to 1/2 the ceiling
      counter_X = counter_Y =  -(current_block->step_event_count >> 1);


      step_events_completed = 0;

      #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
        e_hit = 2; // Needed for the case an endstop is already triggered before the new move begins.
                   // No 'change' can be detected.
      #endif

	}
    else {
      _NEXT_ISR(HAL_STEPPER_TIMER_RATE / 1000); // Run at slow speed - 1 KHz
      HAL_ENABLE_ISRs(); // re-enable ISRs
      return;
    }
  }

  // Update endstops state, if enabled
  #if ENABLED(ENDSTOP_INTERRUPTS_FEATURE)
    if (e_hit && ENDSTOPS_ENABLED) {
      endstops.update();
      e_hit--;
    }
  #else
    if (ENDSTOPS_ENABLED) endstops.update();
  #endif

  // Take multiple steps per interrupt (For high speed moves)
  bool all_steps_done = false;
  for (uint8_t i = step_loops; i--;) {
	  
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

    /**
     * Estimate the number of cycles that the stepper logic already takes
     * up between the start and stop of the X stepper pulse.
     *
     * Currently this uses very modest estimates of around 5 cycles.
     * True values may be derived by careful testing.
     *
     * Once any delay is added, the cost of the delay code itself
     * may be subtracted from this value to get a more accurate delay.
     * Delays under 20 cycles (1.25µs) will be very accurate, using NOPs.
     * Longer delays use a loop. The resolution is 8 cycles.
     */
    #if HAS_X_STEP
      #define _CYCLE_APPROX_1 5
    #else
      #define _CYCLE_APPROX_1 0
    #endif
	
     #define _CYCLE_APPROX_2 _CYCLE_APPROX_1
    #if HAS_Y_STEP
      #define _CYCLE_APPROX_3 _CYCLE_APPROX_2 + 5
    #else
      #define _CYCLE_APPROX_3 _CYCLE_APPROX_2
    #endif
	
    #define CYCLES_EATEN_XY _CYCLE_APPROX_3
    #define EXTRA_CYCLES_XY (STEP_PULSE_CYCLES - (CYCLES_EATEN_XY))

    /**
     * If a minimum pulse time was specified get the timer 0 value.
     *
     * On AVR the TCNT0 timer has an 8x prescaler, so it increments every 8 cycles.
     * That's every 0.5µs on 16MHz and every 0.4µs on 20MHz.
     * 20 counts of TCNT0 -by itself- is a good pulse delay.
     * 10µs = 160 or 200 cycles.
     */
    #if EXTRA_CYCLES_XY > 20
      hal_timer_t pulse_start = HAL_timer_get_current_count(PULSE_TIMER_NUM);
    #endif

    #if HAS_X_STEP
      PULSE_START(X);
    #endif
    #if HAS_Y_STEP
      PULSE_START(Y);
    #endif


    // For minimum pulse time wait before stopping pulses
    #if EXTRA_CYCLES_XY > 20
      while (EXTRA_CYCLES_XY > (uint32_t)(HAL_timer_get_current_count(PULSE_TIMER_NUM) - pulse_start) * (PULSE_TIMER_PRESCALE)) { /* nada */ }
      pulse_start = HAL_timer_get_current_count(PULSE_TIMER_NUM);
    #elif EXTRA_CYCLES_XY > 0
      DELAY_NOPS(EXTRA_CYCLES_XY);
    #endif

    #if HAS_X_STEP
      PULSE_STOP(X);
    #endif
    #if HAS_Y_STEP
      PULSE_STOP(Y);
    #endif
	

    if (++step_events_completed >= current_block->step_event_count) {
      all_steps_done = true;
      break;
    }

    // For minimum pulse time wait after stopping pulses also
    #if EXTRA_CYCLES_XY > 20
      if (i) while (EXTRA_CYCLES_XY> (uint32_t)(HAL_timer_get_current_count(PULSE_TIMER_NUM) - pulse_start) * (PULSE_TIMER_PRESCALE)) { /* nada */ }
    #elif EXTRA_CYCLES_XY > 0
      if (i) DELAY_NOPS(EXTRA_CYCLES_XY);
    #endif

  } // steps_loop

 
  // Calculate new timer value
  if (step_events_completed <= (uint32_t)current_block->accelerate_until) {

    if (first_step) {
      acc_step_rate = current_block->initial_rate;
      acceleration_time = 0;
    }
    else {
      #ifdef CPU_32_BIT
        MultiU32X24toH32(acc_step_rate, acceleration_time, current_block->acceleration_rate);
      #else
        MultiU24X32toH16(acc_step_rate, acceleration_time, current_block->acceleration_rate);
      #endif
      acc_step_rate += current_block->initial_rate;
    }

    // upper limit
    NOMORE(acc_step_rate, current_block->nominal_rate);

    // step_rate to timer interval
    const hal_timer_t interval = calc_timer_interval(acc_step_rate);

    SPLIT(interval);  // split step into multiple ISRs if larger than  ENDSTOP_NOMINAL_OCR_VAL
    _NEXT_ISR(ocr_val);

    acceleration_time += interval;

  }
  else if (step_events_completed > (uint32_t)current_block->decelerate_after) {
    hal_timer_t step_rate;
    #ifdef CPU_32_BIT
      MultiU32X24toH32(step_rate, deceleration_time, current_block->acceleration_rate);
    #else
      MultiU24X32toH16(step_rate, deceleration_time, current_block->acceleration_rate);
    #endif

    if (step_rate < acc_step_rate) { // Still decelerating?
      step_rate = acc_step_rate - step_rate;
      NOLESS(step_rate, current_block->final_rate);
    }
    else
      step_rate = current_block->final_rate;

    // step_rate to timer interval
    const hal_timer_t interval = calc_timer_interval(step_rate);

    SPLIT(interval);  // split step into multiple ISRs if larger than  ENDSTOP_NOMINAL_OCR_VAL
    _NEXT_ISR(ocr_val);
    deceleration_time += interval;

  }
  else {


    SPLIT(OCR1A_nominal);  // split step into multiple ISRs if larger than  ENDSTOP_NOMINAL_OCR_VAL
    _NEXT_ISR(ocr_val);
    // ensure we're running at the correct step rate, even if we just came off an acceleration
    step_loops = step_loops_nominal;
  }

  #if DISABLED(LIN_ADVANCE)
    #ifdef CPU_32_BIT
      // Make sure stepper interrupt does not monopolise CPU by adjusting count to give about 8 us room
      hal_timer_t stepper_timer_count = HAL_timer_get_count(STEP_TIMER_NUM),
                  stepper_timer_current_count = HAL_timer_get_current_count(STEP_TIMER_NUM) + 8 * HAL_TICKS_PER_US;
      HAL_timer_set_count(STEP_TIMER_NUM, max(stepper_timer_count, stepper_timer_current_count));
    #else
      NOLESS(OCR1A, TCNT1 + 16);
    #endif
  #endif

  // If current block is finished, reset pointer
  if (all_steps_done) {
    current_block = NULL;
    planner.discard_current_block();
  }
  #if DISABLED(LIN_ADVANCE)
    HAL_ENABLE_ISRs(); // re-enable ISRs
  #endif
}


void Stepper::init() {


  // Init Microstepping Pins
  #if HAS_MICROSTEPS
    microstep_init();
  #endif

  // Init TMC Steppers
  #if ENABLED(HAVE_TMCDRIVER)
    tmc_init();
  #endif

  // Init TMC2130 Steppers
  #if ENABLED(HAVE_TMC2130)
    tmc2130_init();
  #endif


  // Init Dir Pins
  #if HAS_X_DIR
    X_DIR_INIT;
  #endif
 
  #if HAS_Y_DIR
    Y_DIR_INIT;
  #endif
 
  // Init Enable Pins - steppers default to disabled.
  #if HAS_X_ENABLE
    X_ENABLE_INIT;
    if (!X_ENABLE_ON) X_ENABLE_WRITE(HIGH);
  #endif
  #if HAS_Y_ENABLE
    Y_ENABLE_INIT;
    if (!Y_ENABLE_ON) Y_ENABLE_WRITE(HIGH);
  #endif
	

  // Init endstops and pullups
  endstops.init();

  #define _STEP_INIT(AXIS) AXIS ##_STEP_INIT
  #define _WRITE_STEP(AXIS, HIGHLOW) AXIS ##_STEP_WRITE(HIGHLOW)
  #define _DISABLE(AXIS) disable_## AXIS()

  #define AXIS_INIT(AXIS, PIN) \
    _STEP_INIT(AXIS); \
    _WRITE_STEP(AXIS, _INVERT_STEP_PIN(PIN)); \
    _DISABLE(AXIS)


  // Init Step Pins
  #if HAS_X_STEP
    AXIS_INIT(X, X);
  #endif

  #if HAS_Y_STEP
    AXIS_INIT(Y, Y);
  #endif

 
  #ifdef __AVR__
    // waveform generation = 0100 = CTC
    SET_WGM(1, CTC_OCRnA);

    // output mode = 00 (disconnected)
    SET_COMA(1, NORMAL);

    // Set the timer pre-scaler
    // Generally we use a divider of 8, resulting in a 2MHz timer
    // frequency on a 16MHz MCU. If you are going to change this, be
    // sure to regenerate speed_lookuptable.h with
    // create_speed_lookuptable.py
    SET_CS(1, PRESCALER_8);  //  CS 2 = 1/8 prescaler

    // Init Stepper ISR to 122 Hz for quick starting
    OCR1A = 0x4000;
    TCNT1 = 0;
  #else
    // Init Stepper ISR to 122 Hz for quick starting
    HAL_timer_start(STEP_TIMER_NUM, 122);
  #endif

  ENABLE_STEPPER_DRIVER_INTERRUPT();


  endstops.enable(true); // Start with endstops active. After homing they can be disabled
  sei();

  set_directions(); // Init directions to last_direction_bits = 0
}


/**
 * Block until all buffered steps are executed / cleaned
 */
void Stepper::synchronize() { while (planner.blocks_queued() || cleaning_buffer_counter) idle(); }

/**
 * Set the stepper positions directly in steps
 *
 * The input is based on the typical per-axis XY steps.
 * For CORE machines XY needs to be translated to AB.
 *
 * This allows get_axis_position_mm to correctly
 * derive the current XY position later on.
 */
void Stepper::set_position(const long &a, const long &b) {

  synchronize(); // Bad to set stepper counts in the middle of a move

  CRITICAL_SECTION_START;

  #if CORE_IS_XY
    // corexy positioning
    // these equations follow the form of the dA and dB equations on http://www.corexy.com/theory.html
    count_position[A_AXIS] = a + b;
    count_position[B_AXIS] = CORESIGN(a - b);
  #else
    // default non-h-bot planning
    count_position[X_AXIS] = a;
    count_position[Y_AXIS] = b;
  #endif

  CRITICAL_SECTION_END;
}

void Stepper::set_position(const AxisEnum &axis, const long &v) {
  CRITICAL_SECTION_START;
  count_position[axis] = v;
  CRITICAL_SECTION_END;
}

/**
 * Get a stepper's position in steps.
 */
long Stepper::position(const AxisEnum axis) {
  CRITICAL_SECTION_START;
  const long count_pos = count_position[axis];
  CRITICAL_SECTION_END;
  return count_pos;
}

/**
 * Get an axis position according to stepper position(s)
 * For CORE machines apply translation from AB to XY.
 */
float Stepper::get_axis_position_mm(const AxisEnum axis) {
  float axis_steps;
  #if IS_CORE
    // Requesting one of the "core" axes?
    if (axis == CORE_AXIS_1 || axis == CORE_AXIS_2) {
      CRITICAL_SECTION_START;
      // ((a1+a2)+(a1-a2))/2 -> (a1+a2+a1-a2)/2 -> (a1+a1)/2 -> a1
      // ((a1+a2)-(a1-a2))/2 -> (a1+a2-a1+a2)/2 -> (a2+a2)/2 -> a2
      axis_steps = 0.5f * (
        axis == CORE_AXIS_2 ? CORESIGN(count_position[CORE_AXIS_1] - count_position[CORE_AXIS_2])
                            : count_position[CORE_AXIS_1] + count_position[CORE_AXIS_2]
      );
      CRITICAL_SECTION_END;
    }
    else
      axis_steps = position(axis);
  #else
    axis_steps = position(axis);
  #endif
  return axis_steps * planner.steps_to_mm[axis];
}

void Stepper::finish_and_disable() {
  synchronize();
  disable_all_steppers();
}

void Stepper::quick_stop() {
  cleaning_buffer_counter = 5000;
  DISABLE_STEPPER_DRIVER_INTERRUPT();
  while (planner.blocks_queued()) planner.discard_current_block();
  current_block = NULL;
  ENABLE_STEPPER_DRIVER_INTERRUPT();

}

void Stepper::endstop_triggered(const AxisEnum axis) {

  #if IS_CORE

    endstops_trigsteps[axis] = 0.5f * (
      axis == CORE_AXIS_2 ? CORESIGN(count_position[CORE_AXIS_1] - count_position[CORE_AXIS_2])
                          : count_position[CORE_AXIS_1] + count_position[CORE_AXIS_2]
    );

  #else // !COREXY
	
    endstops_trigsteps[axis] = count_position[axis];

  #endif // !COREXY

  kill_current_block();
  cleaning_buffer_counter = -1; // Discard the rest of the move
}

void Stepper::report_positions() {
  CRITICAL_SECTION_START;
  const long xpos = count_position[X_AXIS],
             ypos = count_position[Y_AXIS];
  CRITICAL_SECTION_END;

  #if CORE_IS_XY
    SERIAL_PROTOCOLPGM(MSG_COUNT_A);
  #else
    SERIAL_PROTOCOLPGM(MSG_COUNT_X);
  #endif
  SERIAL_PROTOCOL(xpos);

  #if CORE_IS_XY
    SERIAL_PROTOCOLPGM(" B:");
  #else
    SERIAL_PROTOCOLPGM(" Y:");
  #endif
  SERIAL_PROTOCOL(ypos);

  SERIAL_EOL();
}


/**
 * Software-controlled Stepper Motor Current
 */


#if HAS_MOTOR_CURRENT_PWM

  void Stepper::refresh_motor_power() {
    for (uint8_t i = 0; i < COUNT(motor_current_setting); ++i) {
      switch (i) {
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_XY)
          case 0:
        #endif
            digipot_current(i, motor_current_setting[i]);
        default: break;
      }
    }
  }

#endif // HAS_MOTOR_CURRENT_PWM

#if HAS_MOTOR_CURRENT_PWM

  void Stepper::digipot_current(const uint8_t driver, const int current) {

    #if HAS_DIGIPOTSS

      const uint8_t digipot_ch[] = DIGIPOT_CHANNELS;
      digitalPotWrite(digipot_ch[driver], current);

    #elif HAS_MOTOR_CURRENT_PWM

      if (WITHIN(driver, 0, 2))
        motor_current_setting[driver] = current; // update motor_current_setting

      #define _WRITE_CURRENT_PWM(P) analogWrite(MOTOR_CURRENT_PWM_## P ##_PIN, 255L * current / (MOTOR_CURRENT_PWM_RANGE))
      switch (driver) {
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_XY)
          case 0: _WRITE_CURRENT_PWM(XY); break;
        #endif
		  
      }
    #endif
  }

  void Stepper::digipot_init() {

    #if HAS_MOTOR_CURRENT_PWM

      #if PIN_EXISTS(MOTOR_CURRENT_PWM_XY)
        SET_OUTPUT(MOTOR_CURRENT_PWM_XY_PIN);
      #endif

      refresh_motor_power();

      // Set Timer5 to 31khz so the PWM of the motor power is as constant as possible. (removes a buzzing noise)
      SET_CS5(PRESCALER_1);

    #endif
  }

#endif

#if HAS_MICROSTEPS

  /**
   * Software-controlled Microstepping
   */

  void Stepper::microstep_init() {
    SET_OUTPUT(X_MS1_PIN);
    SET_OUTPUT(X_MS2_PIN);
    #if HAS_Y_MICROSTEPS
      SET_OUTPUT(Y_MS1_PIN);
      SET_OUTPUT(Y_MS2_PIN);
    #endif
	static const uint8_t microstep_modes[] = MICROSTEP_MODES;
    for (uint16_t i = 0; i < COUNT(microstep_modes); i++)
      microstep_mode(i, microstep_modes[i]);
  }

  void Stepper::microstep_ms(const uint8_t driver, const int8_t ms1, const int8_t ms2) {
    if (ms1 >= 0) switch (driver) {
      case 0: WRITE(X_MS1_PIN, ms1); break;
      #if HAS_Y_MICROSTEPS
        case 1: WRITE(Y_MS1_PIN, ms1); break;
      #endif
	}
    if (ms2 >= 0) switch (driver) {
      case 0: WRITE(X_MS2_PIN, ms2); break;
      #if HAS_Y_MICROSTEPS
        case 1: WRITE(Y_MS2_PIN, ms2); break;
      #endif
    }
  }

  void Stepper::microstep_mode(const uint8_t driver, const uint8_t stepping_mode) {
    switch (stepping_mode) {
      case 1: microstep_ms(driver, MICROSTEP1); break;
      case 2: microstep_ms(driver, MICROSTEP2); break;
      case 4: microstep_ms(driver, MICROSTEP4); break;
      case 8: microstep_ms(driver, MICROSTEP8); break;
      case 16: microstep_ms(driver, MICROSTEP16); break;
        case 32: microstep_ms(driver, MICROSTEP32); break;
    }
  }

  void Stepper::microstep_readings() {
    SERIAL_PROTOCOLLNPGM("MS1,MS2 Pins");
    SERIAL_PROTOCOLPGM("X: ");
    SERIAL_PROTOCOL(READ(X_MS1_PIN));
    SERIAL_PROTOCOLLN(READ(X_MS2_PIN));
    #if HAS_Y_MICROSTEPS
      SERIAL_PROTOCOLPGM("Y: ");
      SERIAL_PROTOCOL(READ(Y_MS1_PIN));
      SERIAL_PROTOCOLLN(READ(Y_MS2_PIN));
    #endif
  }

#endif // HAS_MICROSTEPS

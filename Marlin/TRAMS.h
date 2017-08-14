#pragma once
#include <stdio.h>
#include <stdbool.h>
#include "Marlin.h"
#include "Stepper.h"
#include "endstops.h"
#include <TMCStepper.h>

#define nop() asm volatile("nop")

class Trams;
extern Trams stepper;
class TramsEndstops;
extern TramsEndstops endstops;

/*
 * Reference switch configuration
 *
 * switch position, select right or left reference switch (not both)
 * switch polarity, select high or low activ (not both)
 */
#if ENABLED(USE_XMIN_PLUG)
  #define SWITCH_POSITION_X	0x21                  // left
  #if X_MIN_ENDSTOP_INVERTING
    #define SWITCH_POLARITY_X REF_SW_LOW_ACTIV    // low activ
  #else
    #define SWITCH_POLARITY_X REF_SW_HIGH_ACTIV   // high activ
  #endif
#else
  #define SWITCH_POSITION_X	0x11                  // right
  #if X_MAX_ENDSTOP_INVERTING
    #define SWITCH_POLARITY_X REF_SW_LOW_ACTIV    // low activ
  #else
    #define SWITCH_POLARITY_X REF_SW_HIGH_ACTIV   // high activ
  #endif
#endif

#if ENABLED(USE_YMIN_PLUG)
  #define SWITCH_POSITION_Y 0x21                  // left
  #if Y_MIN_ENDSTOP_INVERTING
    #define SWITCH_POLARITY_Y REF_SW_LOW_ACTIV    // low activ
  #else
    #define SWITCH_POLARITY_Y REF_SW_HIGH_ACTIV   // high activ
  #endif
#else
  #define SWITCH_POSITION_Y 0x11                  // right
  #if Y_MAX_ENDSTOP_INVERTING
    #define SWITCH_POLARITY_Y REF_SW_LOW_ACTIV    // low activ
  #else
    #define SWITCH_POLARITY_Y REF_SW_HIGH_ACTIV   // high activ
  #endif
#endif

#if ENABLED(USE_ZMIN_PLUG)
  #define SWITCH_POSITION_Z 0x21                  // left
  #if Z_MIN_ENDSTOP_INVERTING
    #define SWITCH_POLARITY_Z REF_SW_LOW_ACTIV    // low activ
  #else
    #define SWITCH_POLARITY_Z REF_SW_HIGH_ACTIV   // high activ
  #endif
#else
  #define SWITCH_POSITION_Z 0x11                  // right
  #if Z_MAZ_ENDSTOP_INVERTING
    #define SWITCH_POLARITY_Z REF_SW_LOW_ACTIV    // low activ
  #else
    #define SWITCH_POLARITY_Z REF_SW_HIGH_ACTIV   // high activ
  #endif
#endif

/**
 * Stepper direction
 */

// Motor direction
#define NORMAL_MOTOR_DIRECTION  0x00  // Normal motor direction
#define INVERSE_MOTOR_DIRECTION 0x10  // Inverse motor direction
#if INVERT_X_DIR
  #define STEPPER_DIRECTION_X INVERSE_MOTOR_DIRECTION
#else
  #define STEPPER_DIRECTION_X	NORMAL_MOTOR_DIRECTION
#endif

#if INVERT_Y_DIR
  #define STEPPER_DIRECTION_Y INVERSE_MOTOR_DIRECTION
#else
  #define STEPPER_DIRECTION_Y NORMAL_MOTOR_DIRECTION
#endif

#if INVERT_Z_DIR
  #define STEPPER_DIRECTION_Z INVERSE_MOTOR_DIRECTION
#else
  #define STEPPER_DIRECTION_Z NORMAL_MOTOR_DIRECTION
#endif

#if INVERT_E0_DIR
  #define STEPPER_DIRECTION_E0 INVERSE_MOTOR_DIRECTION
#else
  #define STEPPER_DIRECTION_E0 NORMAL_MOTOR_DIRECTION
#endif

// Polarity for reference switch
#define REF_SW_HIGH_ACTIV     0x00 	// non-inverted, high active: a high level on REFL stops the motor
#define REF_SW_LOW_ACTIV      0x0C	// inverted, low active: a low level on REFL stops the motor
/*
// Stop switch bits in RAMP_STAT register
#define STATUS_STOP_R_bp      1
#define STATUS_STOP_R_bm      0x2
#define STATUS_STOP_L_bp      0
#define STATUS_STOP_L_bm      0x1
*/
// Modes for RAMPMODE register
#define POSITIONING_MODE      0x00  // using all A, D and V parameters)
#define VELOCITY_MODE_POS     0x01  // positiv VMAX, using AMAX acceleration
#define VELOCITY_MODE_NEG     0x02  // negativ VMAX, using AMAX acceleration
#define HOLD_MODE             0x03  // velocity remains unchanged, unless stop event occurs

#define VZERO                 0x400 // flag in RAMP_STAT, 1: signals that the actual velocity is 0.

#define SPI_SPEED             16000000/8

// TRAMS steppers
extern TMC5130Stepper stepperX;
extern TMC5130Stepper stepperY;
extern TMC5130Stepper stepperZ;
extern TMC5130Stepper stepperE0;

class Trams: public Stepper {
  public:
    static void init();
    static void set_position(const long &a, const long &b, const long &c, const long &e);
    static void set_position(const AxisEnum &axis, const long &v);
    static void set_e_position(const long &e);
    static void calculate(void);
    static void isr();
    static void TMC5130_homing(const AxisEnum &axis, const float homing_feedrate_mm_s);
    static void synchronize();
    static void report_positions();
    typedef struct {
      uint32_t  initial_speed[NUM_AXIS],
                nominal_speed[NUM_AXIS],
                final_speed[NUM_AXIS],
                accel[NUM_AXIS];
      long  pos[NUM_AXIS],
            nextTimerClk;
      bool  calcready,
            pos_change_z;
    } motion_block_t;
  //private:
    static FORCE_INLINE void discard_current_motion_block(void);
    static FORCE_INLINE motion_block_t *get_current_motion_block(void);
    static FORCE_INLINE bool motion_blocks_queued(void);
    static FORCE_INLINE motion_block_t *get_next_free_motion_block(void);
    static FORCE_INLINE void append_motion_block(void);
    static unsigned char blocks_in_motion_queue();
    static void set_directions();
    static void TMC5130_enableDriver(const AxisEnum &axis);
    static void TMC5130_disableDriver(const AxisEnum &axis);
    static void TMC5130_init(TMC5130Stepper &st, uint8_t stepper_direction, uint16_t sw_register);
};

class TramsEndstops: public Endstops {
  public:
    void M119();
  private:
};

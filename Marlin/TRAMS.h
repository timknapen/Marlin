#pragma once
#include <stdio.h>
#include <stdbool.h>
#include "Marlin.h"
#include "Stepper.h"
#include "endstops.h"

#define nop() asm volatile("nop")

class Trams;
extern Trams stepper;
class TramsEndstops;
extern TramsEndstops endstops;

/**
 * Current configuration
 */
// only change if necessary
#define X_CURRENT_RUN   25  // Motor run current (0=1/32…31=32/32)
#define X_CURRENT_HOLD   8  // Standstill current (0=1/32…31=32/32)

#define Y_CURRENT_RUN   25  // Motor run current (0=1/32…31=32/32)
#define Y_CURRENT_HOLD	 8  // Standstill current (0=1/32…31=32/32)

#define Z_CURRENT_RUN   23  // Motor run current (0=1/32…31=32/32)
#define Z_CURRENT_HOLD   8  // Standstill current (0=1/32…31=32/32)

#define E0_CURRENT_RUN  28  // Motor run current (0=1/32…31=32/32)
#define E0_CURRENT_HOLD  8  // Standstill current (0=1/32…31=32/32)

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



/**
 * STALLGUARD
 */
// x-axis
#define STALLGUARD_X                // if selected, stallguard is active
#define STALLGUARDTHRESHOLD_X	0x08  // range 0x00..0x7F

// y-axis
#define STALLGUARD_Y                // if selected, stallguard is active
#define STALLGUARDTHRESHOLD_Y	0x80  // range 0x00..0x7F

// z-axis
//#define STALLGUARD_Z              // if selected, stallguard is active
#define STALLGUARDTHRESHOLD_Z	0x08  // range 0x00..0x7F

// TRINAMIC TMC5130 Register Address Defines
#define GCONF                 0x00 	//Global configuration flags
#define X_COMPARE             0x05	//Position  comparison  register
#define IHOLD_IRUN            0x10	//Driver current control
#define TCOOLTHRS             0x14	//This is the lower threshold velocity for switching on smart energy coolStep and stallGuard feature.
#define RAMPMODE              0x20	//Driving mode (Velocity, Positioning, Hold)
#define XACTUAL               0x21	//Actual motor position
#define VACTUAL               0x22	//Actual  motor  velocity  from  ramp  generator
#define VSTART                0x23	//Motor start velocity
#define A_1                   0x24	//First  acceleration  between  VSTART  and  V1
#define V_1                   0x25	//First  acceleration  /  deceleration  phase  target velocity
#define AMAX                  0x26	//Second  acceleration  between  V1  and  VMAX
#define VMAX                  0x27	//This is the target velocity in velocity mode. It can be changed any time during a motion.
#define DMAX                  0x28	//Deceleration between VMAX and V1
#define D_1                   0x2A 	//Deceleration  between  V1  and  VSTOP
                                    //Attention:  Do  not  set  0  in  positioning  mode, even if V1=0!
#define VSTOP                 0x2B	//Motor stop velocity (unsigned)
                                    //Attention: Set VSTOP > VSTART!
                                    //Attention:  Do  not  set  0  in  positioning  mode, minimum 10 recommend!
#define TZEROWAIT             0x2C	//Defines  the  waiting  time  after  ramping  down
                                    //to  zero  velocity  before  next  movement  or
                                    //direction  inversion  can  start.  Time  range  is about 0 to 2 seconds.
#define XTARGET               0x2D	//Target position for ramp mode
#define SW_MODE               0x34	//Switch mode configuration
#define RAMP_STAT             0x35	//Ramp status and switch event status
#define XLATCH                0x36	//Latches  XACTUAL  upon  a programmable switch event
#define CHOPCONF              0x6C	//Chopper and driver configuration
#define COOLCONF              0x6D	//coolStep smart current control register and stallGuard2 configuration
#define DRV_STATUS            0x6F	//stallGuard2 value and driver error flags

#define SET_IHOLD(a)		((a & 0x1F)<<0)
#define SET_IRUN(a)			((a & 0x1F)<<8)
#define SET_IHOLDDELAY(a)	((uint32_t)(a & 0xF)<<16)

#define READ_ACCESS           0x80	// Read access for spi communication

// Polarity for reference switch
#define REF_SW_HIGH_ACTIV     0x00 	// non-inverted, high active: a high level on REFL stops the motor
#define REF_SW_LOW_ACTIV      0x0C	// inverted, low active: a low level on REFL stops the motor

// Stop switch bits in RAMP_STAT register
#define STATUS_STOP_R_bp      1
#define STATUS_STOP_R_bm      0x2
#define STATUS_STOP_L_bp      0
#define STATUS_STOP_L_bm      0x1

// Modes for RAMPMODE register
#define POSITIONING_MODE      0x00  // using all A, D and V parameters)
#define VELOCITY_MODE_POS     0x01  // positiv VMAX, using AMAX acceleration
#define VELOCITY_MODE_NEG     0x02  // negativ VMAX, using AMAX acceleration
#define HOLD_MODE             0x03  // velocity remains unchanged, unless stop event occurs

#define VZERO                 0x400 // flag in RAMP_STAT, 1: signals that the actual velocity is 0.

#define SPI_SPEED             16000000/8

class TramsSPI {
  protected:
    static void spi_init(void);
    static uint32_t spi_readRegister(uint8_t address, const AxisEnum &axis);
    static uint8_t spi_writeRegister(uint8_t address, uint32_t data, const AxisEnum &axis);
    static uint8_t spi_readStatus(const AxisEnum &axis);
  private:
    static uint8_t spi_readWriteByte(uint8_t data);
    static void spi_writeByte(uint8_t data);
};

class Trams: public Stepper, public TramsSPI {
  public:
    static void init();
    static void set_position(const long &a, const long &b, const long &c, const long &e);
    static void set_position(const AxisEnum &axis, const long &v);
    static void set_e_position(const long &e);
    static void calculate(void);
    static void isr();
    static void TMC5130_homing(const AxisEnum &axis);
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
  private:
    static FORCE_INLINE void discard_current_motion_block(void);
    static FORCE_INLINE motion_block_t *get_current_motion_block(void);
    static FORCE_INLINE bool motion_blocks_queued(void);
    static FORCE_INLINE motion_block_t *get_next_free_motion_block(void);
    static FORCE_INLINE void append_motion_block(void);
    static unsigned char blocks_in_motion_queue();
    static void set_directions();
    static void TMC5130_enableDriver(const AxisEnum &axis);
    static void TMC5130_disableDriver(const AxisEnum &axis);
    static void TMC5130_init(const AxisEnum &axis, uint8_t irun, uint8_t ihold, uint8_t stepper_direction, uint16_t sw_register);
};

class TramsEndstops: public Endstops, public TramsSPI {
  public:
    void M119();
  private:
};

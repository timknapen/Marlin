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
 * Conditionals_post.h
 * Defines that depend on configuration but are not editable.
 */

#ifndef CONDITIONALS_POST_H
#define CONDITIONALS_POST_H

#define AVR_ATmega2560_FAMILY_PLUS_70 ( \
     MB(BQ_ZUM_MEGA_3D)                 \
  || MB(MIGHTYBOARD_REVE)               \
  || MB(MINIRAMBO)                      \
  || MB(SCOOVO_X9H)                     \
)

#define IS_CARTESIAN 1

/**
 * Axis lengths and center
 */
#define X_MAX_LENGTH (X_MAX_POS - (X_MIN_POS))
#define Y_MAX_LENGTH (Y_MAX_POS - (Y_MIN_POS))

// Defined only if the sanity-check is bypassed
#ifndef X_BED_SIZE
  #define X_BED_SIZE X_MAX_LENGTH
#endif
#ifndef Y_BED_SIZE
  #define Y_BED_SIZE Y_MAX_LENGTH
#endif



// Define center values for future use
#if ENABLED(BED_CENTER_AT_0_0)
  #define X_CENTER 0
  #define Y_CENTER 0
#else
  #define X_CENTER ((X_BED_SIZE) / 2)
  #define Y_CENTER ((Y_BED_SIZE) / 2)
#endif

// Get the linear boundaries of the bed
#define X_MIN_BED (X_CENTER - (X_BED_SIZE) / 2)
#define X_MAX_BED (X_CENTER + (X_BED_SIZE) / 2)
#define Y_MIN_BED (Y_CENTER - (Y_BED_SIZE) / 2)
#define Y_MAX_BED (Y_CENTER + (Y_BED_SIZE) / 2)

/**
 * CoreXY, CoreXZ, and CoreYZ - and their reverse
 */
#define CORE_IS_XY (ENABLED(COREXY) || ENABLED(COREYX))
#define IS_CORE (CORE_IS_XY)
#if IS_CORE
  #if CORE_IS_XY
    #define CORE_AXIS_1 A_AXIS
    #define CORE_AXIS_2 B_AXIS
  #endif
  #if (ENABLED(COREYX))
    #define CORESIGN(n) (-(n))
  #endif
#endif


/**
 * Set the home position based on settings or manual overrides
 */
#ifdef MANUAL_X_HOME_POS
  #define X_HOME_POS MANUAL_X_HOME_POS
#elif ENABLED(BED_CENTER_AT_0_0)
  #if ENABLED(DELTA)
    #define X_HOME_POS 0
  #else
    #define X_HOME_POS ((X_BED_SIZE) * (X_HOME_DIR) * 0.5)
  #endif
#else
  #if ENABLED(DELTA)
    #define X_HOME_POS (X_MIN_POS + (X_BED_SIZE) * 0.5)
  #else
    #define X_HOME_POS (X_HOME_DIR < 0 ? X_MIN_POS : X_MAX_POS)
  #endif
#endif

#ifdef MANUAL_Y_HOME_POS
  #define Y_HOME_POS MANUAL_Y_HOME_POS
#elif ENABLED(BED_CENTER_AT_0_0)
  #if ENABLED(DELTA)
    #define Y_HOME_POS 0
  #else
    #define Y_HOME_POS ((Y_BED_SIZE) * (Y_HOME_DIR) * 0.5)
  #endif
#else
  #if ENABLED(DELTA)
    #define Y_HOME_POS (Y_MIN_POS + (Y_BED_SIZE) * 0.5)
  #else
    #define Y_HOME_POS (Y_HOME_DIR < 0 ? Y_MIN_POS : Y_MAX_POS)
  #endif
#endif


/**
 * Host keep alive
 */
#ifndef DEFAULT_KEEPALIVE_INTERVAL
  #define DEFAULT_KEEPALIVE_INTERVAL 2
#endif

#ifdef CPU_32_BIT
  /**
   * Hidden options for developer
   */
  // Double stepping starts at STEP_DOUBLER_FREQUENCY + 1, quad stepping starts at STEP_DOUBLER_FREQUENCY * 2 + 1
  #ifndef STEP_DOUBLER_FREQUENCY
    #if ENABLED(LIN_ADVANCE)
      #define STEP_DOUBLER_FREQUENCY 60000 // Hz
    #else
      #define STEP_DOUBLER_FREQUENCY 80000 // Hz
    #endif
  #endif
  // Disable double / quad stepping
  //#define DISABLE_MULTI_STEPPING
#endif

#ifdef CPU_32_BIT
	#define MAX_STEP_FREQUENCY (STEP_DOUBLER_FREQUENCY * 4) // Max step frequency for the Due is approx. 330kHz
#else
	#define MAX_STEP_FREQUENCY 40000 // Max step frequency for Ultimaker (5000 pps / half step)
#endif

// MS1 MS2 Stepper Driver Microstepping mode table
#define MICROSTEP1 LOW,LOW
#define MICROSTEP2 HIGH,LOW
#define MICROSTEP4 LOW,HIGH
#define MICROSTEP8 HIGH,HIGH
#ifdef __SAM3X8E__
    #define MICROSTEP16 HIGH,HIGH
#else
#define MICROSTEP16 HIGH,HIGH
#endif



/**
 * Set defaults for missing (newer) options
 */
#ifndef DISABLE_INACTIVE_X
  #define DISABLE_INACTIVE_X DISABLE_X
#endif
#ifndef DISABLE_INACTIVE_Y
  #define DISABLE_INACTIVE_Y DISABLE_Y
#endif


// Power Signal Control Definitions
// By default use ATX definition
#ifndef POWER_SUPPLY
  #define POWER_SUPPLY 1
#endif
#if (POWER_SUPPLY == 1)     // 1 = ATX
  #define PS_ON_AWAKE  LOW
  #define PS_ON_ASLEEP HIGH
#elif (POWER_SUPPLY == 2)   // 2 = X-Box 360 203W
  #define PS_ON_AWAKE  HIGH
  #define PS_ON_ASLEEP LOW
#endif
#define HAS_POWER_SWITCH (POWER_SUPPLY > 0 && PIN_EXISTS(PS_ON))

/**
 * X_DUAL_ENDSTOPS endstop reassignment
 */
#if ENABLED(X_DUAL_ENDSTOPS)
  #if X_HOME_DIR > 0
    #if X2_USE_ENDSTOP == _XMIN_
      #define X2_MAX_ENDSTOP_INVERTING X_MIN_ENDSTOP_INVERTING
      #define X2_MAX_PIN X_MIN_PIN
    #elif X2_USE_ENDSTOP == _XMAX_
      #define X2_MAX_ENDSTOP_INVERTING X_MAX_ENDSTOP_INVERTING
      #define X2_MAX_PIN X_MAX_PIN
    #elif X2_USE_ENDSTOP == _YMIN_
      #define X2_MAX_ENDSTOP_INVERTING Y_MIN_ENDSTOP_INVERTING
      #define X2_MAX_PIN Y_MIN_PIN
    #elif X2_USE_ENDSTOP == _YMAX_
      #define X2_MAX_ENDSTOP_INVERTING Y_MAX_ENDSTOP_INVERTING
      #define X2_MAX_PIN Y_MAX_PIN
    #else
      #define X2_MAX_ENDSTOP_INVERTING false
    #endif
    #define X2_MIN_ENDSTOP_INVERTING false
  #else
    #if X2_USE_ENDSTOP == _XMIN_
      #define X2_MIN_ENDSTOP_INVERTING X_MIN_ENDSTOP_INVERTING
      #define X2_MIN_PIN X_MIN_PIN
    #elif X2_USE_ENDSTOP == _XMAX_
      #define X2_MIN_ENDSTOP_INVERTING X_MAX_ENDSTOP_INVERTING
      #define X2_MIN_PIN X_MAX_PIN
    #elif X2_USE_ENDSTOP == _YMIN_
      #define X2_MIN_ENDSTOP_INVERTING Y_MIN_ENDSTOP_INVERTING
      #define X2_MIN_PIN Y_MIN_PIN
    #elif X2_USE_ENDSTOP == _YMAX_
      #define X2_MIN_ENDSTOP_INVERTING Y_MAX_ENDSTOP_INVERTING
      #define X2_MIN_PIN Y_MAX_PIN
    #else
      #define X2_MIN_ENDSTOP_INVERTING false
    #endif
    #define X2_MAX_ENDSTOP_INVERTING false
  #endif
#endif

// Is an endstop plug used for the X2 endstop?
#define IS_X2_ENDSTOP(A,M) (ENABLED(X_DUAL_ENDSTOPS) && X2_USE_ENDSTOP == _##A##M##_)

/**
 * Y_DUAL_ENDSTOPS endstop reassignment
 */
#if ENABLED(Y_DUAL_ENDSTOPS)
  #if Y_HOME_DIR > 0
    #if Y2_USE_ENDSTOP == _XMIN_
      #define Y2_MAX_ENDSTOP_INVERTING X_MIN_ENDSTOP_INVERTING
      #define Y2_MAX_PIN X_MIN_PIN
    #elif Y2_USE_ENDSTOP == _XMAX_
      #define Y2_MAX_ENDSTOP_INVERTING X_MAX_ENDSTOP_INVERTING
      #define Y2_MAX_PIN X_MAX_PIN
    #elif Y2_USE_ENDSTOP == _YMIN_
      #define Y2_MAX_ENDSTOP_INVERTING Y_MIN_ENDSTOP_INVERTING
      #define Y2_MAX_PIN Y_MIN_PIN
    #elif Y2_USE_ENDSTOP == _YMAX_
      #define Y2_MAX_ENDSTOP_INVERTING Y_MAX_ENDSTOP_INVERTING
      #define Y2_MAX_PIN Y_MAX_PIN
    #else
      #define Y2_MAX_ENDSTOP_INVERTING false
    #endif
    #define Y2_MIN_ENDSTOP_INVERTING false
  #else
    #if Y2_USE_ENDSTOP == _XMIN_
      #define Y2_MIN_ENDSTOP_INVERTING X_MIN_ENDSTOP_INVERTING
      #define Y2_MIN_PIN X_MIN_PIN
    #elif Y2_USE_ENDSTOP == _XMAX_
      #define Y2_MIN_ENDSTOP_INVERTING X_MAX_ENDSTOP_INVERTING
      #define Y2_MIN_PIN X_MAX_PIN
    #elif Y2_USE_ENDSTOP == _YMIN_
      #define Y2_MIN_ENDSTOP_INVERTING Y_MIN_ENDSTOP_INVERTING
      #define Y2_MIN_PIN Y_MIN_PIN
    #elif Y2_USE_ENDSTOP == _YMAX_
      #define Y2_MIN_ENDSTOP_INVERTING Y_MAX_ENDSTOP_INVERTING
      #define Y2_MIN_PIN Y_MAX_PIN
    #else
      #define Y2_MIN_ENDSTOP_INVERTING false
    #endif
    #define Y2_MAX_ENDSTOP_INVERTING false
  #endif
#endif

// Is an endstop plug used for the Y2 endstop or the bed probe?
#define IS_Y2_ENDSTOP(A,M) (ENABLED(Y_DUAL_ENDSTOPS) && Y2_USE_ENDSTOP == _##A##M##_)


/**
 * Set ENDSTOPPULLUPS for active endstop switches
 */
#if ENABLED(ENDSTOPPULLUPS)
  #if ENABLED(USE_XMAX_PLUG)
    #define ENDSTOPPULLUP_XMAX
  #endif
  #if ENABLED(USE_YMAX_PLUG)
    #define ENDSTOPPULLUP_YMAX
  #endif
  #if ENABLED(USE_XMIN_PLUG)
    #define ENDSTOPPULLUP_XMIN
  #endif
  #if ENABLED(USE_YMIN_PLUG)
    #define ENDSTOPPULLUP_YMIN
  #endif
#endif

/**
 * Shorthand for pin tests, used wherever needed
 */

// Steppers
#define HAS_X_ENABLE      (PIN_EXISTS(X_ENABLE))
#define HAS_X_DIR         (PIN_EXISTS(X_DIR))
#define HAS_X_STEP        (PIN_EXISTS(X_STEP))
#define HAS_X_MICROSTEPS  (PIN_EXISTS(X_MS1))

#define HAS_Y_ENABLE      (PIN_EXISTS(Y_ENABLE))
#define HAS_Y_DIR         (PIN_EXISTS(Y_DIR))
#define HAS_Y_STEP        (PIN_EXISTS(Y_STEP))
#define HAS_Y_MICROSTEPS  (PIN_EXISTS(Y_MS1))


// Endstops and bed probe
#define HAS_X_MIN (PIN_EXISTS(X_MIN) && !IS_X2_ENDSTOP(X,MIN) && !IS_Y2_ENDSTOP(X,MIN))
#define HAS_X_MAX (PIN_EXISTS(X_MAX) && !IS_X2_ENDSTOP(X,MAX) && !IS_Y2_ENDSTOP(X,MAX))
#define HAS_Y_MIN (PIN_EXISTS(Y_MIN) && !IS_X2_ENDSTOP(Y,MIN) && !IS_Y2_ENDSTOP(Y,MIN))
#define HAS_Y_MAX (PIN_EXISTS(Y_MAX) && !IS_X2_ENDSTOP(Y,MAX) && !IS_Y2_ENDSTOP(Y,MAX))


// Servos
#define HAS_SERVOS (defined(NUM_SERVOS) && NUM_SERVOS > 0)
#define HAS_SERVO_0 (PIN_EXISTS(SERVO0))
#define HAS_SERVO_1 (PIN_EXISTS(SERVO1))
#define HAS_SERVO_2 (PIN_EXISTS(SERVO2))
#define HAS_SERVO_3 (PIN_EXISTS(SERVO3))

// User Interface
#define HAS_HOME (PIN_EXISTS(HOME))
#define HAS_KILL (PIN_EXISTS(KILL))
#define HAS_SUICIDE (PIN_EXISTS(SUICIDE))

// Digital control
#define HAS_MICROSTEPS (HAS_X_MICROSTEPS || HAS_Y_MICROSTEPS )
#define HAS_STEPPER_RESET (PIN_EXISTS(STEPPER_RESET))
#define HAS_MOTOR_CURRENT_PWM (PIN_EXISTS(MOTOR_CURRENT_PWM_XY))

/**
 * Servos and probes
 */




// Stepper pulse duration, in cycles
#define STEP_PULSE_CYCLES ((MINIMUM_STEPPER_PULSE) * CYCLES_PER_MICROSECOND)
#ifdef CPU_32_BIT
  // Add additional delay for between direction signal and pulse signal of stepper
  #ifndef STEPPER_DIRECTION_DELAY
    #define STEPPER_DIRECTION_DELAY 0 // time in microseconds
  #endif
#endif

#ifndef __SAM3X8E__ //todo: hal: broken hal encapsulation
  #undef UI_VOLTAGE_LEVEL
  #undef RADDS_DISPLAY
  #undef MOTOR_CURRENT
#endif

// Updated G92 behavior shifts the workspace
#define HAS_POSITION_SHIFT DISABLED(NO_WORKSPACE_OFFSETS)
// The home offset also shifts the coordinate space
#define HAS_HOME_OFFSET (DISABLED(NO_WORKSPACE_OFFSETS) && DISABLED(DELTA))
// Either offset yields extra calculations on all moves
#define HAS_WORKSPACE_OFFSET (HAS_POSITION_SHIFT || HAS_HOME_OFFSET)
// M206 doesn't apply to DELTA
#define HAS_M206_COMMAND (HAS_HOME_OFFSET && DISABLED(DELTA))

// Add commands that need sub-codes to this list
#define USE_GCODE_SUBCODES ENABLED(G38_PROBE_TARGET) || ENABLED(CNC_COORDINATE_SYSTEMS)

// Use float instead of double. Needs profiling.
#if defined(ARDUINO_ARCH_SAM) && ENABLED(DELTA_FAST_SQRT)
  #undef ATAN2
  #undef FABS
  #undef POW
  #undef SQRT
  #undef CEIL
  #undef FLOOR
  #undef LROUND
  #undef FMOD
  #define ATAN2(y, x) atan2f(y, x)
  #define FABS(x) fabsf(x)
  #define POW(x, y) powf(x, y)
  #define SQRT(x) sqrtf(x)
  #define CEIL(x) ceilf(x)
  #define FLOOR(x) floorf(x)
  #define LROUND(x) lroundf(x)
  #define FMOD(x, y) fmodf(x, y)
#endif

#ifdef TEENSYDUINO
  #undef max
  #define max(a,b) ((a)>(b)?(a):(b))
  #undef min
  #define min(a,b) ((a)<(b)?(a):(b))

  #undef NOT_A_PIN    // Override Teensyduino legacy CapSense define work-around
  #define NOT_A_PIN 0 // For PINS_DEBUGGING
#endif

// Number of VFAT entries used. Each entry has 13 UTF-16 characters
#if ENABLED(SCROLL_LONG_FILENAMES)
  #define MAX_VFAT_ENTRIES (5)
#else
  #define MAX_VFAT_ENTRIES (2)
#endif


// Force SDCARD_SORT_ALPHA to be enabled for Graphical l c d on LPC1768
// because of a bug in the shared SPI implementation. (See #8122)
#if defined(TARGET_LPC1768) && ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
  #define SDCARD_SORT_ALPHA         // Keeps one directory level in RAM. Changing
                                    // directory levels still glitches the screen,
                                    // but the following LCD update cleans it up.
  #undef SDSORT_LIMIT
  #undef SDSORT_USES_RAM
  #undef SDSORT_USES_STACK
  #undef SDSORT_CACHE_NAMES
  #define SDSORT_LIMIT       256
  #define SDSORT_USES_RAM    true
  #define SDSORT_USES_STACK  false
  #define SDSORT_CACHE_NAMES true
  #ifndef FOLDER_SORTING
    #define FOLDER_SORTING     -1
  #endif
  #ifndef SDSORT_GCODE
    #define SDSORT_GCODE       false
  #endif
  #ifndef SDSORT_DYNAMIC_RAM
    #define SDSORT_DYNAMIC_RAM false
  #endif
  #ifndef SDSORT_CACHE_VFATS
    #define SDSORT_CACHE_VFATS 2
  #endif
#endif

// needs to be here so that we catch the above changes to our defines
#if ENABLED(SDCARD_SORT_ALPHA)
  #define HAS_FOLDER_SORTING (FOLDER_SORTING || ENABLED(SDSORT_GCODE))
#endif

#endif // CONDITIONALS_POST_H

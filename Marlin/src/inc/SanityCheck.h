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
 * SanityCheck.h
 *
 * Test configuration values for errors at compile-time.
 */

#ifndef _SANITYCHECK_H_
#define _SANITYCHECK_H_

/**
 * Require gcc 4.7 or newer (first included with Arduino 1.6.8) for C++11 features.
 */
#if __cplusplus < 201103L
  #error "Marlin requires C++11 support (gcc >= 4.7, Arduino IDE >= 1.6.8). Please upgrade your toolchain."
#endif

/**
 * We try our best to include sanity checks for all changed configuration
 * directives because users have a tendency to use outdated config files with
 * the bleeding-edge source code, but sometimes this is not enough. This check
 * forces a minimum config file revision. Otherwise Marlin will not build.
 */
#if ! defined(CONFIGURATION_H_VERSION) || CONFIGURATION_H_VERSION < REQUIRED_CONFIGURATION_H_VERSION
  #error "You are using an old Configuration.h file, update it before building Marlin."
#endif

#if ! defined(CONFIGURATION_ADV_H_VERSION) || CONFIGURATION_ADV_H_VERSION < REQUIRED_CONFIGURATION_ADV_H_VERSION
  #error "You are using an old Configuration_adv.h file, update it before building Marlin."
#endif

/**
 * Warnings for old configurations
 */
#if !defined(X_BED_SIZE) || !defined(Y_BED_SIZE)
  #error "X_BED_SIZE and Y_BED_SIZE are now required! Please update your configuration."
#elif WATCH_TEMP_PERIOD > 500
  #error "WATCH_TEMP_PERIOD now uses seconds instead of milliseconds."
#elif defined(X_HOME_RETRACT_MM)
  #error "[XY]_HOME_RETRACT_MM settings have been renamed [XY]_HOME_BUMP_MM."
#elif defined(SDCARDDETECTINVERTED)
  #error "SDCARDDETECTINVERTED is now SD_DETECT_INVERTED. Please update your configuration."
#elif defined(BTENABLED)
  #error "BTENABLED is now BLUETOOTH. Please update your configuration."
#elif defined(CUSTOM_MENDEL_NAME)
  #error "CUSTOM_MENDEL_NAME is now CUSTOM_MACHINE_NAME. Please update your configuration."
#elif defined(HAS_AUTOMATIC_VERSIONING)
  #error "HAS_AUTOMATIC_VERSIONING is now USE_AUTOMATIC_VERSIONING. Please update your configuration."
#elif defined(SDSLOW)
  #error "SDSLOW deprecated. Set SPI_SPEED to SPI_HALF_SPEED instead."
#elif defined(SDEXTRASLOW)
  #error "SDEXTRASLOW deprecated. Set SPI_SPEED to SPI_QUARTER_SPEED instead."
#elif defined(DISABLE_MAX_ENDSTOPS) || defined(DISABLE_MIN_ENDSTOPS)
  #error "DISABLE_MAX_ENDSTOPS and DISABLE_MIN_ENDSTOPS deprecated. Use individual USE_*_PLUG options instead."
#elif defined(LANGUAGE_INCLUDE)
  #error "LANGUAGE_INCLUDE has been replaced by LCD_LANGUAGE. Please update your configuration."
#elif defined(X_ENDSTOP_SERVO_NR) || defined(Y_ENDSTOP_SERVO_NR)
  #error "X_ENDSTOP_SERVO_NR and Y_ENDSTOP_SERVO_NR are deprecated and should be removed."
#elif defined(DEFAULT_XYJERK)
  #error "DEFAULT_XYJERK is deprecated. Use DEFAULT_XJERK and DEFAULT_YJERK instead."
#elif defined(SERVO_DEACTIVATION_DELAY)
  #error "SERVO_DEACTIVATION_DELAY is deprecated. Use SERVO_DELAY instead."
#elif defined(MANUAL_HOME_POSITIONS)
  #error "MANUAL_HOME_POSITIONS is deprecated. Set MANUAL_[XY]_HOME_POS as-needed instead."
#elif !defined(MIN_STEPS_PER_SEGMENT)
  #error Please replace "const int dropsegments" with "#define MIN_STEPS_PER_SEGMENT" (and increase by 1) in Configuration_adv.h.
#elif defined(SDCARDDETECT)
  #error "SDCARDDETECT is now SD_DETECT_PIN. Please update your pins definitions."
#endif

/**
 * Marlin release, version and default string
 */
#ifndef SHORT_BUILD_VERSION
  #error "SHORT_BUILD_VERSION must be specified."
#elif !defined(DETAILED_BUILD_VERSION)
  #error "BUILD_VERSION must be specified."
#elif !defined(STRING_DISTRIBUTION_DATE)
  #error "STRING_DISTRIBUTION_DATE must be specified."
#elif !defined(PROTOCOL_VERSION)
  #error "PROTOCOL_VERSION must be specified."
#elif !defined(MACHINE_NAME)
  #error "MACHINE_NAME must be specified."
#elif !defined(SOURCE_CODE_URL)
  #error "SOURCE_CODE_URL must be specified."
#elif !defined(DEFAULT_MACHINE_UUID)
  #error "DEFAULT_MACHINE_UUID must be specified."
#elif !defined(WEBSITE_URL)
  #error "WEBSITE_URL must be specified."
#endif

/**
 * Serial
 */
#ifndef USBCON
  #if ENABLED(SERIAL_XON_XOFF) && RX_BUFFER_SIZE < 1024
    #error "SERIAL_XON_XOFF requires RX_BUFFER_SIZE >= 1024 for reliable transfers without drops."
  #endif

  #if !IS_POWER_OF_2(RX_BUFFER_SIZE) || RX_BUFFER_SIZE < 2
    #error "RX_BUFFER_SIZE must be a power of 2 greater than 1."
  #endif

  #if TX_BUFFER_SIZE && (TX_BUFFER_SIZE < 2 || TX_BUFFER_SIZE > 256 || !IS_POWER_OF_2(TX_BUFFER_SIZE))
    #error "TX_BUFFER_SIZE must be 0, a power of 2 greater than 1, and no greater than 256."
  #endif
#elif ENABLED(SERIAL_XON_XOFF)
  #error "SERIAL_XON_XOFF is not supported on USB-native AVR devices."
#endif


/**
 * Validate that the bed size fits
 */
static_assert(X_MAX_LENGTH >= X_BED_SIZE && Y_MAX_LENGTH >= Y_BED_SIZE,
  "Movement bounds ([XY]_MIN_POS, [XY]_MAX_POS) are too narrow to contain [XY]_BED_SIZE.");


/**
 * SD File Sorting
 */
#if ENABLED(SDCARD_SORT_ALPHA)
  #if SDSORT_LIMIT > 256
    #error "SDSORT_LIMIT must be 256 or smaller."
  #elif SDSORT_LIMIT < 10
    #error "SDSORT_LIMIT should be greater than 9 to be useful."
  #elif DISABLED(SDSORT_USES_RAM)
    #if ENABLED(SDSORT_DYNAMIC_RAM)
      #error "SDSORT_DYNAMIC_RAM requires SDSORT_USES_RAM (which reads the directory into RAM)."
    #elif ENABLED(SDSORT_CACHE_NAMES)
      #error "SDSORT_CACHE_NAMES requires SDSORT_USES_RAM (which reads the directory into RAM)."
    #endif
  #endif

  #if ENABLED(SDSORT_CACHE_NAMES) && DISABLED(SDSORT_DYNAMIC_RAM)
    #if SDSORT_CACHE_VFATS < 2
      #error "SDSORT_CACHE_VFATS must be 2 or greater!"
    #elif SDSORT_CACHE_VFATS > MAX_VFAT_ENTRIES
      #undef SDSORT_CACHE_VFATS
      #define SDSORT_CACHE_VFATS MAX_VFAT_ENTRIES
      #warning "SDSORT_CACHE_VFATS was reduced to MAX_VFAT_ENTRIES!"
    #endif
  #endif
#endif



/**
 * Limited number of servos
 */
#if NUM_SERVOS > 4
  #error "The maximum number of SERVOS in Marlin is 4."
#endif


/**
 * Kinematics
 */

/**
 * Allow only one kinematic type to be defined
 */
static_assert(1 >= 0
  #if ENABLED(COREXY)
    + 1
  #endif
  #if ENABLED(COREYX)
    + 1
  #endif
  , "Please enable only one of COREXY, COREYX."
);


/**
 * Homing Bump
 */
#if X_HOME_BUMP_MM < 0 || Y_HOME_BUMP_MM < 0
  #error "[XY]_HOME_BUMP_MM must be greater than or equal to 0."
#endif

/**
 * Make sure DISABLE_[XY] compatible with selected homing options
 */
#if ENABLED(DISABLE_X) || ENABLED(DISABLE_Y)
  #if ENABLED(HOME_AFTER_DEACTIVATE)
    #error "DISABLE_[XY] not compatible with HOME_AFTER_DEACTIVATE."
  #endif
#endif // DISABLE_[XY]


/**
 * Endstop Tests
 */

#define _PLUG_UNUSED_TEST(AXIS,PLUG) (DISABLED(USE_##PLUG##MIN_PLUG) && DISABLED(USE_##PLUG##MAX_PLUG))
#define _AXIS_PLUG_UNUSED_TEST(AXIS) (_PLUG_UNUSED_TEST(AXIS,X) && _PLUG_UNUSED_TEST(AXIS,Y))

// At least 2 endstop plugs must be used
#if _AXIS_PLUG_UNUSED_TEST(X)
  #error "You must enable USE_XMIN_PLUG or USE_XMAX_PLUG."
#endif
#if _AXIS_PLUG_UNUSED_TEST(Y)
  #error "You must enable USE_YMIN_PLUG or USE_YMAX_PLUG."
#endif

// Cartesian use 2 homing endstops
  #if X_HOME_DIR < 0 && DISABLED(USE_XMIN_PLUG)
    #error "Enable USE_XMIN_PLUG when homing X to MIN."
  #elif X_HOME_DIR > 0 && DISABLED(USE_XMAX_PLUG)
    #error "Enable USE_XMAX_PLUG when homing X to MAX."
  #elif Y_HOME_DIR < 0 && DISABLED(USE_YMIN_PLUG)
    #error "Enable USE_YMIN_PLUG when homing Y to MIN."
  #elif Y_HOME_DIR > 0 && DISABLED(USE_YMAX_PLUG)
    #error "Enable USE_YMAX_PLUG when homing Y to MAX."
  #endif


/**
 * emergency-command parser
 */
#if ENABLED(EMERGENCY_PARSER) && defined(USBCON)
  #error "EMERGENCY_PARSER does not work on boards with AT90USB processors (USBCON)."
#endif


/**
 * Make sure HAVE_TMCDRIVER is warranted
 */
#if ENABLED(HAVE_TMCDRIVER) && !( \
         ENABLED(  X_IS_TMC ) \
      || ENABLED(  Y_IS_TMC ) \
)
  #error "HAVE_TMCDRIVER requires at least one TMC stepper to be set."
#endif

/**
 * Make sure HAVE_TMC2130 is warranted
 */
#if ENABLED(HAVE_TMC2130)
  #if !( ENABLED(  X_IS_TMC2130 ) \
      || ENABLED(  Y_IS_TMC2130 ) \
)
    #error "HAVE_TMC2130 requires at least one TMC2130 stepper to be set."
  #elif ENABLED(HYBRID_THRESHOLD) && DISABLED(STEALTHCHOP)
    #error "Enable STEALTHCHOP to use HYBRID_THRESHOLD."
  #endif
#endif



/**
 * Require 2 elements in per-axis initializers
 */
constexpr float sanity_arr_1[] = DEFAULT_AXIS_STEPS_PER_UNIT,
                sanity_arr_2[] = DEFAULT_MAX_FEEDRATE,
                sanity_arr_3[] = DEFAULT_MAX_ACCELERATION;
static_assert(COUNT(sanity_arr_1) == XY, "DEFAULT_AXIS_STEPS_PER_UNIT requires 2 elements.");
static_assert(COUNT(sanity_arr_2) == XY, "DEFAULT_MAX_FEEDRATE requires 2 elements.");
static_assert(COUNT(sanity_arr_3) == XY, "DEFAULT_MAX_ACCELERATION requires 2 elements.");
static_assert(COUNT(sanity_arr_1) <= XY, "DEFAULT_AXIS_STEPS_PER_UNIT has too many elements.");
static_assert(COUNT(sanity_arr_2) <= XY, "DEFAULT_MAX_FEEDRATE has too many elements.");
static_assert(COUNT(sanity_arr_3) <= XY, "DEFAULT_MAX_ACCELERATION has too many elements.");


#if !BLOCK_BUFFER_SIZE || !IS_POWER_OF_2(BLOCK_BUFFER_SIZE)
  #error "BLOCK_BUFFER_SIZE must be a power of 2."
#endif


#endif // _SANITYCHECK_H_

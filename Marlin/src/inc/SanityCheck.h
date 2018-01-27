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
#elif DISABLED(THERMAL_PROTECTION_HOTENDS) && (defined(WATCH_TEMP_PERIOD) || defined(THERMAL_PROTECTION_PERIOD))
  #error "Thermal Runaway Protection for hotends is now enabled with THERMAL_PROTECTION_HOTENDS."
#elif DISABLED(THERMAL_PROTECTION_BED) && defined(THERMAL_PROTECTION_BED_PERIOD)
  #error "Thermal Runaway Protection for the bed is now enabled with THERMAL_PROTECTION_BED."
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
#elif defined(FILAMENT_SENSOR)
  #error "FILAMENT_SENSOR is deprecated. Use FILAMENT_WIDTH_SENSOR instead."
#elif defined(DISABLE_MAX_ENDSTOPS) || defined(DISABLE_MIN_ENDSTOPS)
  #error "DISABLE_MAX_ENDSTOPS and DISABLE_MIN_ENDSTOPS deprecated. Use individual USE_*_PLUG options instead."
#elif defined(LANGUAGE_INCLUDE)
  #error "LANGUAGE_INCLUDE has been replaced by LCD_LANGUAGE. Please update your configuration."
#elif defined(SERVO_ENDSTOP_ANGLES)
  #error "SERVO_ENDSTOP_ANGLES is deprecated. Use Z_SERVO_ANGLES instead."
#elif defined(X_ENDSTOP_SERVO_NR) || defined(Y_ENDSTOP_SERVO_NR)
  #error "X_ENDSTOP_SERVO_NR and Y_ENDSTOP_SERVO_NR are deprecated and should be removed."
#elif defined(DEFAULT_XYJERK)
  #error "DEFAULT_XYJERK is deprecated. Use DEFAULT_XJERK and DEFAULT_YJERK instead."
#elif defined(XY_TRAVEL_SPEED)
  #error "XY_TRAVEL_SPEED is deprecated. Use XY_PROBE_SPEED instead."
#elif defined(PROBE_SERVO_DEACTIVATION_DELAY)
  #error "PROBE_SERVO_DEACTIVATION_DELAY is deprecated. Use DEACTIVATE_SERVOS_AFTER_MOVE instead."
#elif defined(SERVO_DEACTIVATION_DELAY)
  #error "SERVO_DEACTIVATION_DELAY is deprecated. Use SERVO_DELAY instead."
#elif ENABLED(FILAMENTCHANGEENABLE)
  #error "FILAMENTCHANGEENABLE is now ADVANCED_PAUSE_FEATURE. Please update your configuration."
#elif ENABLED(FILAMENT_CHANGE_FEATURE)
  #error "FILAMENT_CHANGE_FEATURE is now ADVANCED_PAUSE_FEATURE. Please update your configuration."
#elif ENABLED(FILAMENT_CHANGE_X_POS)
  #error "FILAMENT_CHANGE_X_POS is now PAUSE_PARK_X_POS. Please update your configuration."
#elif ENABLED(FILAMENT_CHANGE_Y_POS)
  #error "FILAMENT_CHANGE_Y_POS is now PAUSE_PARK_Y_POS. Please update your configuration."
#elif ENABLED(FILAMENT_CHANGE_Z_ADD)
  #error "FILAMENT_CHANGE_Z_ADD is now PAUSE_PARK_Z_ADD. Please update your configuration."
#elif ENABLED(FILAMENT_CHANGE_XY_FEEDRATE)
  #error "FILAMENT_CHANGE_XY_FEEDRATE is now PAUSE_PARK_XY_FEEDRATE. Please update your configuration."
#elif defined(MANUAL_HOME_POSITIONS)
  #error "MANUAL_HOME_POSITIONS is deprecated. Set MANUAL_[XY]_HOME_POS as-needed instead."
#elif defined(PID_ADD_EXTRUSION_RATE)
  #error "PID_ADD_EXTRUSION_RATE is now PID_EXTRUSION_SCALING and is DISABLED by default. Are you sure you want to use this option? Please update your configuration."
#elif defined(Z_RAISE_BEFORE_HOMING)
  #error "Z_RAISE_BEFORE_HOMING is now Z_HOMING_HEIGHT. Please update your configuration."
#elif defined(MIN_Z_HEIGHT_FOR_HOMING)
  #error "MIN_Z_HEIGHT_FOR_HOMING is now Z_HOMING_HEIGHT. Please update your configuration."
#elif defined(Z_RAISE_BEFORE_PROBING) || defined(Z_RAISE_AFTER_PROBING)
  #error "Z_RAISE_(BEFORE|AFTER)_PROBING are deprecated. Use Z_CLEARANCE_DEPLOY_PROBE instead."
#elif defined(Z_RAISE_PROBE_DEPLOY_STOW) || defined(Z_RAISE_BETWEEN_PROBINGS)
  #error "Z_RAISE_PROBE_DEPLOY_STOW and Z_RAISE_BETWEEN_PROBINGS are now Z_CLEARANCE_DEPLOY_PROBE and Z_CLEARANCE_BETWEEN_PROBES. Please update your configuration."
#elif defined(Z_PROBE_DEPLOY_HEIGHT) || defined(Z_PROBE_TRAVEL_HEIGHT)
  #error "Z_PROBE_DEPLOY_HEIGHT and Z_PROBE_TRAVEL_HEIGHT are now Z_CLEARANCE_DEPLOY_PROBE and Z_CLEARANCE_BETWEEN_PROBES. Please update your configuration."
#elif defined(MANUAL_BED_LEVELING)
  #error "MANUAL_BED_LEVELING is now LCD_BED_LEVELING. Please update your configuration."
#elif defined(MESH_HOME_SEARCH_Z)
  #error "MESH_HOME_SEARCH_Z is now LCD_PROBE_Z_RANGE. Please update your configuration."
#elif defined(MANUAL_PROBE_Z_RANGE)
  #error "MANUAL_PROBE_Z_RANGE is now LCD_PROBE_Z_RANGE. Please update your configuration."
#elif !defined(MIN_STEPS_PER_SEGMENT)
  #error Please replace "const int dropsegments" with "#define MIN_STEPS_PER_SEGMENT" (and increase by 1) in Configuration_adv.h.
#elif defined(PREVENT_DANGEROUS_EXTRUDE)
  #error "PREVENT_DANGEROUS_EXTRUDE is now PREVENT_COLD_EXTRUSION. Please update your configuration."
#elif defined(ENABLE_AUTO_BED_LEVELING)
  #error "ENABLE_AUTO_BED_LEVELING is deprecated. Specify AUTO_BED_LEVELING_LINEAR, AUTO_BED_LEVELING_BILINEAR, or AUTO_BED_LEVELING_3POINT."
#elif defined(AUTO_BED_LEVELING_FEATURE)
  #error "AUTO_BED_LEVELING_FEATURE is deprecated. Specify AUTO_BED_LEVELING_LINEAR, AUTO_BED_LEVELING_BILINEAR, or AUTO_BED_LEVELING_3POINT."
#elif defined(ABL_GRID_POINTS)
  #error "ABL_GRID_POINTS is now GRID_MAX_POINTS_X and GRID_MAX_POINTS_Y. Please update your configuration."
#elif defined(ABL_GRID_POINTS_X) || defined(ABL_GRID_POINTS_Y)
  #error "ABL_GRID_POINTS_[XY] is now GRID_MAX_POINTS_[XY]. Please update your configuration."
#elif defined(ABL_GRID_MAX_POINTS_X) || defined(ABL_GRID_MAX_POINTS_Y)
  #error "ABL_GRID_MAX_POINTS_[XY] is now GRID_MAX_POINTS_[XY]. Please update your configuration."
#elif defined(MESH_NUM_X_POINTS) || defined(MESH_NUM_Y_POINTS)
  #error "MESH_NUM_[XY]_POINTS is now GRID_MAX_POINTS_[XY]. Please update your configuration."
#elif defined(UBL_MESH_NUM_X_POINTS) || defined(UBL_MESH_NUM_Y_POINTS)
  #error "UBL_MESH_NUM_[XY]_POINTS is now GRID_MAX_POINTS_[XY]. Please update your configuration."
#elif defined(UBL_G26_MESH_VALIDATION)
  #error "UBL_G26_MESH_VALIDATION is now G26_MESH_VALIDATION. Please update your configuration."
#elif defined(UBL_MESH_EDIT_ENABLED)
  #error "UBL_MESH_EDIT_ENABLED is now G26_MESH_VALIDATION. Please update your configuration."
#elif defined(UBL_MESH_EDITING)
  #error "UBL_MESH_EDITING is now G26_MESH_VALIDATION. Please update your configuration."
#elif defined(BLTOUCH_HEATERS_OFF)
  #error "BLTOUCH_HEATERS_OFF is now PROBING_HEATERS_OFF. Please update your configuration."
#elif defined(BEEPER)
  #error "BEEPER is now BEEPER_PIN. Please update your pins definitions."
#elif defined(SDCARDDETECT)
  #error "SDCARDDETECT is now SD_DETECT_PIN. Please update your pins definitions."
#elif defined(STAT_LED_RED) || defined(STAT_LED_BLUE)
  #error "STAT_LED_RED/STAT_LED_BLUE are now STAT_LED_RED_PIN/STAT_LED_BLUE_PIN. Please update your pins definitions."
#elif defined(LCD_PIN_BL)
  #error "LCD_PIN_BL is now LCD_BACKLIGHT_PIN. Please update your pins definitions."
#elif defined(LCD_PIN_RESET)
  #error "LCD_PIN_RESET is now LCD_RESET_PIN. Please update your pins definitions."
#elif defined(min_software_endstops) || defined(max_software_endstops)
  #error "(min|max)_software_endstops are now (MIN|MAX)_SOFTWARE_ENDSTOPS. Please update your configuration."
#elif ENABLED(Z_PROBE_SLED) && defined(SLED_PIN)
  #error "Replace SLED_PIN with SOL1_PIN (applies to both Z_PROBE_SLED and SOLENOID_PROBE)."
#elif defined(CONTROLLERFAN_PIN)
  #error "CONTROLLERFAN_PIN is now CONTROLLER_FAN_PIN, enabled with USE_CONTROLLER_FAN. Please update your Configuration_adv.h."
#elif defined(MIN_RETRACT)
  #error "MIN_RETRACT is now MIN_AUTORETRACT and MAX_AUTORETRACT. Please update your Configuration_adv.h."
#elif defined(ADVANCE)
  #error "ADVANCE was removed in Marlin 1.1.6. Please use LIN_ADVANCE."
#elif defined(NEOPIXEL_RGBW_LED)
  #error "NEOPIXEL_RGBW_LED is now NEOPIXEL_LED. Please update your configuration."
#elif defined(UBL_MESH_INSET)
  #error "UBL_MESH_INSET is now just MESH_INSET. Please update your configuration."
#elif defined(UBL_MESH_MIN_X) || defined(UBL_MESH_MIN_Y) || defined(UBL_MESH_MAX_X)  || defined(UBL_MESH_MAX_Y)
  #error "UBL_MESH_(MIN|MAX)_[XY] is now just MESH_(MIN|MAX)_[XY]. Please update your configuration."
#elif defined(ENABLE_MESH_EDIT_GFX_OVERLAY)
  #error "ENABLE_MESH_EDIT_GFX_OVERLAY is now MESH_EDIT_GFX_OVERLAY. Please update your configuration."
#elif defined(BABYSTEP_ZPROBE_GFX_REVERSE)
  #error "BABYSTEP_ZPROBE_GFX_REVERSE is now set by OVERLAY_GFX_REVERSE. Please update your configurations."
#elif defined(UBL_GRANULAR_SEGMENTATION_FOR_CARTESIAN)
  #error "UBL_GRANULAR_SEGMENTATION_FOR_CARTESIAN is now SEGMENT_LEVELED_MOVES. Please update your configuration."
#elif HAS_PID_HEATING && (defined(K1) || !defined(PID_K1))
  #error "K1 is now PID_K1. Please update your configuration."
#elif defined(PROBE_DOUBLE_TOUCH)
  #error "PROBE_DOUBLE_TOUCH is now MULTIPLE_PROBING. Please update your configuration."
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
 * Dual Stepper Drivers
 */
#if ENABLED(X_DUAL_STEPPER_DRIVERS) && ENABLED(DUAL_X_CARRIAGE)
  #error "DUAL_X_CARRIAGE is not compatible with X_DUAL_STEPPER_DRIVERS."
#elif ENABLED(X_DUAL_STEPPER_DRIVERS) && (!HAS_X2_ENABLE || !HAS_X2_STEP || !HAS_X2_DIR)
  #error "X_DUAL_STEPPER_DRIVERS requires X2 pins (and an extra E plug)."
#elif ENABLED(Y_DUAL_STEPPER_DRIVERS) && (!HAS_Y2_ENABLE || !HAS_Y2_STEP || !HAS_Y2_DIR)
  #error "Y_DUAL_STEPPER_DRIVERS requires Y2 pins (and an extra E plug)."
#elif ENABLED(Z_DUAL_STEPPER_DRIVERS) && (!HAS_Z2_ENABLE || !HAS_Z2_STEP || !HAS_Z2_DIR)
  #error "Z_DUAL_STEPPER_DRIVERS requires Z2 pins (and an extra E plug)."
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
 * Servo deactivation depends on servo endstops, switching nozzle, or switching extruder
 */
#if ENABLED(DEACTIVATE_SERVOS_AFTER_MOVE) && !HAS_Z_SERVO_ENDSTOP && !defined(SWITCHING_NOZZLE_SERVO_NR) && !defined(SWITCHING_EXTRUDER_SERVO_NR)
  #error "Z_ENDSTOP_SERVO_NR, switching nozzle, or switching extruder is required for DEACTIVATE_SERVOS_AFTER_MOVE."
#endif

/**
 * Required LCD language
 */
#if DISABLED(DOGLCD) && ENABLED(ULTRA_LCD) && !defined(DISPLAY_CHARSET_HD44780)
  #error "You must set DISPLAY_CHARSET_HD44780 to JAPANESE, WESTERN or CYRILLIC for your LCD controller."
#endif


/**
 * Kinematics
 */

/**
 * Allow only one kinematic type to be defined
 */
static_assert(1 >= 0
  #if ENABLED(DELTA)
    + 1
  #endif
  #if ENABLED(COREXY)
    + 1
  #endif
  #if ENABLED(COREXZ)
    + 1
  #endif
  #if ENABLED(COREYZ)
    + 1
  #endif
  #if ENABLED(COREYX)
    + 1
  #endif
  #if ENABLED(COREZX)
    + 1
  #endif
  #if ENABLED(COREZY)
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

#define _PLUG_UNUSED_TEST(AXIS,PLUG) (DISABLED(USE_##PLUG##MIN_PLUG) && DISABLED(USE_##PLUG##MAX_PLUG) && !(ENABLED(AXIS##_DUAL_ENDSTOPS) && WITHIN(AXIS##2_USE_ENDSTOP, _##PLUG##MAX_, _##PLUG##MIN_)))
#define _AXIS_PLUG_UNUSED_TEST(AXIS) (_PLUG_UNUSED_TEST(AXIS,X) && _PLUG_UNUSED_TEST(AXIS,Y) && _PLUG_UNUSED_TEST(AXIS,Z))

// At least 3 endstop plugs must be used
#if _AXIS_PLUG_UNUSED_TEST(X)
  #error "You must enable USE_XMIN_PLUG or USE_XMAX_PLUG."
#endif
#if _AXIS_PLUG_UNUSED_TEST(Y)
  #error "You must enable USE_YMIN_PLUG or USE_YMAX_PLUG."
#endif

// Cartesian use 3 homing endstops
  #if X_HOME_DIR < 0 && DISABLED(USE_XMIN_PLUG)
    #error "Enable USE_XMIN_PLUG when homing X to MIN."
  #elif X_HOME_DIR > 0 && DISABLED(USE_XMAX_PLUG)
    #error "Enable USE_XMAX_PLUG when homing X to MAX."
  #elif Y_HOME_DIR < 0 && DISABLED(USE_YMIN_PLUG)
    #error "Enable USE_YMIN_PLUG when homing Y to MIN."
  #elif Y_HOME_DIR > 0 && DISABLED(USE_YMAX_PLUG)
    #error "Enable USE_YMAX_PLUG when homing Y to MAX."
  #endif

// Dual endstops requirements
#if ENABLED(X_DUAL_ENDSTOPS)
  #if !X2_USE_ENDSTOP
    #error "You must set X2_USE_ENDSTOP with X_DUAL_ENDSTOPS."
  #elif X2_USE_ENDSTOP == _X_MIN_ && DISABLED(USE_XMIN_PLUG)
    #error "USE_XMIN_PLUG is required when X2_USE_ENDSTOP is _X_MIN_."
  #elif X2_USE_ENDSTOP == _X_MAX_ && DISABLED(USE_XMAX_PLUG)
    #error "USE_XMAX_PLUG is required when X2_USE_ENDSTOP is _X_MAX_."
  #elif X2_USE_ENDSTOP == _Y_MIN_ && DISABLED(USE_YMIN_PLUG)
    #error "USE_YMIN_PLUG is required when X2_USE_ENDSTOP is _Y_MIN_."
  #elif X2_USE_ENDSTOP == _Y_MAX_ && DISABLED(USE_YMAX_PLUG)
    #error "USE_YMAX_PLUG is required when X2_USE_ENDSTOP is _Y_MAX_."
  #elif X2_USE_ENDSTOP == _Z_MIN_ && DISABLED(USE_ZMIN_PLUG)
    #error "USE_ZMIN_PLUG is required when X2_USE_ENDSTOP is _Z_MIN_."
  #elif X2_USE_ENDSTOP == _Z_MAX_ && DISABLED(USE_ZMAX_PLUG)
    #error "USE_ZMAX_PLUG is required when X2_USE_ENDSTOP is _Z_MAX_."
  #elif !HAS_X2_MIN && !HAS_X2_MAX
    #error "X2_USE_ENDSTOP has been assigned to a nonexistent endstop!"
  #endif
#endif
#if ENABLED(Y_DUAL_ENDSTOPS)
  #if !Y2_USE_ENDSTOP
    #error "You must set Y2_USE_ENDSTOP with Y_DUAL_ENDSTOPS."
  #elif Y2_USE_ENDSTOP == _X_MIN_ && DISABLED(USE_XMIN_PLUG)
    #error "USE_XMIN_PLUG is required when Y2_USE_ENDSTOP is _X_MIN_."
  #elif Y2_USE_ENDSTOP == _X_MAX_ && DISABLED(USE_XMAX_PLUG)
    #error "USE_XMAX_PLUG is required when Y2_USE_ENDSTOP is _X_MAX_."
  #elif Y2_USE_ENDSTOP == _Y_MIN_ && DISABLED(USE_YMIN_PLUG)
    #error "USE_YMIN_PLUG is required when Y2_USE_ENDSTOP is _Y_MIN_."
  #elif Y2_USE_ENDSTOP == _Y_MAX_ && DISABLED(USE_YMAX_PLUG)
    #error "USE_YMAX_PLUG is required when Y2_USE_ENDSTOP is _Y_MAX_."
  #elif Y2_USE_ENDSTOP == _Z_MIN_ && DISABLED(USE_ZMIN_PLUG)
    #error "USE_ZMIN_PLUG is required when Y2_USE_ENDSTOP is _Z_MIN_."
  #elif Y2_USE_ENDSTOP == _Z_MAX_ && DISABLED(USE_ZMAX_PLUG)
    #error "USE_ZMAX_PLUG is required when Y2_USE_ENDSTOP is _Z_MAX_."
  #elif !HAS_Y2_MIN && !HAS_Y2_MAX
    #error "Y2_USE_ENDSTOP has been assigned to a nonexistent endstop!"
  #endif
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

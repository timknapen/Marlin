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
 * gcode.h - Temporary container for all gcode handlers
 */

/**
 * -----------------
 * G-Codes in Marlin
 * -----------------
 *
 * Helpful G-code references:
 *  - http://linuxcnc.org/handbook/gcode/g-code.html
 *  - http://objects.reprap.org/wiki/Mendel_User_Manual:_RepRapGCodes
 *
 * Help to document Marlin's G-codes online:
 *  - http://reprap.org/wiki/G-code
 *  - https://github.com/MarlinFirmware/MarlinDocumentation
 *
 * -----------------
 *
 * "G" Codes
 *
 * G0   -> G1
 * G1   - Coordinated Movement X Y
 * G2   - CW ARC
 * G3   - CCW ARC
 * G4   - Dwell S<seconds> or P<milliseconds>
 * G5   - Cubic B-spline with XY destination and IJPQ offsets
 * G20  - Set input units to inches (Requires INCH_MODE_SUPPORT)
 * G21  - Set input units to millimeters (Requires INCH_MODE_SUPPORT)
 * G28  - Home one or more axes
 * G90  - Use Absolute Coordinates
 * G91  - Use Relative Coordinates
 * G92  - Set current position to coordinates given
 *
 * "M" Codes
 *
 * M0   - Unconditional stop - Wait for user to press a button on the LCD (Only if ULTRA_LCD is enabled)
 * M1   -> M0
 * M17  - Enable/Power all stepper motors
 * M18  - Disable all stepper motors; same as M84
 * M20  - List SD card. (Requires SDSUPPORT)
 * M21  - Init SD card. (Requires SDSUPPORT)
 * M22  - Release SD card. (Requires SDSUPPORT)
 * M23  - Select SD file: "M23 /path/file.gco". (Requires SDSUPPORT)
 * M24  - Start/resume SD print. (Requires SDSUPPORT)
 * M25  - Pause SD print. (Requires SDSUPPORT)
 * M26  - Set SD position in bytes: "M26 S12345". (Requires SDSUPPORT)
 * M27  - Report SD print status. (Requires SDSUPPORT)
 * M28  - Start SD write: "M28 /path/file.gco". (Requires SDSUPPORT)
 * M29  - Stop SD write. (Requires SDSUPPORT)
 * M30  - Delete file from SD: "M30 /path/file.gco"
 * M31  - Report time since last M109 or SD card start to serial.
 * M32  - Select file and start SD print: "M32 [S<bytepos>] !/path/file.gco#". (Requires SDSUPPORT)
 *        Use P to run other files as sub-programs: "M32 P !filename#"
 *        The '#' is necessary when calling from within sd files, as it stops buffer prereading
 * M33  - Get the longname version of a path. (Requires LONG_FILENAME_HOST_SUPPORT)
 * M34  - Set SD Card sorting options. (Requires SDCARD_SORT_ALPHA)
 * M42  - Change pin status via gcode: M42 P<pin> S<value>. LED pin assumed if P is omitted.
 * M43  - Display pin status, watch pins for changes, watch endstops & toggle LED, Z servo probe test, toggle pins

 * M75  - Start the print job timer.
 * M76  - Pause the print job timer.
 * M77  - Stop the print job timer.
 * M78  - Show statistical information about the print jobs. (Requires PRINTCOUNTER)
 * M80  - Turn on Power Supply. (Requires POWER_SUPPLY > 0)
 * M81  - Turn off Power Supply. (Requires POWER_SUPPLY > 0)

 * M84  - Disable steppers until next move, or use S<seconds> to specify an idle
 *        duration after which steppers should turn off. S0 disables the timeout.
 * M85  - Set inactivity shutdown timer with parameter S<seconds>. To disable set zero (default)
 * M92  - Set planner.axis_steps_per_mm for one or more axes.
 * M100 - Watch Free Memory (for debugging) (Requires M100_FREE_MEMORY_WATCHER)

 * M110 - Set the current line number. (Used by host printing)
 * M111 - Set debug flags: "M111 S<flagbits>". See flag bits defined in enum.h.
 * M112 - Emergency stop.
 * M113 - Get or set the timeout interval for Host Keepalive "busy" messages. (Requires HOST_KEEPALIVE_FEATURE)
 * M114 - Report current position.
 * M115 - Report capabilities. (Extended capabilities requires EXTENDED_CAPABILITIES_REPORT)

 * M118 - Display a message in the host console.
 * M119 - Report endstops status.
 * M120 - Enable endstops detection.
 * M121 - Disable endstops detection.

 * M201 - Set max acceleration in units/s^2 for print moves: "M201 X<accel> Y<accel> Z<accel> E<accel>"
 * M202 - Set max acceleration in units/s^2 for travel moves: "M202 X<accel> Y<accel> Z<accel> E<accel>" ** UNUSED IN MARLIN! **
 * M203 - Set maximum feedrate: "M203 X<fr> Y<fr> Z<fr> E<fr>" in units/sec.
 * M204 - Set default acceleration in units/sec^2: P<printing> T<travel>
 * M205 - Set advanced settings. Current units apply:
            S<print> T<travel> minimum speeds
            B<minimum segment time>
            X<max X jerk>, Y<max Y jerk>, Z<max Z jerk>, E<max E jerk>
 * M206 - Set additional homing offset. (Disabled by NO_WORKSPACE_OFFSETS)
 * M207 - Set Retract Length: S<length>, Feedrate: F<units/min>, and Z lift: Z<distance>. (Requires FWRETRACT)
 * M208 - Set Recover (unretract) Additional (!) Length: S<length> and Feedrate: F<units/min>. (Requires FWRETRACT)
 * M209 - Turn Automatic Retract Detection on/off: S<0|1> (For slicers that don't support G10/11). (Requires FWRETRACT)
          Every normal extrude-only move will be classified as retract depending on the direction.
 * M211 - Enable, Disable, and/or Report software endstops: S<0|1> (Requires MIN_SOFTWARE_ENDSTOPS or MAX_SOFTWARE_ENDSTOPS)

 * M220 - Set Feedrate Percentage: "M220 S<percent>"

 * M226 - Wait until a pin is in a given state: "M226 P<pin> S<state>"

* M280 - Set servo position absolute: "M280 P<index> S<angle|µs>". (Requires servos)

 * M350 - Set microstepping mode. (Requires digital microstepping pins.)
 * M351 - Toggle MS1 MS2 pins directly. (Requires digital microstepping pins.)
  * M400 - Finish all moves.
 * M410 - Quickstop. Abort all planned moves.

 
 * M428 - Set the home_offset based on the current_position. Nearest edge applies. (Disabled by NO_WORKSPACE_OFFSETS)
 * M500 - Store parameters in EEPROM. (Requires EEPROM_SETTINGS)
 * M501 - Restore parameters from EEPROM. (Requires EEPROM_SETTINGS)
 * M502 - Revert to the default "factory settings". ** Does not write them to EEPROM! **
 * M503 - Print the current settings (in memory): "M503 S<verbose>". S0 specifies compact output.
 * M540 - Enable/disable SD card abort on endstop hit: "M540 S<state>". (Requires ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
 * M600 - Pause for filament change: "M600 X<pos> Y<pos> Z<raise> E<first_retract> L<later_retract>". (Requires ADVANCED_PAUSE_FEATURE)
 * M665 - Set delta configurations: "M665 L<diagonal rod> R<delta radius> S<segments/s> A<rod A trim mm> B<rod B trim mm> C<rod C trim mm> I<tower A trim angle> J<tower B trim angle> K<tower C trim angle>" (Requires DELTA)
 * M605 - Set dual x-carriage movement mode: "M605 S<mode> [X<x_offset>] [R<temp_offset>]". (Requires DUAL_X_CARRIAGE)
 * M860 - Report the position of position encoder modules.
 * M861 - Report the status of position encoder modules.
 * M862 - Perform an axis continuity test for position encoder modules.
 * M863 - Perform steps-per-mm calibration for position encoder modules.
 * M864 - Change position encoder module I2C address.
 * M865 - Check position encoder module firmware version.
 * M866 - Report or reset position encoder module error count.
 * M867 - Enable/disable or toggle error correction for position encoder modules.
 * M868 - Report or set position encoder module error correction threshold.
 * M869 - Report position encoder module error.
 * M900 - Get and/or Set advance K factor and WH/D ratio. (Requires LIN_ADVANCE)
 * M906 - Set or get motor current in milliamps using axis codes X, Y. Report values if no axis codes given. (Requires HAVE_TMC2130)
 * M907 - Set digital trimpot motor current using axis codes. (Requires a board with digital trimpots)
 * M908 - Control digital trimpot directly. (Requires DAC_STEPPER_CURRENT or DIGIPOTSS_PIN)
 * M909 - Print digipot/DAC current value. (Requires DAC_STEPPER_CURRENT)
 * M910 - Commit digipot/DAC value to external EEPROM via I2C. (Requires DAC_STEPPER_CURRENT)
 * M911 - Report stepper driver overtemperature pre-warn condition. (Requires HAVE_TMC2130)
 * M912 - Clear stepper driver overtemperature pre-warn condition flag. (Requires HAVE_TMC2130)
 * M913 - Set HYBRID_THRESHOLD speed. (Requires HYBRID_THRESHOLD)
 * M914 - Set SENSORLESS_HOMING sensitivity. (Requires SENSORLESS_HOMING)
 *
 * ************ Custom codes - This can change to suit future G-code regulations
 * M928 - Start SD logging: "M928 filename.gco". Stop with M29. (Requires SDSUPPORT)
 * M999 - Restart after being stopped by error
 *
 *
 */

#ifndef _GCODE_H_
#define _GCODE_H_

#include "../inc/MarlinConfig.h"
#include "parser.h"


class GcodeSuite {
public:

  GcodeSuite() {}
  static bool axis_relative_modes[];

  #if ENABLED(CNC_COORDINATE_SYSTEMS)
    #define MAX_COORDINATE_SYSTEMS 9
    static int8_t active_coordinate_system;
    static float coordinate_system[MAX_COORDINATE_SYSTEMS][XY];
    static bool select_coordinate_system(const int8_t _new);
  #endif

  static millis_t previous_cmd_ms;
  FORCE_INLINE static void refresh_cmd_timeout() { previous_cmd_ms = millis(); }


  static void get_destination_from_command();
  static void process_parsed_command();
  static void process_next_command();

  FORCE_INLINE static void home_all_axes() { G28(true); }


  #if ENABLED(HOST_KEEPALIVE_FEATURE)
    /**
     * States for managing Marlin and host communication
     * Marlin sends messages if blocked or busy
     */
    enum MarlinBusyState {
      NOT_BUSY,           // Not in a handler
      IN_HANDLER,         // Processing a GCode
      IN_PROCESS,         // Known to be blocking command input (as in G29)
      PAUSED_FOR_USER,    // Blocking pending any input
      PAUSED_FOR_INPUT    // Blocking pending text input (concept)
    };

    static MarlinBusyState busy_state;
    static uint8_t host_keepalive_interval;

    static void host_keepalive();

    #define KEEPALIVE_STATE(n) gcode.busy_state = gcode.n
  #else
    #define KEEPALIVE_STATE(n) NOOP
  #endif

  static void dwell(millis_t time);

private:

  static void G0_G1(
	
  );

  #if ENABLED(ARC_SUPPORT)
    static void G2_G3(const bool clockwise);
  #endif

  static void G4();

  #if ENABLED(BEZIER_CURVE_SUPPORT)
    static void G5();
  #endif

 #if ENABLED(INCH_MODE_SUPPORT)
    static void G20();
    static void G21();
  #endif

  static void G28(const bool always_home_all);

  #if ENABLED(CNC_COORDINATE_SYSTEMS)
    bool select_coordinate_system(const int8_t _new);
    static void G53();
    static void G54();
    static void G55();
    static void G56();
    static void G57();
    static void G58();
    static void G59();
  #endif

  static void G92();

  #if HAS_RESUME_CONTINUE
    static void M0_M1();
  #endif


  static void M17();

  static void M18_M84();

  #if ENABLED(SDSUPPORT)
    static void M20();
    static void M21();
    static void M22();
    static void M23();
    static void M24();
    static void M25();
    static void M26();
    static void M27();
    static void M28();
    static void M29();
    static void M30();
  #endif

  static void M31();

  #if ENABLED(SDSUPPORT)
    static void M32();
    #if ENABLED(LONG_FILENAME_HOST_SUPPORT)
      static void M33();
    #endif
    #if ENABLED(SDCARD_SORT_ALPHA) && ENABLED(SDSORT_GCODE)
      static void M34();
    #endif
  #endif

  static void M42();

  #if ENABLED(PINS_DEBUGGING)
    static void M43();
  #endif

  static void M75();
  static void M76();
  static void M77();

  #if ENABLED(PRINTCOUNTER)
    static void M78();
  #endif

  #if HAS_POWER_SWITCH
    static void M80();
  #endif

  static void M81();
  static void M82();
  static void M83();
  static void M85();
  static void M92();

  #if ENABLED(M100_FREE_MEMORY_WATCHER)
    static void M100();
  #endif


  #if DISABLED(EMERGENCY_PARSER)
    static void M112();
    static void M410();
  #endif

  static void M109();

  static void M110();
  static void M111();

  #if ENABLED(HOST_KEEPALIVE_FEATURE)
    static void M113();
  #endif

  static void M114();
  static void M115();
  static void M117();
  static void M118();
  static void M119();
  static void M120();
  static void M121();


  static void M200();
  static void M201();

  static void M203();
  static void M204();
  static void M205();

  #if HAS_M206_COMMAND
    static void M206();
  #endif

	
  static void M211();

  static void M220();
  static void M226();

  #if HAS_SERVOS
    static void M280();
  #endif

  static void M303();

  #if HAS_MICROSTEPS
    static void M350();
    static void M351();
  #endif

  static void M355();
 
  static void M400();

  #if HAS_M206_COMMAND
    static void M428();
  #endif

  static void M500();
  static void M501();
  static void M502();
  #if DISABLED(DISABLE_M503)
    static void M503();
  #endif

  #if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
    static void M540();
  #endif


  #if ENABLED(HAVE_TMC2130)
    static void M906();
    static void M911();
    static void M912();
    #if ENABLED(HYBRID_THRESHOLD)
      static void M913();
    #endif
    #if ENABLED(SENSORLESS_HOMING)
      static void M914();
    #endif
  #endif

  #if ENABLED(SDSUPPORT)
    static void M928();
  #endif

  static void M999();

};

extern GcodeSuite gcode;

#endif // _GCODE_H_

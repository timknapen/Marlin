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
 * gcode.cpp - Temporary container for all gcode handlers
 *             Most will migrate to classes, by feature.
 */

#include "gcode.h"
GcodeSuite gcode;

#include "parser.h"
#include "queue.h"
#include "../module/motion.h"

#if ENABLED(PRINTCOUNTER)
  #include "../module/printcounter.h"
#endif


#include "../Marlin.h" // for idle()

millis_t GcodeSuite::previous_cmd_ms;

bool GcodeSuite::axis_relative_modes[] = AXIS_RELATIVE_MODES;

#if ENABLED(HOST_KEEPALIVE_FEATURE)
  GcodeSuite::MarlinBusyState GcodeSuite::busy_state = NOT_BUSY;
  uint8_t GcodeSuite::host_keepalive_interval = DEFAULT_KEEPALIVE_INTERVAL;
#endif

#if ENABLED(CNC_COORDINATE_SYSTEMS)
  int8_t GcodeSuite::active_coordinate_system = -1; // machine space
  float GcodeSuite::coordinate_system[MAX_COORDINATE_SYSTEMS][XY];
#endif


/**
 * Set XY  destination and feedrate from the current GCode command
 *
 *  - Set destination from included axis codes
 *  - Set to current for missing axis codes
 *  - Set the feedrate, if included
 */
void GcodeSuite::get_destination_from_command() {
  LOOP_XY(i) {
    if (parser.seen(axis_codes[i])) {
      const float v = parser.value_axis_units((AxisEnum)i) + (axis_relative_modes[i] || relative_mode ? current_position[i] : 0);
      destination[i] = LOGICAL_TO_NATIVE(v, i);
    }
    else
      destination[i] = current_position[i];
  }

  if (parser.linearval('F') > 0.0)
    feedrate_mm_s = MMM_TO_MMS(parser.value_feedrate());

}

/**
 * Dwell waits immediately. It does not synchronize. Use M400 instead of G4
 */
void GcodeSuite::dwell(millis_t time) {
  refresh_cmd_timeout();
  time += previous_cmd_ms;
  while (PENDING(millis(), time)) idle();
}

//
// Placeholders for non-migrated codes
//
#if ENABLED(M100_FREE_MEMORY_WATCHER)
  extern void M100_dump_routine(const char * const title, const char *start, const char *end);
#endif

/**
 * Process the parsed command and dispatch it to its handler
 */
void GcodeSuite::process_parsed_command() {
  KEEPALIVE_STATE(IN_HANDLER);

  // Handle a known G, M, or T
  switch (parser.command_letter) {
    case 'G': switch (parser.codenum) {

      // G0, G1
      case 0:
      case 1:
          G0_G1();
        break;

      // G2, G3
      #if ENABLED(ARC_SUPPORT)
        case 2: // G2: CW ARC
        case 3: // G3: CCW ARC
          G2_G3(parser.codenum == 2);
          break;
      #endif

      // G4 Dwell
      case 4:
        G4();
        break;

      #if ENABLED(BEZIER_CURVE_SUPPORT)
        case 5: // G5: Cubic B_spline
          G5();
          break;
      #endif // BEZIER_CURVE_SUPPORT

      #if ENABLED(INCH_MODE_SUPPORT)
        case 20: // G20: Inch Mode
          G20();
          break;

        case 21: // G21: MM Mode
          G21();
          break;
      #endif // INCH_MODE_SUPPORT

      case 28: // G28: Home all axes, one at a time
        G28(false);
        break;

      case 90: // G90
        relative_mode = false;
        break;
      case 91: // G91
        relative_mode = true;
        break;

      case 92: // G92 - Set current axis position(s)
        G92();
        break;

      #if ENABLED(DEBUG_GCODE_PARSER)
        case 800:
          parser.debug(); // GCode Parser Test for G
          break;
      #endif
    }
    break;

    case 'M': switch (parser.codenum) {
      #if HAS_RESUME_CONTINUE
        case 0: // M0: Unconditional stop
        case 1: // M1: Conditional stop
          M0_M1();
          break;
      #endif // ULTIPANEL

      case 17: // M17: Enable all stepper motors
        M17();
        break;

      #if ENABLED(SDSUPPORT)
        case 20: M20(); break;    // M20: list SD card
        case 21: M21(); break;    // M21: init SD card
        case 22: M22(); break;    // M22: release SD card
        case 23: M23(); break;    // M23: Select file
        case 24: M24(); break;    // M24: Start SD print
        case 25: M25(); break;    // M25: Pause SD print
        case 26: M26(); break;    // M26: Set SD index
        case 27: M27(); break;    // M27: Get SD status
        case 28: M28(); break;    // M28: Start SD write
        case 29: M29(); break;    // M29: Stop SD write
        case 30: M30(); break;    // M30 <filename> Delete File
        case 32: M32(); break;    // M32: Select file and start SD print

        #if ENABLED(LONG_FILENAME_HOST_SUPPORT)
          case 33: M33(); break;  // M33: Get the long full path to a file or folder
        #endif

        #if ENABLED(SDCARD_SORT_ALPHA) && ENABLED(SDSORT_GCODE)
          case 34: M34(); break;  // M34: Set SD card sorting options
        #endif

        case 928: M928(); break;  // M928: Start SD write
      #endif // SDSUPPORT

      case 31: M31(); break;      // M31: Report time since the start of SD print or last M109

      case 42: M42(); break;      // M42: Change pin state

      #if ENABLED(PINS_DEBUGGING)
        case 43: M43(); break;    // M43: Read pin state
      #endif

      case 75: M75(); break;      // M75: Start print timer
      case 76: M76(); break;      // M76: Pause print timer
      case 77: M77(); break;      // M77: Stop print timer

      #if ENABLED(PRINTCOUNTER)
        case 78: M78(); break;    // M78: Show print statistics
      #endif

      #if ENABLED(M100_FREE_MEMORY_WATCHER)
        case 100: M100(); break;  // M100: Free Memory Report
      #endif

      case 110: M110(); break;    // M110: Set Current Line Number

      case 111: M111(); break;    // M111: Set debug level

      #if DISABLED(EMERGENCY_PARSER)
        case 112: M112(); break;  // M112: Emergency Stop
        case 410: M410(); break;  // M410: Quickstop - Abort all the planned moves.
      #endif

      #if ENABLED(HOST_KEEPALIVE_FEATURE)
        case 113: M113(); break; // M113: Set Host Keepalive interval
      #endif

	  #if HAS_POWER_SWITCH
        case 80: M80(); break;    // M80: Turn on Power Supply
      #endif

      case 81: M81(); break;      // M81: Turn off Power, including Power Supply, if possible

      case 18: // M18 => M84
      case 84: M18_M84(); break;  // M84: Disable all steppers or set timeout
      case 85: M85(); break;      // M85: Set inactivity stepper shutdown timeout

      case 92: M92(); break;      // M92: Set the steps-per-unit for one or more axes

      case 114: M114(); break;    // M114: Report current position

      case 115: M115(); break;    // M115: Report capabilities

      case 118: M118(); break;    // M118: Display a message in the host console

      case 119: M119(); break;    // M119: Report endstop states
      case 120: M120(); break;    // M120: Enable endstops
      case 121: M121(); break;    // M121: Disable endstops

	 case 201: M201(); break;  // M201: Set max acceleration for print moves (units/s^2)

      #if 0
        case 202: M202(); break; // Not used for Sprinter/grbl gen6
      #endif

      case 203: M203(); break;    // M203: Set max feedrate (units/sec)
      case 204: M204(); break;    // M204: Set acceleration
      case 205: M205(); break;    // M205: Set advanced settings

      #if HAS_M206_COMMAND
        case 206: M206(); break;  // M206: Set home offsets
      #endif

	  case 211: M211(); break;    // M211: Enable, Disable, and/or Report software endstops

      case 220: M220(); break;    // M220: Set Feedrate Percentage: S<percent>

	  case 226: M226(); break;    // M226: Wait until a pin reaches a state

      #if HAS_SERVOS
        case 280: M280(); break;  // M280: Set servo position absolute
	  #endif

      case 400: M400(); break;    // M400: Finish all moves

      #if HAS_M206_COMMAND
        case 428: M428(); break;  // M428: Apply current_position to home_offset
      #endif

      case 500: M500(); break;    // M500: Store settings in EEPROM
      case 501: M501(); break;    // M501: Read settings from EEPROM
      case 502: M502(); break;    // M502: Revert to default settings
      #if DISABLED(DISABLE_M503)
        case 503: M503(); break;  // M503: print settings currently in memory
      #endif

      #if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
        case 540: M540(); break;  // M540: Set abort on endstop hit for SD printing
      #endif

	#if ENABLED(HAVE_TMC2130)
        case 906: M906(); break;    // M906: Set motor current in milliamps using axis codes X, Y
        case 911: M911(); break;    // M911: Report TMC2130 prewarn triggered flags
        case 912: M912(); break;    // M912: Clear TMC2130 prewarn triggered flags
        #if ENABLED(HYBRID_THRESHOLD)
          case 913: M913(); break;  // M913: Set HYBRID_THRESHOLD speed.
        #endif
        #if ENABLED(SENSORLESS_HOMING)
          case 914: M914(); break;  // M914: Set SENSORLESS_HOMING sensitivity.
        #endif
      #endif

      #if HAS_MICROSTEPS
        case 350: M350(); break;    // M350: Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
        case 351: M351(); break;    // M351: Toggle MS1 MS2 pins directly, S# determines MS1 or MS2, X# sets the pin high/low.
      #endif


      #if ENABLED(DEBUG_GCODE_PARSER)
        case 800:
          parser.debug(); // GCode Parser Test for M
          break;
      #endif

      case 999: M999(); break;  // M999: Restart after being Stopped
    }
    break;

    default: parser.unknown_command_error();
  }

  KEEPALIVE_STATE(NOT_BUSY);

  ok_to_send();
}

/**
 * Process a single command and dispatch it to its handler
 * This is called from the main loop()
 */
void GcodeSuite::process_next_command() {
  char * const current_command = command_queue[cmd_queue_index_r];

  if (DEBUGGING(ECHO)) {
    SERIAL_ECHO_START();
    SERIAL_ECHOLN(current_command);
    #if ENABLED(M100_FREE_MEMORY_WATCHER)
      SERIAL_ECHOPAIR("slot:", cmd_queue_index_r);
      M100_dump_routine("   Command Queue:", (const char*)command_queue, (const char*)(command_queue + sizeof(command_queue)));
    #endif
  }

  // Parse the next command in the queue
  parser.parse(current_command);
  process_parsed_command();
}

#if ENABLED(HOST_KEEPALIVE_FEATURE)

  /**
   * Output a "busy" message at regular intervals
   * while the machine is not accepting commands.
   */
  void GcodeSuite::host_keepalive() {
    const millis_t ms = millis();
    static millis_t next_busy_signal_ms = 0;
    if (host_keepalive_interval && busy_state != NOT_BUSY) {
      if (PENDING(ms, next_busy_signal_ms)) return;
      switch (busy_state) {
        case IN_HANDLER:
        case IN_PROCESS:
          SERIAL_ECHO_START();
          SERIAL_ECHOLNPGM(MSG_BUSY_PROCESSING);
          break;
        case PAUSED_FOR_USER:
          SERIAL_ECHO_START();
          SERIAL_ECHOLNPGM(MSG_BUSY_PAUSED_FOR_USER);
          break;
        case PAUSED_FOR_INPUT:
          SERIAL_ECHO_START();
          SERIAL_ECHOLNPGM(MSG_BUSY_PAUSED_FOR_INPUT);
          break;
        default:
          break;
      }
    }
    next_busy_signal_ms = ms + host_keepalive_interval * 1000UL;
  }

#endif // HOST_KEEPALIVE_FEATURE

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

#include "../../inc/MarlinConfig.h"

#include "../gcode.h"
#include "../../module/motion.h"
#include "../../module/stepper.h"

#if ENABLED(M114_DETAIL)

  void report_xy(const float pos[], const uint8_t n = 2, const uint8_t precision = 3) {
    char str[12];
    for (uint8_t i = 0; i < n; i++) {
      SERIAL_CHAR(' ');
      SERIAL_CHAR(axis_codes[i]);
      SERIAL_CHAR(':');
      SERIAL_PROTOCOL(dtostrf(pos[i], 8, precision, str));
    }
    SERIAL_EOL();
  }

  void report_current_position_detail() {

    stepper.synchronize();

    SERIAL_PROTOCOLPGM("\nLogical:");
    const float logical[XY] = {
      LOGICAL_X_POSITION(current_position[X_AXIS]),
      LOGICAL_Y_POSITION(current_position[Y_AXIS]),
    };
    report_xy(logical);

    SERIAL_PROTOCOLPGM("Raw:    ");
    report_xy(current_position);

    float leveled[XY] = { current_position[X_AXIS], current_position[Y_AXIS] };


    SERIAL_PROTOCOLPGM("Stepper:");
    LOOP_XY(i) {
      SERIAL_CHAR(' ');
      SERIAL_CHAR(axis_codes[i]);
      SERIAL_CHAR(':');
      SERIAL_PROTOCOL(stepper.position((AxisEnum)i));
    }
    SERIAL_EOL();

    SERIAL_PROTOCOLPGM("FromStp:");
    get_cartesian_from_steppers();  // writes cartes[XY] (with forward kinematics)
    const float from_steppers[XY] = { cartes[X_AXIS], cartes[Y_AXIS] };
    report_xy(from_steppers);

    const float diff[XY] = {
      from_steppers[X_AXIS] - leveled[X_AXIS],
      from_steppers[Y_AXIS] - leveled[Y_AXIS]
    };
    SERIAL_PROTOCOLPGM("Differ: ");
    report_xy(diff);
  }

#endif // M114_DETAIL

/**
 * M114: Report current position to host
 */
void GcodeSuite::M114() {

  #if ENABLED(M114_DETAIL)
    if (parser.seen('D')) {
      report_current_position_detail();
      return;
    }
  #endif

  stepper.synchronize();
  report_current_position();
}

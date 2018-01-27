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

#if HAS_M206_COMMAND

#include "../gcode.h"
#include "../../module/motion.h"
#include "../../Marlin.h" // for axis_homed

/**
 * M206: Set Additional Homing Offset (X Y ).
 *
 */
void GcodeSuite::M206() {
  LOOP_XY(i)
    if (parser.seen(axis_codes[i]))
      set_home_offset((AxisEnum)i, parser.value_linear_units());

  report_current_position();
}

/**
 * M428: Set home_offset based on the distance between the
 *       current_position and the nearest "reference point."
 *       If an axis is past center its endstop position
 *       is the reference-point. Otherwise it uses 0. This allows
 *       the Z offset to be set near the bed when using a max endstop.
 *
 *       M428 can't be used more than 2cm away from 0 or an endstop.
 *
 *       Use M206 to set these values directly.
 */
void GcodeSuite::M428() {
  if (axis_unhomed_error()) return;

  float diff[XY];
  LOOP_XY(i) {
    diff[i] = base_home_pos((AxisEnum)i) - current_position[i];
    if (!WITHIN(diff[i], -20, 20) && home_dir((AxisEnum)i) > 0)
      diff[i] = -current_position[i];
    if (!WITHIN(diff[i], -20, 20)) {
      SERIAL_ERROR_START();
      SERIAL_ERRORLNPGM(MSG_ERR_M428_TOO_FAR);
      return;
    }
  }

  LOOP_XY(i) set_home_offset((AxisEnum)i, diff[i]);
  report_current_position();
}

#endif // HAS_M206_COMMAND

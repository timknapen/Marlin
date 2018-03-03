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

#include "../gcode.h"
#include "../../module/motion.h"
#include "../../Marlin.h"
#include "../../sd/cardreader.h"

extern float destination[XY];

#if ENABLED(NO_MOTION_BEFORE_HOMING)
  #define G0_G1_CONDITION !axis_unhomed_error(parser.seen('X'), parser.seen('Y'))
#else
  #define G0_G1_CONDITION true
#endif

/**
 * G0, G1: Coordinated movement of X Y axes
 */
void GcodeSuite::G0() {
	if (IsRunning() && G0_G1_CONDITION) {
		get_destination_from_command(); // For X Y
		prepare_move_to_destination();
	}
}
void GcodeSuite::G1() {
  if (IsRunning() && G0_G1_CONDITION) {
    get_destination_from_command(); // For X Y
	prepare_move_to_destination();


  }
}

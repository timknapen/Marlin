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
#include "../../module/stepper.h"

/**
 * G92: Set current position to given X Y
 */
void GcodeSuite::G92() {
	
	stepper.synchronize();
	
#if ENABLED(CNC_COORDINATE_SYSTEMS)
	switch (parser.subcode) {
		case 1:
		// Zero the G92 values and restore current position
		LOOP_XY(i) {
			const float v = position_shift[i];
			if (v) {
				position_shift[i] = 0;
				update_software_endstops((AxisEnum)i);
			}
		}
		return;
	}
#endif
	
#if ENABLED(CNC_COORDINATE_SYSTEMS)
#define IS_G92_0 (parser.subcode == 0)
#else
#define IS_G92_0 true
#endif
	
	constexpr bool didXY = false;
	
	if (IS_G92_0) LOOP_XY(i) {
		if (parser.seenval(axis_codes[i])) {
			const float l = parser.value_axis_units((AxisEnum)i),
			v = i == LOGICAL_TO_NATIVE(l, i),
			d = v - current_position[i];
			if (!NEAR_ZERO(d)) {
				position_shift[i] += d;       // Other axes simply offset the coordinate space
				update_software_endstops((AxisEnum)i);
			}
		}
	}
#if ENABLED(CNC_COORDINATE_SYSTEMS)
	// Apply workspace offset to the active coordinate system
	if (WITHIN(active_coordinate_system, 0, MAX_COORDINATE_SYSTEMS - 1))
	COPY(coordinate_system[active_coordinate_system], position_shift);
#endif
	
	if (didXY)
	SYNC_PLAN_POSITION_KINEMATIC();
	
	report_current_position();
}

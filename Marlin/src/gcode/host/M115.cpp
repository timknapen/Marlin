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
#include "../../inc/MarlinConfig.h"

/**
 * M115: Capabilities string
 */
void GcodeSuite::M115() {
  SERIAL_PROTOCOLLNPGM(MSG_M115_REPORT);

  #if ENABLED(EXTENDED_CAPABILITIES_REPORT)

    // SERIAL_XON_XOFF
    #if ENABLED(SERIAL_XON_XOFF)
      SERIAL_PROTOCOLLNPGM("Cap:SERIAL_XON_XOFF:1");
    #else
      SERIAL_PROTOCOLLNPGM("Cap:SERIAL_XON_XOFF:0");
    #endif

    // EEPROM (M500, M501)
    #if ENABLED(EEPROM_SETTINGS)
      SERIAL_PROTOCOLLNPGM("Cap:EEPROM:1");
    #else
      SERIAL_PROTOCOLLNPGM("Cap:EEPROM:0");
    #endif

	
    // PROGRESS (M530 S L, M531 <file>, M532 X L)
    SERIAL_PROTOCOLLNPGM("Cap:PROGRESS:0");

    // Print Job timer M75, M76, M77
    SERIAL_PROTOCOLLNPGM("Cap:PRINT_JOB:1");

	
	// BUILD_PERCENT (M73)
    #if ENABLED(LCD_SET_PROGRESS_MANUALLY)
      SERIAL_PROTOCOLLNPGM("Cap:BUILD_PERCENT:1");
    #else
      SERIAL_PROTOCOLLNPGM("Cap:BUILD_PERCENT:0");
    #endif

    // SOFTWARE_POWER (M80, M81)
    #if HAS_POWER_SWITCH
      SERIAL_PROTOCOLLNPGM("Cap:SOFTWARE_POWER:1");
    #else
      SERIAL_PROTOCOLLNPGM("Cap:SOFTWARE_POWER:0");
    #endif


    // EMERGENCY_PARSER (M108, M112, M410)
    #if ENABLED(EMERGENCY_PARSER)
      SERIAL_PROTOCOLLNPGM("Cap:EMERGENCY_PARSER:1");
    #else
      SERIAL_PROTOCOLLNPGM("Cap:EMERGENCY_PARSER:0");
    #endif

  #endif // EXTENDED_CAPABILITIES_REPORT
}

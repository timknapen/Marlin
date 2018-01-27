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
 * configuration_store.cpp
 *
 * Settings and EEPROM storage
 *
 * IMPORTANT:  Whenever there are changes made to the variables stored in EEPROM
 * in the functions below, also increment the version number. This makes sure that
 * the default values are used whenever there is a change to the data, to prevent
 * wrong data being written to the variables.
 *
 * ALSO: Variables in the Store and Retrieve sections must be in the same order.
 *       If a feature is disabled, some data must still be written that, when read,
 *       either sets a Sane Default, or results in No Change to the existing value.
 *
 */

#define EEPROM_VERSION "V46"

// Change EEPROM version if these are changed:
#define EEPROM_OFFSET 100

/**
 * V46 EEPROM Layout:
 *
 *  100  Version                                    (char x4)
 *  104  EEPROM CRC16                               (uint16_t)
 *
 *  106            E_STEPPERS                       (uint8_t)
 *  107  M92 XY    planner.axis_steps_per_mm        (float x4 ... x8) + 64
 *  123  M203 XY   planner.max_feedrate_mm_s        (float x4 ... x8) + 64
 *  139  M201 XY   planner.max_acceleration_mm_per_s2 (uint32_t x4 ... x8) + 64
 *  155  M204 P    planner.acceleration             (float)
 *  163  M204 T    planner.travel_acceleration      (float)
 *  167  M205 S    planner.min_feedrate_mm_s        (float)
 *  171  M205 T    planner.min_travel_feedrate_mm_s (float)
 *  175  M205 B    planner.min_segment_time_us      (ulong)
 *  179  M205 X    planner.max_jerk[X_AXIS]         (float)
 *  183  M205 Y    planner.max_jerk[Y_AXIS]         (float)
 *  195  M206 XY   home_offset                      (float x3)
 *  207  M218 XY   hotend_offset                    (float x3 per additional hotend) +16
 *
 * [XY]_DUAL_ENDSTOPS:                             12 bytes
 *  354  M666 X    x_endstop_adj                    (float)
 *  358  M666 Y    y_endstop_adj                    (float)
 *
 * HAVE_TMC2130:                                    22 bytes
 *  560  M906 X    Stepper X current                (uint16_t)
 *  562  M906 Y    Stepper Y current                (uint16_t)
 *  564  M906 Z    Stepper Z current                (uint16_t)
 *  566  M906 X2   Stepper X2 current               (uint16_t)
 *  568  M906 Y2   Stepper Y2 current               (uint16_t)
 *  570  M906 Z2   Stepper Z2 current               (uint16_t)
 *  572  M906 E0   Stepper E0 current               (uint16_t)
 *  574  M906 E1   Stepper E1 current               (uint16_t)
 *  576  M906 E2   Stepper E2 current               (uint16_t)
 *  578  M906 E3   Stepper E3 current               (uint16_t)
 *  580  M906 E4   Stepper E4 current               (uint16_t)
 *
 * HAS_MOTOR_CURRENT_PWM:
 *  590  M907 X    Stepper XY current               (uint32_t)
 *  594  M907 Z    Stepper Z current                (uint32_t)
 *  598  M907 E    Stepper E current                (uint32_t)
 *
 * CNC_COORDINATE_SYSTEMS                           108 bytes
 *  602  G54-G59.3 coordinate_system                (float x 27)
 *
 *
 *  722                                   Minimum end-point
 * 2251 (722 + 208 + 36 + 9 + 288 + 988)  Maximum end-point
 *
 * ========================================================================
 * meshes_begin (between max and min end-point, directly above)
 * -- MESHES --
 * meshes_end
 * -- MAT (Mesh Allocation Table) --                128 bytes (placeholder size)
 * mat_end = E2END (0xFFF)
 *
 */
#include "configuration_store.h"

MarlinSettings settings;

#include "endstops.h"
#include "planner.h"
#include "stepper.h"
#include "../core/language.h"
#include "../Marlin.h"

#include "../gcode/parser.h"

#if ENABLED(HAVE_TMC2130)
  #include "stepper_indirection.h"
#endif


/**
 * Post-process after Retrieve or Reset
 */
void MarlinSettings::postprocess() {
  const float oldpos[] = { current_position[X_AXIS], current_position[Y_AXIS] };

  // steps per s2 needs to be updated to agree with units per s2
  planner.reset_acceleration_rates();

  #if HAS_HOME_OFFSET
    // Software endstops depend on home_offset
    LOOP_XY(i) update_software_endstops((AxisEnum)i);
  #endif


#if HAS_MOTOR_CURRENT_PWM
    stepper.refresh_motor_power();
  #endif

 
  // Refresh steps_to_mm with the reciprocal of axis_steps_per_mm
  // and init stepper.count[], planner.position[] with current_position
  planner.refresh_positioning();

  // Various factors can change the current position
  if (memcmp(oldpos, current_position, sizeof(oldpos)))
    report_current_position();
}

#if ENABLED(EEPROM_SETTINGS)
  #include "../HAL/persistent_store_api.h"

  #define DUMMY_PID_VALUE 3000.0f
  #define EEPROM_START() int eeprom_index = EEPROM_OFFSET; HAL::PersistentStore::access_start()
  #define EEPROM_FINISH() HAL::PersistentStore::access_finish()
  #define EEPROM_SKIP(VAR) eeprom_index += sizeof(VAR)
  #define EEPROM_WRITE(VAR) HAL::PersistentStore::write_data(eeprom_index, (uint8_t*)&VAR, sizeof(VAR), &working_crc)
  #define EEPROM_READ(VAR) HAL::PersistentStore::read_data(eeprom_index, (uint8_t*)&VAR, sizeof(VAR), &working_crc)
  #define EEPROM_ASSERT(TST,ERR) if (!(TST)) do{ SERIAL_ERROR_START(); SERIAL_ERRORLNPGM(ERR); eeprom_read_error = true; }while(0)

  const char version[4] = EEPROM_VERSION;

  bool MarlinSettings::eeprom_error;


  /**
   * M500 - Store Configuration
   */
  bool MarlinSettings::save() {
    float dummy = 0.0f;
    char ver[4] = "000";

    uint16_t working_crc = 0;

    EEPROM_START();

    eeprom_error = false;
    #if ENABLED(FLASH_EEPROM_EMULATION)
      EEPROM_SKIP(ver);   // Flash doesn't allow rewriting without erase
    #else
      EEPROM_WRITE(ver);  // invalidate data first
    #endif
    EEPROM_SKIP(working_crc); // Skip the checksum slot

    working_crc = 0; // clear before first "real data"


    EEPROM_WRITE(planner.axis_steps_per_mm);
    EEPROM_WRITE(planner.max_feedrate_mm_s);
    EEPROM_WRITE(planner.max_acceleration_mm_per_s2);

    EEPROM_WRITE(planner.acceleration);
    EEPROM_WRITE(planner.travel_acceleration);
    EEPROM_WRITE(planner.min_feedrate_mm_s);
    EEPROM_WRITE(planner.min_travel_feedrate_mm_s);
    EEPROM_WRITE(planner.min_segment_time_us);
    EEPROM_WRITE(planner.max_jerk);
    #if !HAS_HOME_OFFSET
      const float home_offset[XY] = { 0 };
    #endif
    EEPROM_WRITE(home_offset);

   
    EEPROM_WRITE(zfh);

	#if ENABLED(X_DUAL_ENDSTOPS) || ENABLED(Y_DUAL_ENDSTOPS)
      // Write dual endstops in X, Y order. Unused = 0.0
      dummy = 0.0f;
      #if ENABLED(X_DUAL_ENDSTOPS)
        EEPROM_WRITE(endstops.x_endstop_adj);   // 1 float
      #else
        EEPROM_WRITE(dummy);
      #endif

      #if ENABLED(Y_DUAL_ENDSTOPS)
        EEPROM_WRITE(endstops.y_endstop_adj);   // 1 float
      #else
        EEPROM_WRITE(dummy);
      #endif

      for (uint8_t q = 8; q--;) EEPROM_WRITE(dummy);

    #else
      dummy = 0.0f;
      for (uint8_t q = 11; q--;) EEPROM_WRITE(dummy);
    #endif

    // Save TMC2130 Configuration, and placeholder values
    uint16_t val;
    #if ENABLED(HAVE_TMC2130)
      #if ENABLED(X_IS_TMC2130)
        val = stepperX.getCurrent();
      #else
        val = 0;
      #endif
      EEPROM_WRITE(val);
      #if ENABLED(Y_IS_TMC2130)
        val = stepperY.getCurrent();
      #else
        val = 0;
      #endif
      EEPROM_WRITE(val);
         #else
      val = 0;
      for (uint8_t q = 11; q--;) EEPROM_WRITE(val);
    #endif


    #if HAS_MOTOR_CURRENT_PWM
      for (uint8_t q = 3; q--;) EEPROM_WRITE(stepper.motor_current_setting[q]);
    #else
      const uint32_t dummyui32 = 0;
      for (uint8_t q = 3; q--;) EEPROM_WRITE(dummyui32);
    #endif

    //
    // CNC Coordinate Systems
    //

    #if ENABLED(CNC_COORDINATE_SYSTEMS)
      EEPROM_WRITE(coordinate_system); // 27 floats
    #else
      dummy = 0.0f;
      for (uint8_t q = 27; q--;) EEPROM_WRITE(dummy);
    #endif

    //
    // Skew correction factors
    //

  
      dummy = 0.0f;
      for (uint8_t q = 3; q--;) EEPROM_WRITE(dummy);

    if (!eeprom_error) {
      #if ENABLED(EEPROM_CHITCHAT)
        const int eeprom_size = eeprom_index;
      #endif

      const uint16_t final_crc = working_crc;

      // Write the EEPROM header
      eeprom_index = EEPROM_OFFSET;

      EEPROM_WRITE(version);
      EEPROM_WRITE(final_crc);

      // Report storage size
      #if ENABLED(EEPROM_CHITCHAT)
        SERIAL_ECHO_START();
        SERIAL_ECHOPAIR("Settings Stored (", eeprom_size - (EEPROM_OFFSET));
        SERIAL_ECHOPAIR(" bytes; crc ", (uint32_t)final_crc);
        SERIAL_ECHOLNPGM(")");
      #endif
    }
    EEPROM_FINISH();

    #if ENABLED(UBL_SAVE_ACTIVE_ON_M500)
      if (ubl.storage_slot >= 0)
        store_mesh(ubl.storage_slot);
    #endif

    return !eeprom_error;
  }

  /**
   * M501 - Retrieve Configuration
   */
  bool MarlinSettings::load() {
    uint16_t working_crc = 0;

    EEPROM_START();

    char stored_ver[4];
    EEPROM_READ(stored_ver);

    uint16_t stored_crc;
    EEPROM_READ(stored_crc);

    // Version has to match or defaults are used
    if (strncmp(version, stored_ver, 3) != 0) {
      if (stored_ver[0] != 'V') {
        stored_ver[0] = '?';
        stored_ver[1] = '\0';
      }
      #if ENABLED(EEPROM_CHITCHAT)
        SERIAL_ECHO_START();
        SERIAL_ECHOPGM("EEPROM version mismatch ");
        SERIAL_ECHOPAIR("(EEPROM=", stored_ver);
        SERIAL_ECHOLNPGM(" Marlin=" EEPROM_VERSION ")");
      #endif
      reset();
    }
    else {
      float dummy = 0;
      bool dummyb;

      working_crc = 0;  // Init to 0. Accumulated by EEPROM_READ

      // Number of esteppers may change
      uint8_t esteppers;
      EEPROM_READ(esteppers);

      //
      // Planner Motion
      //

      // Get only the number of E stepper parameters previously stored
      // Any steppers added later are set to their defaults
      const float def1[] = DEFAULT_AXIS_STEPS_PER_UNIT, def2[] = DEFAULT_MAX_FEEDRATE;
      const uint32_t def3[] = DEFAULT_MAX_ACCELERATION;
      float tmp1[XY + esteppers], tmp2[XY + esteppers];
      uint32_t tmp3[XY + esteppers];
      EEPROM_READ(tmp1);
      EEPROM_READ(tmp2);
      EEPROM_READ(tmp3);
      LOOP_XY(i) {
        planner.axis_steps_per_mm[i]          = i < XY + esteppers ? tmp1[i] : def1[i < COUNT(def1) ? i : COUNT(def1) - 1];
        planner.max_feedrate_mm_s[i]          = i < XY + esteppers ? tmp2[i] : def2[i < COUNT(def2) ? i : COUNT(def2) - 1];
        planner.max_acceleration_mm_per_s2[i] = i < XY + esteppers ? tmp3[i] : def3[i < COUNT(def3) ? i : COUNT(def3) - 1];
      }

      EEPROM_READ(planner.acceleration);
      EEPROM_READ(planner.travel_acceleration);
      EEPROM_READ(planner.min_feedrate_mm_s);
      EEPROM_READ(planner.min_travel_feedrate_mm_s);
      EEPROM_READ(planner.min_segment_time_us);
      EEPROM_READ(planner.max_jerk);

      //
      // Home Offset (M206)
      //

      #if !HAS_HOME_OFFSET
        float home_offset[XY];
      #endif
      EEPROM_READ(home_offset);

      //
      // Dual Endstops offsets
      //

      #if ENABLED(X_DUAL_ENDSTOPS) || ENABLED(Y_DUAL_ENDSTOPS) 
		
        #if ENABLED(X_DUAL_ENDSTOPS)
          EEPROM_READ(endstops.x_endstop_adj);  // 1 float
        #else
          EEPROM_READ(dummy);
        #endif
        #if ENABLED(Y_DUAL_ENDSTOPS)
          EEPROM_READ(endstops.y_endstop_adj);  // 1 float
        #else
          EEPROM_READ(dummy);
        #endif
		
		EEPROM_READ(dummy);
		

        for (uint8_t q=8; q--;) EEPROM_READ(dummy);

      #else

        for (uint8_t q=11; q--;) EEPROM_READ(dummy);

      #endif

      //
      // LCD Preheat settings
      //

      #if DISABLED(ULTIPANEL)
        int lcd_preheat_hotend_temp[2], lcd_preheat_bed_temp[2], lcd_preheat_fan_speed[2];
      #endif
      EEPROM_READ(lcd_preheat_hotend_temp); // 2 floats
      EEPROM_READ(lcd_preheat_bed_temp);    // 2 floats
      EEPROM_READ(lcd_preheat_fan_speed);   // 2 floats

      //EEPROM_ASSERT(
      //  WITHIN(lcd_preheat_fan_speed, 0, 255),
      //  "lcd_preheat_fan_speed out of range"
      //);

      //
      // Hotend PID
      //

      //
      // PID Extrusion Scaling
      //

      #if DISABLED(PID_EXTRUSION_SCALING)
        int lpq_len;
      #endif
      EEPROM_READ(lpq_len);

      //
      // LCD Contrast
      //

      #if !HAS_LCD_CONTRAST
        uint16_t lcd_contrast;
      #endif
      EEPROM_READ(lcd_contrast);

      //
      // Firmware Retraction
      //

      #if ENABLED(FWRETRACT)
        EEPROM_READ(fwretract.autoretract_enabled);
        EEPROM_READ(fwretract.retract_length);
        EEPROM_READ(fwretract.retract_feedrate_mm_s);
        EEPROM_READ(fwretract.retract_zlift);
        EEPROM_READ(fwretract.retract_recover_length);
        EEPROM_READ(fwretract.retract_recover_feedrate_mm_s);
        EEPROM_READ(fwretract.swap_retract_length);
        EEPROM_READ(fwretract.swap_retract_recover_length);
        EEPROM_READ(fwretract.swap_retract_recover_feedrate_mm_s);
      #else
        EEPROM_READ(dummyb);
        for (uint8_t q=8; q--;) EEPROM_READ(dummy);
      #endif


      //
      // TMC2130 Stepper Current
      //

      uint16_t val;
      #if ENABLED(HAVE_TMC2130)
        EEPROM_READ(val);
        #if ENABLED(X_IS_TMC2130)
          stepperX.setCurrent(val, R_SENSE, HOLD_MULTIPLIER);
        #endif
        EEPROM_READ(val);
        #if ENABLED(Y_IS_TMC2130)
          stepperY.setCurrent(val, R_SENSE, HOLD_MULTIPLIER);
        #endif
      #else
        for (uint8_t q = 11; q--;) EEPROM_READ(val);
      #endif

		
        EEPROM_READ(dummy);
        EEPROM_READ(dummy);

      //
      // Motor Current PWM
      //

      #if HAS_MOTOR_CURRENT_PWM
        for (uint8_t q = 3; q--;) EEPROM_READ(stepper.motor_current_setting[q]);
      #else
        uint32_t dummyui32;
        for (uint8_t q = 3; q--;) EEPROM_READ(dummyui32);
      #endif

      //
      // CNC Coordinate System
      //

      #if ENABLED(CNC_COORDINATE_SYSTEMS)
        (void)gcode.select_coordinate_system(-1); // Go back to machine space
        EEPROM_READ(gcode.coordinate_system);                  // 27 floats
      #else
        for (uint8_t q = 27; q--;) EEPROM_READ(dummy);
      #endif

      if (working_crc == stored_crc) {
        postprocess();
        #if ENABLED(EEPROM_CHITCHAT)
          SERIAL_ECHO_START();
          SERIAL_ECHO(version);
          SERIAL_ECHOPAIR(" stored settings retrieved (", eeprom_index - (EEPROM_OFFSET));
          SERIAL_ECHOPAIR(" bytes; crc ", (uint32_t)working_crc);
          SERIAL_ECHOLNPGM(")");
        #endif
      }
      else {
        #if ENABLED(EEPROM_CHITCHAT)
          SERIAL_ERROR_START();
          SERIAL_ERRORPGM("EEPROM CRC mismatch - (stored) ");
          SERIAL_ERROR(stored_crc);
          SERIAL_ERRORPGM(" != ");
          SERIAL_ERROR(working_crc);
          SERIAL_ERRORLNPGM(" (calculated)!");
        #endif
        reset();
      }

      #if ENABLED(AUTO_BED_LEVELING_UBL)
        meshes_begin = (eeprom_index + 32) & 0xFFF8;  // Pad the end of configuration data so it
                                                      // can float up or down a little bit without
                                                      // disrupting the mesh data
        ubl.report_state();

        if (!ubl.sanity_check()) {
          SERIAL_EOL();
          #if ENABLED(EEPROM_CHITCHAT)
            ubl.echo_name();
            SERIAL_ECHOLNPGM(" initialized.\n");
          #endif
        }
        else {
          #if ENABLED(EEPROM_CHITCHAT)
            SERIAL_PROTOCOLPGM("?Can't enable ");
            ubl.echo_name();
            SERIAL_PROTOCOLLNPGM(".");
          #endif
          ubl.reset();
        }

        if (ubl.storage_slot >= 0) {
          load_mesh(ubl.storage_slot);
          #if ENABLED(EEPROM_CHITCHAT)
            SERIAL_ECHOPAIR("Mesh ", ubl.storage_slot);
            SERIAL_ECHOLNPGM(" loaded from storage.");
          #endif
        }
        else {
          ubl.reset();
          #if ENABLED(EEPROM_CHITCHAT)
            SERIAL_ECHOLNPGM("UBL System reset()");
          #endif
        }
      #endif
    }

    #if ENABLED(EEPROM_CHITCHAT) && DISABLED(DISABLE_M503)
      report();
    #endif
    EEPROM_FINISH();

    return !eeprom_error;
  }


#else // !EEPROM_SETTINGS

  bool MarlinSettings::save() {
    SERIAL_ERROR_START();
    SERIAL_ERRORLNPGM("EEPROM disabled");
    return false;
  }

#endif // !EEPROM_SETTINGS

/**
 * M502 - Reset Configuration
 */
void MarlinSettings::reset() {
  static const float tmp1[] PROGMEM = DEFAULT_AXIS_STEPS_PER_UNIT, tmp2[] PROGMEM = DEFAULT_MAX_FEEDRATE;
  static const uint32_t tmp3[] PROGMEM = DEFAULT_MAX_ACCELERATION;
  LOOP_XY(i) {
    planner.axis_steps_per_mm[i]          = pgm_read_float(&tmp1[i < COUNT(tmp1) ? i : COUNT(tmp1) - 1]);
    planner.max_feedrate_mm_s[i]          = pgm_read_float(&tmp2[i < COUNT(tmp2) ? i : COUNT(tmp2) - 1]);
    planner.max_acceleration_mm_per_s2[i] = pgm_read_dword_near(&tmp3[i < COUNT(tmp3) ? i : COUNT(tmp3) - 1]);
  }

  planner.acceleration = DEFAULT_ACCELERATION;
  planner.travel_acceleration = DEFAULT_TRAVEL_ACCELERATION;
  planner.min_feedrate_mm_s = DEFAULT_MINIMUMFEEDRATE;
  planner.min_segment_time_us = DEFAULT_MINSEGMENTTIME;
  planner.min_travel_feedrate_mm_s = DEFAULT_MINTRAVELFEEDRATE;
  planner.max_jerk[X_AXIS] = DEFAULT_XJERK;
  planner.max_jerk[Y_AXIS] = DEFAULT_YJERK;



  #if HAS_HOME_OFFSET
    ZERO(home_offset);
  #endif

	
  #if ENABLED(X_DUAL_ENDSTOPS) || ENABLED(Y_DUAL_ENDSTOPS)

    #if ENABLED(X_DUAL_ENDSTOPS)
      endstops.x_endstop_adj = (
        #ifdef X_DUAL_ENDSTOPS_ADJUSTMENT
          X_DUAL_ENDSTOPS_ADJUSTMENT
        #else
          0
        #endif
      );
    #endif
    #if ENABLED(Y_DUAL_ENDSTOPS)
      endstops.y_endstop_adj = (
        #ifdef Y_DUAL_ENDSTOPS_ADJUSTMENT
          Y_DUAL_ENDSTOPS_ADJUSTMENT
        #else
          0
        #endif
      );
    #endif
  #endif

	

  endstops.enable_globally(
    #if ENABLED(ENDSTOPS_ALWAYS_ON_DEFAULT)
      true
    #else
      false
    #endif
  );

  #if ENABLED(HAVE_TMC2130)
    #if ENABLED(X_IS_TMC2130)
      stepperX.setCurrent(X_CURRENT, R_SENSE, HOLD_MULTIPLIER);
    #endif
    #if ENABLED(Y_IS_TMC2130)
      stepperY.setCurrent(Y_CURRENT, R_SENSE, HOLD_MULTIPLIER);
    #endif
	
  #endif

 
  #if HAS_MOTOR_CURRENT_PWM
    uint32_t tmp_motor_current_setting[3] = PWM_MOTOR_CURRENT;
    for (uint8_t q = 3; q--;)
      stepper.digipot_current(q, (stepper.motor_current_setting[q] = tmp_motor_current_setting[q]));
  #endif

  postprocess();

  #if ENABLED(EEPROM_CHITCHAT)
    SERIAL_ECHO_START();
    SERIAL_ECHOLNPGM(" Hardcoded Default Settings Loaded");
  #endif
}

#if DISABLED(DISABLE_M503)

  #define CONFIG_ECHO_START do{ if (!forReplay) SERIAL_ECHO_START(); }while(0)

  /**
   * M503 - Report current settings in RAM
   *
   * Unless specifically disabled, M503 is available even without EEPROM
   */
  void MarlinSettings::report(const bool forReplay) {

    /**
     * Announce current units, in case inches are being displayed
     */
    CONFIG_ECHO_START;
    #if ENABLED(INCH_MODE_SUPPORT)
      #define LINEAR_UNIT(N) (float(N) / parser.linear_unit_factor)
      SERIAL_ECHOPGM("  G2");
      SERIAL_CHAR(parser.linear_unit_factor == 1.0 ? '1' : '0');
      SERIAL_ECHOPGM(" ; Units in ");
      serialprintPGM(parser.linear_unit_factor == 1.0 ? PSTR("mm\n") : PSTR("inches\n"));
    #else
      #define LINEAR_UNIT(N) (N)
      SERIAL_ECHOLNPGM("  G21    ; Units in mm");
    #endif

    SERIAL_EOL();

    if (!forReplay) {
      CONFIG_ECHO_START;
      SERIAL_ECHOLNPGM(" Steps per unit:");
    }
    CONFIG_ECHO_START;
    SERIAL_ECHOPAIR("  M92 X", LINEAR_UNIT(planner.axis_steps_per_mm[X_AXIS]));
    SERIAL_ECHOPAIR(" Y", LINEAR_UNIT(planner.axis_steps_per_mm[Y_AXIS]));
    SERIAL_EOL();
   
    if (!forReplay) {
      CONFIG_ECHO_START;
      SERIAL_ECHOLNPGM(" Maximum feedrates (units/s):");
    }
    CONFIG_ECHO_START;
    SERIAL_ECHOPAIR("  M203 X", LINEAR_UNIT(planner.max_feedrate_mm_s[X_AXIS]));
    SERIAL_ECHOPAIR(" Y", LINEAR_UNIT(planner.max_feedrate_mm_s[Y_AXIS]));
	SERIAL_EOL();
   
    if (!forReplay) {
      CONFIG_ECHO_START;
      SERIAL_ECHOLNPGM(" Maximum Acceleration (units/s2):");
    }
    CONFIG_ECHO_START;
    SERIAL_ECHOPAIR("  M201 X", LINEAR_UNIT(planner.max_acceleration_mm_per_s2[X_AXIS]));
    SERIAL_ECHOPAIR(" Y", LINEAR_UNIT(planner.max_acceleration_mm_per_s2[Y_AXIS]));
    SERIAL_EOL();

    if (!forReplay) {
      CONFIG_ECHO_START;
      SERIAL_ECHOLNPGM(" Acceleration (units/s2): P<print_accel> R<retract_accel> T<travel_accel>");
    }
    CONFIG_ECHO_START;
    SERIAL_ECHOPAIR("  M204 P", LINEAR_UNIT(planner.acceleration));
    SERIAL_ECHOLNPAIR(" T", LINEAR_UNIT(planner.travel_acceleration));

    if (!forReplay) {
      CONFIG_ECHO_START;
      SERIAL_ECHOLNPGM(" Advanced: S<min_feedrate> T<min_travel_feedrate> B<min_segment_time_us> X<max_xy_jerk> ");
    }
    CONFIG_ECHO_START;
    SERIAL_ECHOPAIR("  M205 S", LINEAR_UNIT(planner.min_feedrate_mm_s));
    SERIAL_ECHOPAIR(" T", LINEAR_UNIT(planner.min_travel_feedrate_mm_s));
    SERIAL_ECHOPAIR(" B", planner.min_segment_time_us);
    SERIAL_ECHOPAIR(" X", LINEAR_UNIT(planner.max_jerk[X_AXIS]));
    SERIAL_ECHOLNPAIR(" Y", LINEAR_UNIT(planner.max_jerk[Y_AXIS]));
   

    #if HAS_M206_COMMAND
      if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM(" Home offset:");
      }
      CONFIG_ECHO_START;
      SERIAL_ECHOPAIR("  M206 X", LINEAR_UNIT(home_offset[X_AXIS]));
      SERIAL_ECHOLNPAIR(" Y", LINEAR_UNIT(home_offset[Y_AXIS]));
    #endif

  
   
	  
    #if ENABLED(X_DUAL_ENDSTOPS) || ENABLED(Y_DUAL_ENDSTOPS)
      if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM(" Endstop adjustment:");
      }
      CONFIG_ECHO_START;
      SERIAL_ECHOPGM("  M666");
      #if ENABLED(X_DUAL_ENDSTOPS)
        SERIAL_ECHOPAIR(" X", LINEAR_UNIT(endstops.x_endstop_adj));
      #endif
      #if ENABLED(Y_DUAL_ENDSTOPS)
        SERIAL_ECHOPAIR(" Y", LINEAR_UNIT(endstops.y_endstop_adj));
      #endif
      SERIAL_EOL();
    #endif 


  

    /**
     * TMC2130 stepper driver current
     */
    #if ENABLED(HAVE_TMC2130)
      if (!forReplay) {
        CONFIG_ECHO_START;
        SERIAL_ECHOLNPGM(" Stepper driver current:");
      }
      CONFIG_ECHO_START;
      SERIAL_ECHO("  M906");
      #if ENABLED(X_IS_TMC2130)
        SERIAL_ECHOPAIR(" X", stepperX.getCurrent());
      #endif
      #if ENABLED(Y_IS_TMC2130)
        SERIAL_ECHOPAIR(" Y", stepperY.getCurrent());
      #endif
      SERIAL_EOL();
    #endif

   
    #if HAS_MOTOR_CURRENT_PWM
      CONFIG_ECHO_START;
      if (!forReplay) {
        SERIAL_ECHOLNPGM(" Stepper motor currents:");
        CONFIG_ECHO_START;
      }
      SERIAL_ECHOPAIR("  M907 X", stepper.motor_current_setting[0]);
      SERIAL_ECHOPAIR(" Z", stepper.motor_current_setting[1]);
      SERIAL_ECHOPAIR(" E", stepper.motor_current_setting[2]);
      SERIAL_EOL();
    #endif
  }

#endif // !DISABLE_M503

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
 * endstops.cpp - A singleton object to manage endstops
 */

#include "endstops.h"
#include "stepper.h"

#include "../Marlin.h"
#include "../sd/cardreader.h"

// TEST_ENDSTOP: test the old and the current status of an endstop
#define TEST_ENDSTOP(ENDSTOP) (TEST(current_endstop_bits & old_endstop_bits, ENDSTOP))

Endstops endstops;

// public:

bool Endstops::enabled, Endstops::enabled_globally; // Initialized by settings.load()
volatile char Endstops::endstop_hit_bits; // use X_MIN, Y_MIN as BIT value

#if ENABLED(X_DUAL_ENDSTOPS) || ENABLED(Y_DUAL_ENDSTOPS)
  uint16_t
#else
  byte
#endif
    Endstops::current_endstop_bits = 0,
    Endstops::old_endstop_bits = 0;


#if ENABLED(X_DUAL_ENDSTOPS)
  float Endstops::x_endstop_adj; // Initialized by settings.load()
#endif
#if ENABLED(Y_DUAL_ENDSTOPS)
  float Endstops::y_endstop_adj; // Initialized by settings.load()
#endif


/**
 * Class and Instance Methods
 */

void Endstops::init() {

  #if HAS_X_MIN
    #if ENABLED(ENDSTOPPULLUP_XMIN)
      SET_INPUT_PULLUP(X_MIN_PIN);
    #else
      SET_INPUT(X_MIN_PIN);
    #endif
  #endif
	

  #if HAS_Y_MIN
    #if ENABLED(ENDSTOPPULLUP_YMIN)
      SET_INPUT_PULLUP(Y_MIN_PIN);
    #else
      SET_INPUT(Y_MIN_PIN);
    #endif
  #endif

	
  #if HAS_X_MAX
    #if ENABLED(ENDSTOPPULLUP_XMAX)
      SET_INPUT_PULLUP(X_MAX_PIN);
    #else
      SET_INPUT(X_MAX_PIN);
    #endif
  #endif

	
  #if HAS_Y_MAX
    #if ENABLED(ENDSTOPPULLUP_YMAX)
      SET_INPUT_PULLUP(Y_MAX_PIN);
    #else
      SET_INPUT(Y_MAX_PIN);
    #endif
  #endif

 
} // Endstops::init

void Endstops::report_state() {
  if (endstop_hit_bits) {
   
      #define _SET_STOP_CHAR(A,C) ;

    #define _ENDSTOP_HIT_ECHO(A,C) do{ \
      SERIAL_ECHOPAIR(" " STRINGIFY(A) ":", stepper.triggered_position_mm(A ##_AXIS)); \
      _SET_STOP_CHAR(A,C); }while(0)

    #define _ENDSTOP_HIT_TEST(A,C) \
      if (TEST(endstop_hit_bits, A ##_MIN) || TEST(endstop_hit_bits, A ##_MAX)) \
        _ENDSTOP_HIT_ECHO(A,C)

    #define ENDSTOP_HIT_TEST_X() _ENDSTOP_HIT_TEST(X,'X')
    #define ENDSTOP_HIT_TEST_Y() _ENDSTOP_HIT_TEST(Y,'Y')

    SERIAL_ECHO_START();
    SERIAL_ECHOPGM(MSG_ENDSTOPS_HIT);
    ENDSTOP_HIT_TEST_X();
    ENDSTOP_HIT_TEST_Y();

	SERIAL_EOL();


    hit_on_purpose();

    #if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED) && ENABLED(SDSUPPORT)
      if (stepper.abort_on_endstop_hit) {
        card.sdprinting = false;
        card.closefile();
        quickstop_stepper();
      }
    #endif
  }
} // Endstops::report_state

void Endstops::M119() {
  SERIAL_PROTOCOLLNPGM(MSG_M119_REPORT);
  #define ES_REPORT(AXIS) do{ \
    SERIAL_PROTOCOLPGM(MSG_##AXIS); \
    SERIAL_PROTOCOLLN(((READ(AXIS##_PIN)^AXIS##_ENDSTOP_INVERTING) ? MSG_ENDSTOP_HIT : MSG_ENDSTOP_OPEN)); \
  }while(0)
  #if HAS_X_MIN
    ES_REPORT(X_MIN);
  #endif
  #if HAS_X_MAX
    ES_REPORT(X_MAX);
  #endif
  #if HAS_Y_MIN
    ES_REPORT(Y_MIN);
  #endif
  #if HAS_Y_MAX
    ES_REPORT(Y_MAX);
  #endif
} // Endstops::M119

#if ENABLED(X_DUAL_ENDSTOPS)
  void Endstops::test_dual_x_endstops(const EndstopEnum es1, const EndstopEnum es2) {
    const byte x_test = TEST_ENDSTOP(es1) | (TEST_ENDSTOP(es2) << 1); // bit 0 for X, bit 1 for X2
    if (x_test && stepper.current_block->steps[X_AXIS] > 0) {
      SBI(endstop_hit_bits, X_MIN);
      if (!stepper.performing_homing || (x_test == 0x3))  //if not performing home or if both endstops were trigged during homing...
        stepper.kill_current_block();
    }
  }
#endif
#if ENABLED(Y_DUAL_ENDSTOPS)
  void Endstops::test_dual_y_endstops(const EndstopEnum es1, const EndstopEnum es2) {
    const byte y_test = TEST_ENDSTOP(es1) | (TEST_ENDSTOP(es2) << 1); // bit 0 for Y, bit 1 for Y2
    if (y_test && stepper.current_block->steps[Y_AXIS] > 0) {
      SBI(endstop_hit_bits, Y_MIN);
      if (!stepper.performing_homing || (y_test == 0x3))  //if not performing home or if both endstops were trigged during homing...
        stepper.kill_current_block();
    }
  }
#endif

// Check endstops - Called from ISR!
void Endstops::update() {

  #define _ENDSTOP(AXIS, MINMAX) AXIS ##_## MINMAX
  #define _ENDSTOP_PIN(AXIS, MINMAX) AXIS ##_## MINMAX ##_PIN
  #define _ENDSTOP_INVERTING(AXIS, MINMAX) AXIS ##_## MINMAX ##_ENDSTOP_INVERTING
  #define _ENDSTOP_HIT(AXIS, MINMAX) SBI(endstop_hit_bits, _ENDSTOP(AXIS, MINMAX))

  // UPDATE_ENDSTOP_BIT: set the current endstop bits for an endstop to its status
  #define UPDATE_ENDSTOP_BIT(AXIS, MINMAX) SET_BIT(current_endstop_bits, _ENDSTOP(AXIS, MINMAX), (READ(_ENDSTOP_PIN(AXIS, MINMAX)) != _ENDSTOP_INVERTING(AXIS, MINMAX)))
  // COPY_BIT: copy the value of SRC_BIT to DST_BIT in DST
  #define COPY_BIT(DST, SRC_BIT, DST_BIT) SET_BIT(DST, DST_BIT, TEST(DST, SRC_BIT))

  #define UPDATE_ENDSTOP(AXIS,MINMAX) do { \
      UPDATE_ENDSTOP_BIT(AXIS, MINMAX); \
      if (TEST_ENDSTOP(_ENDSTOP(AXIS, MINMAX)) && stepper.current_block->steps[_AXIS(AXIS)] > 0) { \
        _ENDSTOP_HIT(AXIS, MINMAX); \
        stepper.endstop_triggered(_AXIS(AXIS)); \
      } \
    } while(0)

  /**
   * Define conditions for checking endstops
   */

  #if IS_CORE
    #define S_(N) stepper.current_block->steps[CORE_AXIS_##N]
    #define D_(N) stepper.motor_direction(CORE_AXIS_##N)
  #endif

  #if CORE_IS_XY
    /**
     * Head direction in -X axis for CoreXY bots.
     *
     * If steps differ, both axes are moving.
     * If DeltaA == -DeltaB, the movement is only in the 2nd axis (Y or Z, handled below)
     * If DeltaA ==  DeltaB, the movement is only in the 1st axis (X)
     */
    #if ENABLED(COREXY)
      #define X_CMP ==
    #else
      #define X_CMP !=
    #endif
    #define X_MOVE_TEST ( S_(1) != S_(2) || (S_(1) > 0 && D_(1) X_CMP D_(2)) )
    #define X_AXIS_HEAD X_HEAD
  #else
    #define X_MOVE_TEST stepper.current_block->steps[X_AXIS] > 0
    #define X_AXIS_HEAD X_AXIS
  #endif

  #if CORE_IS_XY
    /**
     * Head direction in -Y axis for CoreXY bots.
     *
     * If steps differ, both axes are moving
     * If DeltaA ==  DeltaB, the movement is only in the 1st axis (X or Y)
     */
    #if ENABLED(COREYX)
      #define Y_CMP ==
    #else
      #define Y_CMP !=
    #endif
    #define Y_MOVE_TEST ( S_(1) != S_(2) || (S_(1) > 0 && D_(1) Y_CMP D_(2)) )
    #define Y_AXIS_HEAD Y_HEAD
  #else
    #define Y_MOVE_TEST stepper.current_block->steps[Y_AXIS] > 0
    #define Y_AXIS_HEAD Y_AXIS
  #endif

	#define X_MIN_TEST true
	#define X_MAX_TEST true
	
  /**
   * Check and update endstops according to conditions
   */

  if (X_MOVE_TEST) {
    if (stepper.motor_direction(X_AXIS_HEAD)) { // -direction
      #if HAS_X_MIN
        #if ENABLED(X_DUAL_ENDSTOPS)
          UPDATE_ENDSTOP_BIT(X, MIN);
          #if HAS_X2_MIN
            UPDATE_ENDSTOP_BIT(X2, MIN);
          #else
            COPY_BIT(current_endstop_bits, X_MIN, X2_MIN);
          #endif
          test_dual_x_endstops(X_MIN, X2_MIN);
        #else
          if (X_MIN_TEST) UPDATE_ENDSTOP(X, MIN);
        #endif
      #endif
    }
    else { // +direction
      #if HAS_X_MAX
        #if ENABLED(X_DUAL_ENDSTOPS)
          UPDATE_ENDSTOP_BIT(X, MAX);
          #if HAS_X2_MAX
            UPDATE_ENDSTOP_BIT(X2, MAX);
          #else
            COPY_BIT(current_endstop_bits, X_MAX, X2_MAX);
          #endif
          test_dual_x_endstops(X_MAX, X2_MAX);
        #else
          if (X_MIN_TEST) UPDATE_ENDSTOP(X, MAX);
        #endif

      #endif
    }
  }

  if (Y_MOVE_TEST) {
    if (stepper.motor_direction(Y_AXIS_HEAD)) { // -direction
      #if HAS_Y_MIN
        #if ENABLED(Y_DUAL_ENDSTOPS)
          UPDATE_ENDSTOP_BIT(Y, MIN);
          #if HAS_Y2_MIN
            UPDATE_ENDSTOP_BIT(Y2, MIN);
          #else
            COPY_BIT(current_endstop_bits, Y_MIN, Y2_MIN);
          #endif
          test_dual_y_endstops(Y_MIN, Y2_MIN);
        #else
          UPDATE_ENDSTOP(Y, MIN);
        #endif
      #endif
    }
    else { // +direction
      #if HAS_Y_MAX
        #if ENABLED(Y_DUAL_ENDSTOPS)
          UPDATE_ENDSTOP_BIT(Y, MAX);
          #if HAS_Y2_MAX
            UPDATE_ENDSTOP_BIT(Y2, MAX);
          #else
            COPY_BIT(current_endstop_bits, Y_MAX, Y2_MAX);
          #endif
          test_dual_y_endstops(Y_MAX, Y2_MAX);
        #else
          UPDATE_ENDSTOP(Y, MAX);
        #endif
      #endif
    }
  }


  old_endstop_bits = current_endstop_bits;

} // Endstops::update()

#if ENABLED(PINS_DEBUGGING)

  bool Endstops::monitor_flag = false;

  /**
   * monitors endstops for changes
   *
   * If a change is detected then the LED is toggled and
   * a message is sent out the serial port
   *
   * Yes, we could miss a rapid back & forth change but
   * that won't matter because this is all manual.
   *
   */
  void Endstops::monitor() {

    static uint16_t old_endstop_bits_local = 0;
    static uint8_t local_LED_status = 0;
    uint16_t current_endstop_bits_local = 0;

    #if HAS_X_MIN
      if (READ(X_MIN_PIN)) SBI(current_endstop_bits_local, X_MIN);
    #endif
    #if HAS_X_MAX
      if (READ(X_MAX_PIN)) SBI(current_endstop_bits_local, X_MAX);
    #endif
    #if HAS_Y_MIN
      if (READ(Y_MIN_PIN)) SBI(current_endstop_bits_local, Y_MIN);
    #endif
    #if HAS_Y_MAX
      if (READ(Y_MAX_PIN)) SBI(current_endstop_bits_local, Y_MAX);
    #endif
	  
    uint16_t endstop_change = current_endstop_bits_local ^ old_endstop_bits_local;

    if (endstop_change) {
      #if HAS_X_MIN
        if (TEST(endstop_change, X_MIN)) SERIAL_PROTOCOLPAIR("  X_MIN:", TEST(current_endstop_bits_local, X_MIN));
      #endif
      #if HAS_X_MAX
        if (TEST(endstop_change, X_MAX)) SERIAL_PROTOCOLPAIR("  X_MAX:", TEST(current_endstop_bits_local, X_MAX));
      #endif
      #if HAS_Y_MIN
        if (TEST(endstop_change, Y_MIN)) SERIAL_PROTOCOLPAIR("  Y_MIN:", TEST(current_endstop_bits_local, Y_MIN));
      #endif
      #if HAS_Y_MAX
        if (TEST(endstop_change, Y_MAX)) SERIAL_PROTOCOLPAIR("  Y_MAX:", TEST(current_endstop_bits_local, Y_MAX));
      #endif
      SERIAL_PROTOCOLPGM("\n\n");
      analogWrite(LED_PIN, local_LED_status);
      local_LED_status ^= 255;
      old_endstop_bits_local = current_endstop_bits_local;
    }
  }

#endif // PINS_DEBUGGING

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
 * motion.cpp
 */

#include "motion.h"
#include "endstops.h"
#include "stepper.h"
#include "planner.h"
#include "temperature.h"

#include "../gcode/gcode.h"

#include "../inc/MarlinConfig.h"


#if ENABLED(SENSORLESS_HOMING)
  #include "../feature/tmc2130.h"
#endif

#define XY_CONSTS(type, array, CONFIG) const PROGMEM type array##_P[XY] = { X_##CONFIG, Y_##CONFIG }

XY_CONSTS(float, base_min_pos,   MIN_POS);
XY_CONSTS(float, base_max_pos,   MAX_POS);
XY_CONSTS(float, base_home_pos,  HOME_POS);
XY_CONSTS(float, max_length,     MAX_LENGTH);
XY_CONSTS(float, home_bump_mm,   HOME_BUMP_MM);
XY_CONSTS(signed char, home_dir, HOME_DIR);

// Relative Mode. Enable with G91, disable with G90.
bool relative_mode = false;

/**
 * Cartesian Current Position
 *   Used to track the native machine position as moves are queued.
 *   Used by 'buffer_line_to_current_position' to do a move after changing it.
 *   Used by 'SYNC_PLAN_POSITION_KINEMATIC' to update 'planner.position'.
 */
float current_position[XY] = { 0.0 };

/**
 * Cartesian Destination
 *   The destination for a move, filled in by G-code movement commands,
 *   and expected by functions like 'prepare_move_to_destination'.
 *   Set with 'gcode_get_destination' or 'set_destination_from_current'.
 */
float destination[XY] = { 0.0 };


// The feedrate for the current move, often used as the default if
// no other feedrate is specified. Overridden for special moves.
// Set by the last G0 through G5 command's "F" parameter.
// Functions that override this for custom moves *must always* restore it!
float feedrate_mm_s = MMM_TO_MMS(1500.0);

int16_t feedrate_percentage = 100;

// Homing feedrate is const progmem - compare to constexpr in the header
const float homing_feedrate_mm_s[4] PROGMEM = {
	MMM_TO_MMS(HOMING_FEEDRATE_XY), MMM_TO_MMS(HOMING_FEEDRATE_XY),
};

// Cartesian conversion result goes here:
float cartes[XY];


/**
 * The workspace can be offset by some commands, or
 * these offsets may be omitted to save on computation.
 */
#if HAS_WORKSPACE_OFFSET
  #if HAS_POSITION_SHIFT
    // The distance that XY has been offset by G92. Reset by G28.
    float position_shift[XY] = { 0 };
  #endif
  #if HAS_HOME_OFFSET
    // This offset is added to the configured home position.
    // Set by M206, M428, or menu item. Saved to EEPROM.
    float home_offset[XY] = { 0 };
  #endif
  #if HAS_HOME_OFFSET && HAS_POSITION_SHIFT
    // The above two are combined to save on computes
    float workspace_offset[XY] = { 0 };
  #endif
#endif


/**
 * Output the current position to serial
 */
void report_current_position() {
  SERIAL_PROTOCOLPGM("X:");
  SERIAL_PROTOCOL(LOGICAL_X_POSITION(current_position[X_AXIS]));
  SERIAL_PROTOCOLPGM(" Y:");
  SERIAL_PROTOCOL(LOGICAL_Y_POSITION(current_position[Y_AXIS]));
  stepper.report_positions();
}

/**
 * sync_plan_position
 *
 * Set the planner/stepper positions directly from current_position with
 * no kinematic translation. Used for homing axes and cartesian/core syncing.
 */
void sync_plan_position() {
  planner.set_position_mm(current_position[X_AXIS], current_position[Y_AXIS]);
}

/**
 * Get the stepper positions in the cartes[] array.
 *
 */
void get_cartesian_from_steppers() {

      cartes[X_AXIS] = stepper.get_axis_position_mm(X_AXIS);
      cartes[Y_AXIS] = stepper.get_axis_position_mm(Y_AXIS);
}

/**
 * Set the current_position for an axis based on
 * the stepper positions
 *
 * To prevent small shifts in axis position always call
 * SYNC_PLAN_POSITION_KINEMATIC after updating axes with this.
 *
 * To keep hosts in sync, always call report_current_position
 * after updating the current_position.
 */
void set_current_from_steppers_for_axis(const AxisEnum axis) {
  get_cartesian_from_steppers();
  if (axis == ALL_AXES)
    COPY(current_position, cartes);
  else
    current_position[axis] = cartes[axis];
}

/**
 * Move the planner to the current position from wherever it last moved
 * (or from wherever it has been told it is located).
 */
void line_to_current_position() {
  planner.buffer_line(current_position[X_AXIS], current_position[Y_AXIS], feedrate_mm_s);
}

/**
 * Move the planner to the position stored in the destination array, which is
 * used by G0/G1/G2/G3/G5 and many other functions to set a destination.
 */
void buffer_line_to_destination(const float fr_mm_s) {
  planner.buffer_line(destination[X_AXIS], destination[Y_AXIS], fr_mm_s);
}

#if IS_KINEMATIC

  void sync_plan_position_kinematic() {
    planner.set_position_mm_kinematic(current_position);
  }

  /**
   * Calculate delta, start a line, and set current_position to destination
   */
  void prepare_uninterpolated_move_to_destination(const float fr_mm_s/*=0.0*/) {
    gcode.refresh_cmd_timeout();

   
      if ( current_position[X_AXIS] == destination[X_AXIS]
        && current_position[Y_AXIS] == destination[Y_AXIS]
      ) return;

      planner.buffer_line_kinematic(destination, MMS_SCALED(fr_mm_s ? fr_mm_s : feedrate_mm_s));

    set_current_from_destination();
  }

#endif // IS_KINEMATIC

/**
 *  Plan a move to (X, Y) and set the current_position
 *  The final current_position may not be the one that was requested
 */
void do_blocking_move_to(const float &rx, const float &ry, const float &fr_mm_s/*=0.0*/) {
  const float old_feedrate_mm_s = feedrate_mm_s;


    // If Z needs to raise, do it before moving XY
    /*
	 if (current_position[Z_AXIS] < rz) {
      feedrate_mm_s = z_feedrate;
      current_position[Z_AXIS] = rz;
      line_to_current_position();
    }
	 */

    feedrate_mm_s = fr_mm_s ? fr_mm_s : XY_PROBE_FEEDRATE_MM_S;
    current_position[X_AXIS] = rx;
    current_position[Y_AXIS] = ry;
    line_to_current_position();

    // If Z needs to lower, do it after moving XY
    /*
	 if (current_position[Z_AXIS] > rz) {
      feedrate_mm_s = z_feedrate;
      current_position[Z_AXIS] = rz;
      line_to_current_position();
    }
	 */


  stepper.synchronize();

  feedrate_mm_s = old_feedrate_mm_s;

}
void do_blocking_move_to_x(const float &rx, const float &fr_mm_s/*=0.0*/) {
	do_blocking_move_to(rx, current_position[Y_AXIS], fr_mm_s);
}
void do_blocking_move_to_xy(const float &rx, const float &ry, const float &fr_mm_s/*=0.0*/) {
	do_blocking_move_to(rx, ry, fr_mm_s);
}

//
// Prepare to do endstop or probe moves
// with custom feedrates.
//
//  - Save current feedrates
//  - Reset the rate multiplier
//  - Reset the command timeout
//  - Enable the endstops (for endstop moves)
//
void bracket_probe_move(const bool before) {
  static float saved_feedrate_mm_s;
  static int16_t saved_feedrate_percentage;

  if (before) {
    saved_feedrate_mm_s = feedrate_mm_s;
    saved_feedrate_percentage = feedrate_percentage;
    feedrate_percentage = 100;
    gcode.refresh_cmd_timeout();
  }
  else {
    feedrate_mm_s = saved_feedrate_mm_s;
    feedrate_percentage = saved_feedrate_percentage;
    gcode.refresh_cmd_timeout();
  }
}

void setup_for_endstop_or_probe_move() { bracket_probe_move(true); }
void clean_up_after_endstop_or_probe_move() { bracket_probe_move(false); }

// Software Endstops are based on the configured limits.
float soft_endstop_min[XY] = { X_MIN_BED, Y_MIN_BED },
      soft_endstop_max[XY] = { X_MAX_BED, Y_MAX_BED };

#if HAS_SOFTWARE_ENDSTOPS

  // Software Endstops are based on the configured limits.
  bool soft_endstops_enabled = true;

  /**
   * Constrain the given coordinates to the software endstops.
   *
   */
  void clamp_to_software_endstops(float target[XY]) {
    if (!soft_endstops_enabled) return;
  
      #if ENABLED(MIN_SOFTWARE_ENDSTOP_X)
        NOLESS(target[X_AXIS], soft_endstop_min[X_AXIS]);
      #endif
      #if ENABLED(MIN_SOFTWARE_ENDSTOP_Y)
        NOLESS(target[Y_AXIS], soft_endstop_min[Y_AXIS]);
      #endif
      #if ENABLED(MAX_SOFTWARE_ENDSTOP_X)
        NOMORE(target[X_AXIS], soft_endstop_max[X_AXIS]);
      #endif
      #if ENABLED(MAX_SOFTWARE_ENDSTOP_Y)
        NOMORE(target[Y_AXIS], soft_endstop_max[Y_AXIS]);
      #endif
  }

#endif


/**
 * Prepare a single move and get ready for the next one
 *
 * This may result in several calls to planner.buffer_line to
 * do smaller moves for DELTA, SCARA, mesh moves, etc.
 *
 * Make sure current_position[E] and destination[E] are good
 * before calling or cold/lengthy extrusion may get missed.
 */
void prepare_move_to_destination() {
  clamp_to_software_endstops(destination);
  gcode.refresh_cmd_timeout();


  if (  prepare_move_to_destination_cartesian() ) return;

  set_current_from_destination();
}

#if HAS_AXIS_UNHOMED_ERR

  bool axis_unhomed_error(const bool x/*=true*/, const bool y/*=true*/) {
    #if ENABLED(HOME_AFTER_DEACTIVATE)
      const bool xx = x && !axis_known_position[X_AXIS],
                 yy = y && !axis_known_position[Y_AXIS];
    #else
      const bool xx = x && !axis_homed[X_AXIS],
                 yy = y && !axis_homed[Y_AXIS];
    #endif
    if (xx || yy ) {
      SERIAL_ECHO_START();
      SERIAL_ECHOPGM(MSG_HOME " ");
      if (xx) SERIAL_ECHOPGM(MSG_X);
      if (yy) SERIAL_ECHOPGM(MSG_Y);
      SERIAL_ECHOLNPGM(" " MSG_FIRST);
      return true;
    }
    return false;
  }

#endif // HAS_AXIS_UNHOMED_ERR

/**
 * The homing feedrate may vary
 */
inline float get_homing_bump_feedrate(const AxisEnum axis) {
  static const uint8_t homing_bump_divisor[] PROGMEM = HOMING_BUMP_DIVISOR;
  uint8_t hbd = pgm_read_byte(&homing_bump_divisor[axis]);
  if (hbd < 1) {
    hbd = 10;
    SERIAL_ECHO_START();
    SERIAL_ECHOLNPGM("Warning: Homing Bump Divisor < 1");
  }
  return homing_feedrate(axis) / hbd;
}

/**
 * Home an individual linear axis
 */
static void do_homing_move(const AxisEnum axis, const float distance, const float fr_mm_s=0.0) {

  // Tell the planner the axis is at 0
  current_position[axis] = 0;

	
    sync_plan_position();
    current_position[axis] = distance;
    planner.buffer_line(current_position[X_AXIS],
						current_position[Y_AXIS],
						fr_mm_s ? fr_mm_s : homing_feedrate(axis)
						);

	
  stepper.synchronize();

  endstops.hit_on_purpose();
}

/**
 * Set an axis' current position to its home position (after homing).
 *
 * For Core and Cartesian robots this applies one-to-one when an
 * individual axis has been homed.
 *
 * Callers must sync the planner position after calling this!
 */
void set_axis_is_at_home(const AxisEnum axis) {
 
  axis_known_position[axis] = axis_homed[axis] = true;

  #if HAS_POSITION_SHIFT
    position_shift[axis] = 0;
    update_software_endstops(axis);
  #endif
	
	current_position[axis] = base_home_pos(axis);
}

/**
 * Home an individual "raw axis" to its endstop.
 * This applies to XY on Cartesian and Core robots.
 *
 * At the end of the procedure the axis is marked as
 * homed and the current position of that axis is updated.
 * Kinematic robots should wait till all axes are homed
 * before updating the current position.
 */

void homeaxis(const AxisEnum axis) {

 
    #define CAN_HOME(A) \
      (axis == A##_AXIS && ((A##_MIN_PIN > -1 && A##_HOME_DIR < 0) || (A##_MAX_PIN > -1 && A##_HOME_DIR > 0)))
    if (!CAN_HOME(X) && !CAN_HOME(Y) ) return;

 
  const int axis_home_dir = home_dir(axis);

 
  // Set flags for X, Y motor locking
  #if ENABLED(X_DUAL_ENDSTOPS)
    if (axis == X_AXIS) stepper.set_homing_flag_x(true);
  #endif
  #if ENABLED(Y_DUAL_ENDSTOPS)
    if (axis == Y_AXIS) stepper.set_homing_flag_y(true);
  #endif

  // Disable stealthChop if used. Enable diag1 pin on driver.
  #if ENABLED(SENSORLESS_HOMING)
    #if ENABLED(X_IS_TMC2130)
      if (axis == X_AXIS) tmc2130_sensorless_homing(stepperX);
    #endif
    #if ENABLED(Y_IS_TMC2130)
      if (axis == Y_AXIS) tmc2130_sensorless_homing(stepperY);
    #endif
  #endif


  do_homing_move(axis, 1.5 * max_length(axis) * axis_home_dir);

  // When homing Z with probe respect probe clearance
  const float bump = axis_home_dir * (
    home_bump_mm(axis)
  );

  // If a second homing move is configured...
  if (bump) {
    // Move away from the endstop by the axis HOME_BUMP_MM
    do_homing_move(axis, -bump);

    // Slow move towards endstop until triggered
    do_homing_move(axis, 2 * bump, get_homing_bump_feedrate(axis));
  }

  #if ENABLED(X_DUAL_ENDSTOPS) || ENABLED(Y_DUAL_ENDSTOPS)
    const bool pos_dir = axis_home_dir > 0;
    #if ENABLED(X_DUAL_ENDSTOPS)
      if (axis == X_AXIS) {
        const bool lock_x1 = pos_dir ? (endstops.x_endstop_adj > 0) : (endstops.x_endstop_adj < 0);
        float adj = FABS(endstops.x_endstop_adj);
        if (pos_dir) adj = -adj;
        if (lock_x1) stepper.set_x_lock(true); else stepper.set_x2_lock(true);
        do_homing_move(axis, adj);
        if (lock_x1) stepper.set_x_lock(false); else stepper.set_x2_lock(false);
        stepper.set_homing_flag_x(false);
      }
    #endif
    #if ENABLED(Y_DUAL_ENDSTOPS)
      if (axis == Y_AXIS) {
        const bool lock_y1 = pos_dir ? (endstops.y_endstop_adj > 0) : (endstops.y_endstop_adj < 0);
        float adj = FABS(endstops.y_endstop_adj);
        if (pos_dir) adj = -adj;
        if (lock_y1) stepper.set_y_lock(true); else stepper.set_y2_lock(true);
        do_homing_move(axis, adj);
        if (lock_y1) stepper.set_y_lock(false); else stepper.set_y2_lock(false);
        stepper.set_homing_flag_y(false);
      }
    #endif
#endif



    // For cartesian/core machines,
    // set the axis to its home position
    set_axis_is_at_home(axis);
    sync_plan_position();

    destination[axis] = current_position[axis];


  // Re-enable stealthChop if used. Disable diag1 pin on driver.
  #if ENABLED(SENSORLESS_HOMING)
    #if ENABLED(X_IS_TMC2130)
      if (axis == X_AXIS) tmc2130_sensorless_homing(stepperX, false);
    #endif
    #if ENABLED(Y_IS_TMC2130)
      if (axis == Y_AXIS) tmc2130_sensorless_homing(stepperY, false);
    #endif
  #endif


} // homeaxis()

#if HAS_WORKSPACE_OFFSET

  /**
   * Software endstops can be used to monitor the open end of
   * an axis that has a hardware endstop on the other end. Or
   * they can prevent axes from moving past endstops and grinding.
   *
   * To keep doing their job as the coordinate system changes,
   * the software endstop positions must be refreshed to remain
   * at the same positions relative to the machine.
   */
  void update_software_endstops(const AxisEnum axis) {
    #if HAS_HOME_OFFSET && HAS_POSITION_SHIFT
      workspace_offset[axis] = home_offset[axis] + position_shift[axis];
    #endif

   
      soft_endstop_min[axis] = base_min_pos(axis);
      soft_endstop_max[axis] = base_max_pos(axis);

  }

#endif // HAS_WORKSPACE_OFFSET || DUAL_X_CARRIAGE

#if HAS_M206_COMMAND
  /**
   * Change the home offset for an axis, update the current
   * position and the software endstops to retain the same
   * relative distance to the new home.
   *
   * Since this changes the current_position, code should
   * call sync_plan_position soon after this.
   */
  void set_home_offset(const AxisEnum axis, const float v) {
    current_position[axis] += v - home_offset[axis];
    home_offset[axis] = v;
    update_software_endstops(axis);
  }
#endif // HAS_M206_COMMAND

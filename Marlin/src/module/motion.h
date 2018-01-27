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
 * motion.h
 *
 * High-level motion commands to feed the planner
 * Some of these methods may migrate to the planner class.
 */

#ifndef MOTION_H
#define MOTION_H

#include "../inc/MarlinConfig.h"


extern bool relative_mode;

extern float current_position[XY],  // High-level current tool position
             destination[XY];       // Destination for a move

// Scratch space for a cartesian result
extern float cartes[XY];


/**
 * Feed rates are often configured with mm/m
 * but the planner and stepper like mm/s units.
 */
extern const float homing_feedrate_mm_s[2];
FORCE_INLINE float homing_feedrate(const AxisEnum a) { return pgm_read_float(&homing_feedrate_mm_s[a]); }

extern float feedrate_mm_s;

/**
 * Feedrate scaling and conversion
 */
extern int16_t feedrate_percentage;
#define MMS_SCALED(MM_S) ((MM_S)*feedrate_percentage*0.01)

extern float soft_endstop_min[XY], soft_endstop_max[XY];

FORCE_INLINE float pgm_read_any(const float *p) { return pgm_read_float_near(p); }
FORCE_INLINE signed char pgm_read_any(const signed char *p) { return pgm_read_byte_near(p); }

#define XY_DEFS(type, array, CONFIG) \
  extern const type array##_P[XY]; \
  FORCE_INLINE type array(AxisEnum axis) { return pgm_read_any(&array##_P[axis]); } \
  typedef void __void_##CONFIG##__

XY_DEFS(float, base_min_pos,   MIN_POS);
XY_DEFS(float, base_max_pos,   MAX_POS);
XY_DEFS(float, base_home_pos,  HOME_POS);
XY_DEFS(float, max_length,     MAX_LENGTH);
XY_DEFS(float, home_bump_mm,   HOME_BUMP_MM);
XY_DEFS(signed char, home_dir, HOME_DIR);

#if HAS_SOFTWARE_ENDSTOPS
  extern bool soft_endstops_enabled;
  void clamp_to_software_endstops(float target[XY]);
#else
  #define soft_endstops_enabled false
  #define clamp_to_software_endstops(x) NOOP
#endif

void report_current_position();

inline void set_current_from_destination() { COPY(current_position, destination); }
inline void set_destination_from_current() { COPY(destination, current_position); }

void get_cartesian_from_steppers();
void set_current_from_steppers_for_axis(const AxisEnum axis);

/**
 * sync_plan_position
 *
 * Set the planner/stepper positions directly from current_position with
 * no kinematic translation. Used for homing axes and cartesian/core syncing.
 */
void sync_plan_position();
void sync_plan_position_e();


#define SYNC_PLAN_POSITION_KINEMATIC() sync_plan_position()

/**
 * Move the planner to the current position from wherever it last moved
 * (or from wherever it has been told it is located).
 */
void line_to_current_position();

/**
 * Move the planner to the position stored in the destination array, which is
 * used by G0/G1/G2/G3/G5 and many other functions to set a destination.
 */
void buffer_line_to_destination(const float fr_mm_s);
void prepare_move_to_destination();

/**
 * Blocking movement and shorthand functions
 */
void do_blocking_move_to(const float &x, const float &y, const float &fr_mm_s=0.0);
void do_blocking_move_to_x(const float &x, const float &fr_mm_s=0.0);
void do_blocking_move_to_xy(const float &x, const float &y, const float &fr_mm_s=0.0);

void setup_for_endstop_or_probe_move();
void clean_up_after_endstop_or_probe_move();

void bracket_probe_move(const bool before);
void setup_for_endstop_or_probe_move();
void clean_up_after_endstop_or_probe_move();

//
// Homing
//

#define HAS_AXIS_UNHOMED_ERR (                                                     \
        HAS_M206_COMMAND                                                          \
    ) || ENABLED(NO_MOTION_BEFORE_HOMING)

#if HAS_AXIS_UNHOMED_ERR
  bool axis_unhomed_error(const bool x=true, const bool y=true);
#endif

#if ENABLED(NO_MOTION_BEFORE_HOMING)
  #define MOTION_CONDITIONS (IsRunning() && !axis_unhomed_error())
#else
  #define MOTION_CONDITIONS IsRunning()
#endif

void set_axis_is_at_home(const AxisEnum axis);

void homeaxis(const AxisEnum axis);
#define HOMEAXIS(LETTER) homeaxis(LETTER##_AXIS)

//
// Macros
//

// Workspace offsets
#if HAS_WORKSPACE_OFFSET
  #if HAS_HOME_OFFSET
    extern float home_offset[XY];
  #endif
  #if HAS_POSITION_SHIFT
    extern float position_shift[XY];
  #endif
#endif

#if HAS_HOME_OFFSET && HAS_POSITION_SHIFT
  extern float workspace_offset[XY];
  #define WORKSPACE_OFFSET(AXIS) workspace_offset[AXIS]
#elif HAS_HOME_OFFSET
  #define WORKSPACE_OFFSET(AXIS) home_offset[AXIS]
#elif HAS_POSITION_SHIFT
  #define WORKSPACE_OFFSET(AXIS) position_shift[AXIS]
#else
  #define WORKSPACE_OFFSET(AXIS) 0
#endif

#define NATIVE_TO_LOGICAL(POS, AXIS) ((POS) + WORKSPACE_OFFSET(AXIS))
#define LOGICAL_TO_NATIVE(POS, AXIS) ((POS) - WORKSPACE_OFFSET(AXIS))

#if HAS_POSITION_SHIFT || DISABLED(DELTA)
  #define LOGICAL_X_POSITION(POS)   NATIVE_TO_LOGICAL(POS, X_AXIS)
  #define LOGICAL_Y_POSITION(POS)   NATIVE_TO_LOGICAL(POS, Y_AXIS)
  #define RAW_X_POSITION(POS)       LOGICAL_TO_NATIVE(POS, X_AXIS)
  #define RAW_Y_POSITION(POS)       LOGICAL_TO_NATIVE(POS, Y_AXIS)
#else
  #define LOGICAL_X_POSITION(POS)   (POS)
  #define LOGICAL_Y_POSITION(POS)   (POS)
  #define RAW_X_POSITION(POS)       (POS)
  #define RAW_Y_POSITION(POS)       (POS)
#endif

/**
 * position_is_reachable family of functions
 */

 // CARTESIAN

  inline bool position_is_reachable(const float &rx, const float &ry) {
      // Add 0.001 margin to deal with float imprecision
      return WITHIN(rx, X_MIN_POS - 0.001, X_MAX_POS + 0.001)
          && WITHIN(ry, Y_MIN_POS - 0.001, Y_MAX_POS + 0.001);
  }



#if HAS_WORKSPACE_OFFSET || ENABLED(DUAL_X_CARRIAGE)
  void update_software_endstops(const AxisEnum axis);
#endif

#if HAS_M206_COMMAND
  void set_home_offset(const AxisEnum axis, const float v);
#endif

#endif // MOTION_H

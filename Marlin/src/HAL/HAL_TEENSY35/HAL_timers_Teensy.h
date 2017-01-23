/* **************************************************************************
 
 Marlin 3D Printer Firmware
 Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
 Copyright (c) 2015-2016 Nico Tonnhofer wurstnase.reprap@gmail.com

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
****************************************************************************/

/**
 * Description: HAL for Arduino Due and compatible (SAM3X8E)
 *
 * For __MK64FX512__
 */


#ifndef _HAL_TIMERS_TEENSY_H
#define _HAL_TIMERS_TEENSY_H

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include <stdint.h>

// --------------------------------------------------------------------------
// Defines
// --------------------------------------------------------------------------

#define FORCE_INLINE __attribute__((always_inline)) inline

#define HAL_TIMER_TYPE uint32_t

#define STEP_TIMER_NUM 0
#define TEMP_TIMER_NUM 1

#define STEPPER_TIMER STEP_TIMER_NUM
#define STEPPER_TIMER_PRESCALE // Not defined anywhere else!

#define HAL_TIMER_RATE         (F_BUS)
#define HAL_STEPPER_TIMER_RATE HAL_TIMER_RATE
#define HAL_TICKS_PER_US       (HAL_STEPPER_TIMER_RATE/1000000)

#define TEMP_TIMER_FREQUENCY   1000

#define ENABLE_STEPPER_DRIVER_INTERRUPT() HAL_timer_enable_interrupt (STEP_TIMER_NUM)
#define DISABLE_STEPPER_DRIVER_INTERRUPT()  HAL_timer_disable_interrupt (STEP_TIMER_NUM)

//
#define HAL_STEP_TIMER_ISR  extern "C" void ftm0_isr(void) //void TC3_Handler()
#define HAL_TEMP_TIMER_ISR  extern "C" void ftm1_isr(void) //void TC4_Handler()

// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------
/*
typedef struct {
  Tc          *pTimerRegs;
  uint16_t    channel;
  IRQn_Type   IRQ_Id;
  uint8_t     priority;
} tTimerConfig;
*/
// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

// extern const tTimerConfig TimerConfig [];

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

void HAL_timer_start (uint8_t timer_num, uint32_t frequency);

static FORCE_INLINE void HAL_timer_set_count (uint8_t timer_num, uint32_t count) {
  switch(timer_num) {
  case 0: FTM0_C0V = count; break;
  case 1: FTM1_C0V = count; break;
  default: break;
  }
/*
  const tTimerConfig *pConfig = &TimerConfig[timer_num];

  pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_RC = count;
*/
}

static FORCE_INLINE HAL_TIMER_TYPE HAL_timer_get_count (uint8_t timer_num) {
  switch(timer_num) {
  case 0: return FTM0_MOD;
  case 1: return FTM1_MOD;
  default: return 0;
  }
/*
  const tTimerConfig *pConfig = &TimerConfig[timer_num];

  return pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_RC;
*/
}

static FORCE_INLINE uint32_t HAL_timer_get_current_count(uint8_t timer_num) {
  switch(timer_num) {
  case 0: return FTM0_CNT;
  case 1: return FTM1_CNT;
  default: return 0;
  }
/*
  const tTimerConfig *pConfig = &TimerConfig[timer_num];

  return pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_CV;
*/
}

void HAL_timer_enable_interrupt (uint8_t timer_num);
void HAL_timer_disable_interrupt (uint8_t timer_num);

//void HAL_timer_isr_prologue (uint8_t timer_num);
void HAL_timer_isr_prologue(uint8_t timer_num);


// --------------------------------------------------------------------------
//
// --------------------------------------------------------------------------

#endif // _HAL_TIMERS_TEENSY_H


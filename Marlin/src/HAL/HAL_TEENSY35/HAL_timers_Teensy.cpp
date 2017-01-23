/* **************************************************************************
 
 Marlin 3D Printer Firmware
 Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 Copyright (c) 2016 Bob Cousins bobcousins42@googlemail.com
   
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

#if defined(__MK64FX512__)

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include "../HAL.h"

#include "HAL_timers_Teensy.h"

// --------------------------------------------------------------------------
// Externals
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Local defines
// --------------------------------------------------------------------------

#define NUM_HARDWARE_TIMERS 9

#define PRESCALER 2
// --------------------------------------------------------------------------
// Types
// --------------------------------------------------------------------------


// --------------------------------------------------------------------------
// Public Variables
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Private Variables
// --------------------------------------------------------------------------
/*
const tTimerConfig TimerConfig [NUM_HARDWARE_TIMERS] =
  {
    { TC0, 0, TC0_IRQn, 0},  // 0 - [servo timer5]
    { TC0, 1, TC1_IRQn, 0},  // 1
    { TC0, 2, TC2_IRQn, 0},  // 2
    { TC1, 0, TC3_IRQn, 2},  // 3 - stepper
    { TC1, 1, TC4_IRQn, 15}, // 4 - temperature
    { TC1, 2, TC5_IRQn, 0},  // 5 - [servo timer3]
    { TC2, 0, TC6_IRQn, 0},  // 6
    { TC2, 1, TC7_IRQn, 0},  // 7
    { TC2, 2, TC8_IRQn, 0},  // 8
  };
*/
// --------------------------------------------------------------------------
// Function prototypes
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Private functions
// --------------------------------------------------------------------------

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

/*
  Timer_clock1: Prescaler 2 -> 42MHz
  Timer_clock2: Prescaler 8 -> 10.5MHz
  Timer_clock3: Prescaler 32 -> 2.625MHz
  Timer_clock4: Prescaler 128 -> 656.25kHz
*/


void HAL_timer_start (uint8_t timer_num, uint32_t frequency) {
  switch (timer_num) {
  case 0:
    FTM0_MODE = FTM_MODE_WPDIS | FTM_MODE_FTMEN;
    FTM0_SC = 0x00; // Set this to zero before changing the modulus
    FTM0_CNT = 0x0000; // Reset the count to zero
    FTM0_MOD = 0xFFFF; // max modulus = 65535
    FTM0_C0V = 0xFFFF; // Initial FTM0 Channel 0 compare value 65535 - ?Hz
    FTM0_SC = (FTM_SC_CLKS(0b1)&FTM_SC_CLKS_MASK) | (FTM_SC_PS(0b11)&FTM_SC_PS_MASK); // Bus clock 60MHz divided by prescaler 8
    FTM0_C0SC = FTM_CSC_CHIE | FTM_CSC_MSA | FTM_CSC_ELSA;
    break;
  case 1:
    FTM1_MODE = FTM_MODE_WPDIS | FTM_MODE_FTMEN; // Disable write protection, Enable FTM1
    FTM1_SC = 0x00; // Set this to zero before changing the modulus
    FTM1_CNT = 0x0000; // Reset the count to zero
    FTM1_MOD = 0xFFFF; // max modulus = 65535
    FTM1_C0V = 0xFFFF; // Initial FTM0 Channel 0 compare value 65535 - ?Hz
    FTM1_SC = (FTM_SC_CLKS(0b1)&FTM_SC_CLKS_MASK) | (FTM_SC_PS(0b10)&FTM_SC_PS_MASK); // Bus clock 60MHz divided by prescaler 8
    FTM1_C0SC = FTM_CSC_CHIE | FTM_CSC_MSA | FTM_CSC_ELSA;
    break;
  default:
    break;
  }
/*
	Tc *tc = TimerConfig [timer_num].pTimerRegs;
	IRQn_Type irq = TimerConfig [timer_num].IRQ_Id;
	uint32_t channel = TimerConfig [timer_num].channel;

	pmc_set_writeprotect(false);
	pmc_enable_periph_clk((uint32_t)irq);
  NVIC_SetPriority (irq, TimerConfig [timer_num].priority);

  TC_Configure (tc, channel, TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK1);

	TC_SetRC(tc, channel, VARIANT_MCK/2/frequency);
	TC_Start(tc, channel);

	// enable interrupt on RC compare
	tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;

	NVIC_EnableIRQ(irq);
*/
}

void HAL_timer_enable_interrupt (uint8_t timer_num)
{
  switch(timer_num) {
  case 0:
    NVIC_ENABLE_IRQ(IRQ_FTM0);
    break;
  case 1:
    NVIC_ENABLE_IRQ(IRQ_FTM1);
    break;
  default:
    break;
  }
/*
	const tTimerConfig *pConfig = &TimerConfig [timer_num];

	pConfig->pTimerRegs->TC_CHANNEL [pConfig->channel].TC_IER = TC_IER_CPCS;
*/
}

void HAL_timer_disable_interrupt (uint8_t timer_num)
{
  switch (timer_num) {
  case 0:
    NVIC_DISABLE_IRQ(IRQ_FTM0);
    break;
  case 1:
    NVIC_DISABLE_IRQ(IRQ_FTM1);
    break;
  default:
    break;
  }
/*
	const tTimerConfig *pConfig = &TimerConfig [timer_num];

	pConfig->pTimerRegs->TC_CHANNEL [pConfig->channel].TC_IDR = TC_IDR_CPCS;
*/
}

void HAL_timer_isr_prologue(uint8_t timer_num) {
  switch(timer_num) {
  case 0:
    FTM0_CNT = 0x0000;
    FTM0_SC &= ~FTM_SC_TOF; // Clear FTM Overflow flag
    FTM0_C0SC &= ~FTM_CSC_CHF; // Clear FTM Channel Compare flag
    break;
  case 1:
    FTM1_CNT = 0x0000;
    FTM1_SC &= ~FTM_SC_TOF; // Clear FTM Overflow flag
    FTM1_C0SC &= ~FTM_CSC_CHF; // Clear FTM Channel Compare flag
    break;
  default:
    break;
  }
/*  const tTimerConfig *pConfig = &TimerConfig[timer_num];

  // Reading the status register clears the interrupt flag
  pConfig->pTimerRegs->TC_CHANNEL[pConfig->channel].TC_SR;
*/
}

#if 0
void HAL_timer_set_count (uint8_t timer_num, uint32_t count)
{
	const tTimerConfig *pConfig = &TimerConfig [timer_num];

	TC_SetRC (pConfig->pTimerRegs, pConfig->channel, count);
}

void HAL_timer_isr_prologue (uint8_t timer_num) {
	const tTimerConfig *pConfig = &TimerConfig [timer_num];

	TC_GetStatus (pConfig->pTimerRegs, pConfig->channel);
}
#endif

#endif // ARDUINO_ARCH_SAM


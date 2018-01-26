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
 * Include pins definitions
 *
 * Pins numbering schemes:
 *
 *  - Digital I/O pin number if used by READ/WRITE macros. (e.g., X_STEP_DIR)
 *    The FastIO headers map digital pins to their ports and functions.
 *
 *  - Analog Input number if used by analogRead or DAC. (e.g., TEMP_n_PIN)
 *    These numbers are the same in any pin mapping.
 */

#ifndef __PINS_H__
#define __PINS_H__

#include "../inc/MarlinConfig.h"

#if MB(RAMPS_13_EFB) || MB(RAMPS_14_EFB) || MB(RAMPS_PLUS_EFB) || MB(RAMPS_14_RE_ARM_EFB) || MB(RAMPS_SMART_EFB) || MB(RAMPS_DUO_EFB) || MB(RAMPS4DUE_EFB)
  #define IS_RAMPS_EFB
#elif MB(RAMPS_13_EEB) || MB(RAMPS_14_EEB) || MB(RAMPS_PLUS_EEB) || MB(RAMPS_14_RE_ARM_EEB) || MB(RAMPS_SMART_EEB) || MB(RAMPS_DUO_EEB) || MB(RAMPS4DUE_EEB)
  #define IS_RAMPS_EEB
#elif MB(RAMPS_13_EFF) || MB(RAMPS_14_EFF) || MB(RAMPS_PLUS_EFF) || MB(RAMPS_14_RE_ARM_EFF) || MB(RAMPS_SMART_EFF) || MB(RAMPS_DUO_EFF) || MB(RAMPS4DUE_EFF)
  #define IS_RAMPS_EFF
#elif MB(RAMPS_13_EEF) || MB(RAMPS_14_EEF) || MB(RAMPS_PLUS_EEF) || MB(RAMPS_14_RE_ARM_EEF) || MB(RAMPS_SMART_EEF) || MB(RAMPS_DUO_EEF) || MB(RAMPS4DUE_EEF)
  #define IS_RAMPS_EEF
#elif MB(RAMPS_13_SF)  || MB(RAMPS_14_SF)  || MB(RAMPS_PLUS_SF)  || MB(RAMPS_14_RE_ARM_SF)  || MB(RAMPS_SMART_SF)  || MB(RAMPS_DUO_SF)  || MB(RAMPS4DUE_SF)
  #define IS_RAMPS_SF
#endif

//
// RAMPS 1.3 / 1.4 - ATmega1280, ATmega2560
//

#if MB(RAMPS_OLD)
  #include "pins_RAMPS_OLD.h"
#elif MB(RAMPS_13_EFB)
  #include "pins_RAMPS_13.h"
#elif MB(RAMPS_13_EEB)
  #include "pins_RAMPS_13.h"
#elif MB(RAMPS_13_EFF)
  #include "pins_RAMPS_13.h"
#elif MB(RAMPS_13_EEF)
  #include "pins_RAMPS_13.h"
#elif MB(RAMPS_13_SF)
  #include "pins_RAMPS_13.h"
#elif MB(RAMPS_14_EFB)
  #include "pins_RAMPS.h"
#elif MB(RAMPS_14_EEB)
  #include "pins_RAMPS.h"
#elif MB(RAMPS_14_EFF)
  #include "pins_RAMPS.h"
#elif MB(RAMPS_14_EEF)
  #include "pins_RAMPS.h"
#elif MB(RAMPS_14_SF)
  #include "pins_RAMPS.h"
#elif MB(RAMPS_PLUS_EFB)
  #include "pins_RAMPS_PLUS.h"
#elif MB(RAMPS_PLUS_EEB)
  #include "pins_RAMPS_PLUS.h"
#elif MB(RAMPS_PLUS_EFF)
  #include "pins_RAMPS_PLUS.h"
#elif MB(RAMPS_PLUS_EEF)
  #include "pins_RAMPS_PLUS.h"
#elif MB(RAMPS_PLUS_SF)
  #include "pins_RAMPS_PLUS.h"

//
// RAMPS Derivatives - ATmega1280, ATmega2560
//

#elif MB(3DRAG)
  #include "pins_3DRAG.h"             // ATmega1280, ATmega2560
#elif MB(K8200)
  #include "pins_K8200.h"             // ATmega1280, ATmega2560 (3DRAG)
#elif MB(K8400)
  #include "pins_K8400.h"             // ATmega1280, ATmega2560 (3DRAG)
#elif MB(BAM_DICE)
  #include "pins_RAMPS.h"             // ATmega1280, ATmega2560
#elif MB(BAM_DICE_DUE)
  #include "pins_BAM_DICE_DUE.h"      // ATmega1280, ATmega2560
#elif MB(MKS_BASE)
  #include "pins_MKS_BASE.h"          // ATmega1280, ATmega2560
#elif MB(MKS_13)
  #include "pins_MKS_13.h"            // ATmega1280, ATmega2560
#elif MB(MKS_GEN_L)
  #include "pins_MKS_GEN_L.h"         // ATmega1280, ATmega2560
#elif MB(ZRIB_V20)
  #include "pins_ZRIB_V20.h"          // ATmega1280, ATmega2560 (MKS_13)
#elif MB(FELIX2)
  #include "pins_FELIX2.h"            // ATmega1280, ATmega2560
#elif MB(RIGIDBOARD)
  #include "pins_RIGIDBOARD.h"        // ATmega1280, ATmega2560
#elif MB(RIGIDBOARD_V2)
  #include "pins_RIGIDBOARD_V2.h"     // ATmega1280, ATmega2560
#elif MB(SAINSMART_2IN1)
  #include "pins_SAINSMART_2IN1.h"    // ATmega1280, ATmega2560
#elif MB(ULTIMAKER)
  #include "pins_ULTIMAKER.h"         // ATmega1280, ATmega2560
#elif MB(ULTIMAKER_OLD)
  #include "pins_ULTIMAKER_OLD.h"     // ATmega1280, ATmega2560
#elif MB(AZTEEG_X3)
  #include "pins_AZTEEG_X3.h"         // ATmega2560
#elif MB(AZTEEG_X3_PRO)
  #include "pins_AZTEEG_X3_PRO.h"     // ATmega2560
#elif MB(ULTIMAIN_2)
  #include "pins_ULTIMAIN_2.h"        // ATmega2560
#elif MB(RUMBA)
  #include "pins_RUMBA.h"             // ATmega2560
#elif MB(BQ_ZUM_MEGA_3D)
  #include "pins_BQ_ZUM_MEGA_3D.h"    // ATmega2560
#elif MB(MAKEBOARD_MINI)
  #include "pins_MAKEBOARD_MINI.h"    // ATmega2560

//
// Other ATmega1280, ATmega2560
//

#elif MB(CNCONTROLS_11)
  #include "pins_CNCONTROLS_11.h"     // ATmega1280, ATmega2560
#elif MB(CNCONTROLS_12)
  #include "pins_CNCONTROLS_12.h"     // ATmega1280, ATmega2560
#elif MB(MIGHTYBOARD_REVE)
  #include "pins_MIGHTYBOARD_REVE.h"  // ATmega1280, ATmega2560
#elif MB(CHEAPTRONIC)
  #include "pins_CHEAPTRONIC.h"       // ATmega2560
#elif MB(CHEAPTRONIC_V2)
  #include "pins_CHEAPTRONICv2.h"     // ATmega2560
#elif MB(MEGATRONICS)
  #include "pins_MEGATRONICS.h"       // ATmega2560
#elif MB(MEGATRONICS_2)
  #include "pins_MEGATRONICS_2.h"     // ATmega2560
#elif MB(MEGATRONICS_3) || MB(MEGATRONICS_31)
  #include "pins_MEGATRONICS_3.h"     // ATmega2560
#elif MB(RAMBO)
  #include "pins_RAMBO.h"             // ATmega2560
#elif MB(MINIRAMBO) || MB(MINIRAMBO_10A)
  #include "pins_MINIRAMBO.h"         // ATmega2560
#elif MB(ELEFU_3)
  #include "pins_ELEFU_3.h"           // ATmega2560
#elif MB(LEAPFROG)
  #include "pins_LEAPFROG.h"          // ATmega1280, ATmega2560
#elif MB(MEGACONTROLLER)
  #include "pins_MEGACONTROLLER.h"    // ATmega2560
#elif MB(SCOOVO_X9H)
  #include "pins_SCOOVO_X9H.h"        // ATmega2560
#elif MB(GT2560_REV_A)
  #include "pins_GT2560_REV_A.h"      // ATmega1280, ATmega2560
#elif MB(GT2560_REV_A_PLUS)
  #include "pins_GT2560_REV_A_PLUS.h" // ATmega1280, ATmega2560

//
// ATmega1281, ATmega2561
//

#elif MB(MINITRONICS)
  #include "pins_MINITRONICS.h"       // ATmega1281
#elif MB(SILVER_GATE)
  #include "pins_SILVER_GATE.h"       // ATmega2561

//
// Sanguinololu and Derivatives - ATmega644P, ATmega1284P
//

#elif MB(SANGUINOLOLU_11)
  #include "pins_SANGUINOLOLU_11.h"   // ATmega644P, ATmega1284P
#elif MB(SANGUINOLOLU_12)
  #include "pins_SANGUINOLOLU_12.h"   // ATmega644P, ATmega1284P
#elif MB(MELZI)
  #include "pins_MELZI.h"             // ATmega644P, ATmega1284P
#elif MB(MELZI_MAKR3D)
  #include "pins_MELZI_MAKR3D.h"      // ATmega644P, ATmega1284P
#elif MB(MELZI_CREALITY)
  #include "pins_MELZI_CREALITY.h"    // ATmega644P, ATmega1284P
#elif MB(STB_11)
  #include "pins_STB_11.h"            // ATmega644P, ATmega1284P
#elif MB(AZTEEG_X1)
  #include "pins_AZTEEG_X1.h"         // ATmega644P, ATmega1284P

//
// Other ATmega644P, ATmega644, ATmega1284P
//

#elif MB(GEN3_MONOLITHIC)
  #include "pins_GEN3_MONOLITHIC.h"   // ATmega644P
#elif MB(GEN3_PLUS)
  #include "pins_GEN3_PLUS.h"         // ATmega644P, ATmega1284P
#elif MB(GEN6)
  #include "pins_GEN6.h"              // ATmega644P, ATmega1284P
#elif MB(GEN6_DELUXE)
  #include "pins_GEN6_DELUXE.h"       // ATmega644P, ATmega1284P
#elif MB(GEN7_CUSTOM)
  #include "pins_GEN7_CUSTOM.h"       // ATmega644P, ATmega644, ATmega1284P
#elif MB(GEN7_12)
  #include "pins_GEN7_12.h"           // ATmega644P, ATmega644, ATmega1284P
#elif MB(GEN7_13)
  #include "pins_GEN7_13.h"           // ATmega644P, ATmega644, ATmega1284P
#elif MB(GEN7_14)
  #include "pins_GEN7_14.h"           // ATmega644P, ATmega644, ATmega1284P
#elif MB(OMCA_A)
  #include "pins_OMCA_A.h"            // ATmega644
#elif MB(OMCA)
  #include "pins_OMCA.h"              // ATmega644P, ATmega644
#elif MB(ANET_10)
  #include "pins_ANET_10.h"           // ATmega1284P
#elif MB(SETHI)
  #include "pins_SETHI.h"             // ATmega644P, ATmega644, ATmega1284P

//
// Teensyduino - AT90USB1286, AT90USB1286P
//

#elif MB(TEENSYLU)
  #include "pins_TEENSYLU.h"          // AT90USB1286, AT90USB1286P
#elif MB(PRINTRBOARD)
  #include "pins_PRINTRBOARD.h"       // AT90USB1286
#elif MB(PRINTRBOARD_REVF)
  #include "pins_PRINTRBOARD_REVF.h"  // AT90USB1286
#elif MB(BRAINWAVE)
  #include "pins_BRAINWAVE.h"         // AT90USB646
#elif MB(BRAINWAVE_PRO)
  #include "pins_BRAINWAVE_PRO.h"     // AT90USB1286
#elif MB(SAV_MKI)
  #include "pins_SAV_MKI.h"           // AT90USB1286
#elif MB(TEENSY2)
  #include "pins_TEENSY2.h"           // AT90USB1286
#elif MB(5DPRINT)
  #include "pins_5DPRINT.h"           // AT90USB1286

//
// Re-ARM - LPC1768
//

#elif MB(RAMPS_14_RE_ARM_EFB)
  #include "pins_RAMPS_RE_ARM.h"
#elif MB(RAMPS_14_RE_ARM_EEB)
  #include "pins_RAMPS_RE_ARM.h"
#elif MB(RAMPS_14_RE_ARM_EFF)
  #include "pins_RAMPS_RE_ARM.h"
#elif MB(RAMPS_14_RE_ARM_EEF)
  #include "pins_RAMPS_RE_ARM.h"
#elif MB(RAMPS_14_RE_ARM_SF)
  #include "pins_RAMPS_RE_ARM.h"

//
// Other 32-bit Boards
//

#elif MB(TEENSY35_36) /// HERE WE ARE!!
  #include "pins_TEENSY35_36.h"
#elif MB(DUE3DOM)
  #include "pins_DUE3DOM.h"
#elif MB(DUE3DOM_MINI)
  #include "pins_DUE3DOM_MINI.h"
#elif MB(RADDS)
  #include "pins_RADDS.h"
#elif MB(RURAMPS4D)
  #include "pins_RURAMPS4D.h"
#elif MB(RAMPS_FD_V1)
  #include "pins_RAMPS_FD_V1.h"
#elif MB(RAMPS_FD_V2)
  #include "pins_RAMPS_FD_V2.h"
#elif MB(RAMPS_SMART_EFB)
  #include "pins_RAMPS_SMART.h"
#elif MB(RAMPS_SMART_EEB)
  #include "pins_RAMPS_SMART.h"
#elif MB(RAMPS_SMART_EFF)
  #include "pins_RAMPS_SMART.h"
#elif MB(RAMPS_SMART_EEF)
  #include "pins_RAMPS_SMART.h"
#elif MB(RAMPS_SMART_SF)
  #include "pins_RAMPS_SMART.h"
#elif MB(RAMPS_DUO_EFB)
  #include "pins_RAMPS_DUO.h"
#elif MB(RAMPS_DUO_EEB)
  #include "pins_RAMPS_DUO.h"
#elif MB(RAMPS_DUO_EFF)
  #include "pins_RAMPS_DUO.h"
#elif MB(RAMPS_DUO_EEF)
  #include "pins_RAMPS_DUO.h"
#elif MB(RAMPS_DUO_SF)
  #include "pins_RAMPS_DUO.h"
#elif MB(RAMPS4DUE_EFB)
  #include "pins_RAMPS4DUE.h"
#elif MB(RAMPS4DUE_EEB)
  #include "pins_RAMPS4DUE.h"
#elif MB(RAMPS4DUE_EFF)
  #include "pins_RAMPS4DUE.h"
#elif MB(RAMPS4DUE_EEF)
  #include "pins_RAMPS4DUE.h"
#elif MB(RAMPS4DUE_SF)
  #include "pins_RAMPS4DUE.h"
#elif MB(ULTRATRONICS_PRO)
  #include "pins_ULTRATRONICS_PRO.h"
#elif MB(ARCHIM2)
  #include "pins_ARCHIM2.h"
#elif MB(ALLIGATOR)
  #include "pins_ALLIGATOR_R2.h"
#elif MB(STM32F1R)
  #include "pins_STM32F1R.h"
#elif MB(STM3R_MINI)
  #include "pins_STM3R_MINI.h"
#elif MB(MALYAN_M200)
  #include "pins_MALYAN_M200.h"
#elif MB(BEAST)
  #include "pins_BEAST.h"
#elif MB(CHITU3D)
  #include "pins_CHITU3D.h"
#elif MB(MKS_SBASE)
  #include "pins_MKS_SBASE.h"
#elif MB(AZSMZ_MINI)
  #include "pins_AZSMZ_MINI.h"
#elif MB(AZTEEG_X5_GT)
  #include "pins_AZTEEG_X5_GT.h"
#elif MB(BIQU_BQ111_A4)
  #include "pins_BIQU_BQ111_A4.h"
#else
  #error "Unknown MOTHERBOARD value set in Configuration.h"
#endif

// Define certain undefined pins
#ifndef X_MS1_PIN
  #define X_MS1_PIN -1
#endif
#ifndef X_MS2_PIN
  #define X_MS2_PIN -1
#endif
#ifndef Y_MS1_PIN
  #define Y_MS1_PIN -1
#endif
#ifndef Y_MS2_PIN
  #define Y_MS2_PIN -1
#endif

#ifndef SD_DETECT_PIN
  #define SD_DETECT_PIN -1
#endif
#ifndef SDPOWER
  #define SDPOWER -1
#endif
#ifndef LED_PIN
  #define LED_PIN -1
#endif
#ifndef PS_ON_PIN
  #define PS_ON_PIN -1
#endif
#ifndef KILL_PIN
  #define KILL_PIN -1
#endif
#ifndef SUICIDE_PIN
  #define SUICIDE_PIN -1
#endif


//
// Assign endstop pins for boards with only 3 connectors
//
#ifdef X_STOP_PIN
  #if X_HOME_DIR < 0
    #define X_MIN_PIN X_STOP_PIN
    #define X_MAX_PIN -1
  #else
    #define X_MIN_PIN -1
    #define X_MAX_PIN X_STOP_PIN
  #endif
#endif

#ifdef Y_STOP_PIN
  #if Y_HOME_DIR < 0
    #define Y_MIN_PIN Y_STOP_PIN
    #define Y_MAX_PIN -1
  #else
    #define Y_MIN_PIN -1
    #define Y_MAX_PIN Y_STOP_PIN
  #endif
#endif


//
// Disable unused endstop / probe pins
//

#if DISABLED(USE_XMAX_PLUG)
  #undef X_MAX_PIN
  #define X_MAX_PIN          -1
#endif

#if DISABLED(USE_YMAX_PLUG)
  #undef Y_MAX_PIN
  #define Y_MAX_PIN          -1
#endif

#if DISABLED(USE_XMIN_PLUG)
  #undef X_MIN_PIN
  #define X_MIN_PIN          -1
#endif

#if DISABLED(USE_YMIN_PLUG)
  #undef Y_MIN_PIN
  #define Y_MIN_PIN          -1
#endif

#ifndef HAL_SENSITIVE_PINS
  #define HAL_SENSITIVE_PINS
#endif

#define SENSITIVE_PINS { \
    X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, X_MIN_PIN, X_MAX_PIN, \
    Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, Y_MIN_PIN, Y_MAX_PIN, \
    PS_ON_PIN,  \
    X_MS1_PIN, X_MS2_PIN, Y_MS1_PIN, Y_MS2_PIN,\
    HAL_SENSITIVE_PINS \
  }


// Note: default SPI pins are defined in the HAL

#include "../HAL/HAL_spi_pins.h"

#endif // __PINS_H__

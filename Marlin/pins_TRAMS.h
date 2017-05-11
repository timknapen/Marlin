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

#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
  #error "Oops!  Make sure you have 'Arduino Mega' selected from the 'Tools -> Boards' menu."
#endif

#ifndef BOARD_NAME
  #define BOARD_NAME "TRAMS v1.00"
#endif

#include <avr/io.h>
/*
#ifdef _AVR_IOM2560_H_
  #warning DB1
#endif
#ifdef PH7
  #warning DB2
#endif
#ifdef _AVR_IOMXX0_1_H_
  #warning DB3
#endif
#ifdef PA0
  #warning DB4
#endif
*/
#define LARGE_FLASH true

#define CS_X        46
#define CS_Y        49
#define CS_Z        48
#define CS_E0       47
#define I2C_SCL           21
#define I2C_SDA         20

#ifdef NUM_SERVOS
  #define SERVO0_PIN         11

  #if NUM_SERVOS > 1
    #define SERVO1_PIN         6
  #endif

  #if NUM_SERVOS > 2
    #define SERVO2_PIN         5
  #endif

  #if NUM_SERVOS > 3
    #define SERVO3_PIN         4
  #endif
#endif

// SPI communication
#define SPI_MISO  PB3   // master in slave out
#define SPI_MOSI  PB2   // master out slave in
#define SPI_SCK   PB1   // serial clock
#define SPI_CS    PB0   // chip select not use, only use for spi initialize
#define PORT_SPI    PORTB // port-register for spi interface
#define DDR_SPI     DDRB  // data-direction-register for spi interface

#define XAXIS_CS    PL3   // chip select x
#define YAXIS_CS    PL0   // chip select y
#define ZAXIS_CS    PL1   // chip select z
#define E0AXIS_CS   PL2   // chip select e0
#define SPI_CS_PORT   PORTL // Port for chip select signals
#define SPI_CS_DDR    DDRL  // DDR for chip select signals


// TMC5130 driver enable
// x-axis
#define DRV_EN_X    PD7   // driver enable x
#define DRV_EN_X_PORT PORTD // port-register for driver enable x
#define DRV_EN_X_DDR  DDRD  // data-direction-register for driver enable x

// y-axis
#define DRV_EN_Y    PK0   // driver enable y
#define DRV_EN_Y_PORT PORTK // port-register for driver enable y
#define DRV_EN_Y_DDR  DDRK  // data-direction-register for driver enable y

// z-axis
#define DRV_EN_Z    PF2   // driver enable z
#define DRV_EN_Z_PORT PORTF // port-register for driver enable z
#define DRV_EN_Z_DDR  DDRF  // data-direction-register for driver enable z

// eo-axis
#define DRV_EN_E0   PA2   // driver enable eo
#define DRV_EN_E0_PORT  PORTA // port-register for driver enable e0
#define DRV_EN_E0_DDR DDRA  // data-direction-register for driver enable e0

//
// Servos
//
#ifdef IS_RAMPS_13
  #define SERVO0_PIN        7 // RAMPS_13 // Will conflict with BTN_EN2 on LCD_I2C_VIKI
#else
  #define SERVO0_PIN       11
#endif
#define SERVO1_PIN          6
#define SERVO2_PIN          5
#define SERVO3_PIN          4

//
// Limit Switches
//
#define X_MIN_PIN           3
#define X_MAX_PIN           2
#define Y_MIN_PIN          14
#define Y_MAX_PIN          15
#define Z_MIN_PIN          18
#define Z_MAX_PIN          19

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN  32
#endif

//
// Steppers
//
#define X_STEP_PIN         -1
#define X_DIR_PIN          -1
#define X_ENABLE_PIN       38

#define Y_STEP_PIN         -1
#define Y_DIR_PIN          -1
#define Y_ENABLE_PIN       62

#define Z_STEP_PIN         -1
#define Z_DIR_PIN          -1
#define Z_ENABLE_PIN       56

#define E0_STEP_PIN        -1
#define E0_DIR_PIN         -1
#define E0_ENABLE_PIN      24

//
// Temperature Sensors
//
#define TEMP_0_PIN         13   // ANALOG NUMBERING
#define TEMP_1_PIN         15   // ANALOG NUMBERING
#define TEMP_2_PIN         -1   // ANALOG NUMBERING
#define TEMP_BED_PIN       14   // ANALOG NUMBERING

//
// Heaters / Fans
//
#define HEATER_BED_PIN     8    // BED
#define FAN_PIN            9
#define HEATER_0_PIN       10   // EXTRUDER 1

#define PS_ON_PIN          12
#define KILL_PIN           -1
#define SDPOWER            -1
#define SDSS               53
#define LED_PIN            13

#define TRAMS_XAXIS 3   //Bit in PORTL which function as Slave Select PIN for X Axis
#define TRAMS_YAXIS 0   //Bit in PORTL which function as Slave Select PIN for Y Axis
#define TRAMS_ZAXIS 1   //Bit in PORTL which function as Slave Select PIN for Z Axis
#define TRAMS_E0AXIS 2  //Bit in PORTL which function as Slave Select PIN for E1 Axis

//
// LCD / Controller
//
#if ENABLED(ULTRA_LCD)

  #if ENABLED(REPRAPWORLD_GRAPHICAL_LCD)
    #define LCD_PINS_RS     49 // CS chip select /SS chip slave select
    #define LCD_PINS_ENABLE 51 // SID (MOSI)
    #define LCD_PINS_D4     52 // SCK (CLK) clock
  #elif ENABLED(NEWPANEL) && ENABLED(PANEL_ONE)
    #define LCD_PINS_RS 40
    #define LCD_PINS_ENABLE 42
    #define LCD_PINS_D4 65
    #define LCD_PINS_D5 66
    #define LCD_PINS_D6 44
    #define LCD_PINS_D7 64
  #else
    #define LCD_PINS_RS 16
    #define LCD_PINS_ENABLE 17
    #define LCD_PINS_D4 23
    #define LCD_PINS_D5 25
    #define LCD_PINS_D6 27
    #define LCD_PINS_D7 29
    #if DISABLED(NEWPANEL)
      #define BEEPER_PIN 33
      // Buttons are attached to a shift register
      // Not wired yet
      //#define SHIFT_CLK 38
      //#define SHIFT_LD 42
      //#define SHIFT_OUT 40
      //#define SHIFT_EN 17
    #endif
  #endif

  #if ENABLED(NEWPANEL)

    #if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)
      #define BEEPER_PIN 37

      #define BTN_EN1 31
      #define BTN_EN2 33
      #define BTN_ENC 35

      #define SD_DETECT_PIN 49
      #define KILL_PIN 41

      #if ENABLED(BQ_LCD_SMART_CONTROLLER)
        #define LCD_BACKLIGHT_PIN 39
      #endif

    #elif ENABLED(REPRAPWORLD_GRAPHICAL_LCD)
      #define BTN_EN1 64
      #define BTN_EN2 59
      #define BTN_ENC 63
      #define SD_DETECT_PIN 42
    #elif ENABLED(LCD_I2C_PANELOLU2)
      #define BTN_EN1 47  // reverse if the encoder turns the wrong way.
      #define BTN_EN2 43
      #define BTN_ENC 32
      #define LCD_SDSS 53
      #define SD_DETECT_PIN -1
      #define KILL_PIN 41
    #elif ENABLED(LCD_I2C_VIKI)
      #define BTN_EN1 22  // reverse if the encoder turns the wrong way.
      #define BTN_EN2 7   // http://files.panucatt.com/datasheets/viki_wiring_diagram.pdf
                          // tells about 40/42.
                          // 22/7 are unused on RAMPS_14. 22 is unused and 7 the SERVO0_PIN on RAMPS_13.
      #define BTN_ENC -1
      #define LCD_SDSS 53
      #define SD_DETECT_PIN 49
    #elif ENABLED(VIKI2) || ENABLED(miniVIKI)
      #define BEEPER_PIN       33

      // Pins for DOGM SPI LCD Support
      #define DOGLCD_A0        44
      #define DOGLCD_CS        45
      #define LCD_SCREEN_ROT_180

      #define BTN_EN1          22
      #define BTN_EN2           7
      #define BTN_ENC          39

      #define SDSS             53
      #define SD_DETECT_PIN    -1  // Pin 49 for display sd interface, 72 for easy adapter board

      #define KILL_PIN         31

      #define STAT_LED_RED_PIN 32
      #define STAT_LED_BLUE_PIN 35

    #elif ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)
      #define BTN_EN1 35  // reverse if the encoder turns the wrong way.
      #define BTN_EN2 37
      #define BTN_ENC 31
      #define SD_DETECT_PIN 49
      #define LCD_SDSS 53
      #define KILL_PIN 41
      #define BEEPER_PIN 23
      #define DOGLCD_CS 29
      #define DOGLCD_A0 27
      #define LCD_BACKLIGHT_PIN 33
    #elif ENABLED(MINIPANEL)
      #define BEEPER_PIN 42
      // Pins for DOGM SPI LCD Support
      #define DOGLCD_A0  44
      #define DOGLCD_CS  66
      #define LCD_BACKLIGHT_PIN 65 // backlight LED on A11/D65
      #define SDSS   53

      #define KILL_PIN 64
      // GLCD features
      //#define LCD_CONTRAST 190
      // Uncomment screen orientation
      //#define LCD_SCREEN_ROT_90
      //#define LCD_SCREEN_ROT_180
      //#define LCD_SCREEN_ROT_270
      // The encoder and click button
      #define BTN_EN1 40
      #define BTN_EN2 63
      #define BTN_ENC 59
      // not connected to a pin
      #define SD_DETECT_PIN 49

    #else

      // Beeper on AUX-4
      #define BEEPER_PIN 33

      // buttons are directly attached using AUX-2
      #if ENABLED(REPRAPWORLD_KEYPAD)
        #define BTN_EN1 64 // encoder
        #define BTN_EN2 59 // encoder
        #define BTN_ENC 63 // enter button
        #define SHIFT_OUT 40 // shift register
        #define SHIFT_CLK 44 // shift register
        #define SHIFT_LD 42 // shift register
      #elif ENABLED(PANEL_ONE)
        #define BTN_EN1 59 // AUX2 PIN 3
        #define BTN_EN2 63 // AUX2 PIN 4
        #define BTN_ENC 49 // AUX3 PIN 7
      #else
        #define BTN_EN1 37
        #define BTN_EN2 35
        #define BTN_ENC 31 // the click
      #endif

      #if ENABLED(G3D_PANEL)
        #define SD_DETECT_PIN 49
        #define KILL_PIN 41
      #else
        //#define SD_DETECT_PIN -1 // Ramps doesn't use this
      #endif

    #endif
  #endif // NEWPANEL

#endif // ULTRA_LCD

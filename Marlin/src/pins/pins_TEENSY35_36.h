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

/****************************************************************************************
* Teensy 3.5 (MK64FX512) and Teensy 3.6 (MK66FX1M0) Breadboard pin assignments
* Requires the Teensyduino software with Teensy 3.5 or Teensy 3.6 selected in Arduino IDE!
* http://www.pjrc.com/teensy/teensyduino.html
****************************************************************************************/

#if !IS_32BIT_TEENSY
  #error "Oops!  Make sure you have 'Teensy 3.5' or 'Teensy 3.6' selected from the 'Tools -> Boards' menu."
#endif

#if IS_TEENSY35
  #define BOARD_NAME "Teensy3.5"
#elif IS_TEENSY36
  #define BOARD_NAME "Teensy3.6"
#endif

#define AT90USB 1286   // Disable MarlinSerial etc.
#define USBCON //1286  // Disable MarlinSerial etc.
/*
 
 Teensy + Trinamic 2D Marlin prototype pins:  8 feb 2018
 2	X_END
 3	X_DIR
 4	X_STEP
 5	X_ENABLE
 7	X_CS
 
 11	MOSI
 12	MISO
 13	SCLK
 
 15 Y_DIR
 16	Y_STEP
 19	Y_CS
 22	Y_ENABLE
 23	Y_END

                                         Drawing Machine plan for Teensy3.5
                                                     USB
                                          GND |-----#####-----| VIN 5V
                          MOSI1   RX1       0 |     #####     | Analog GND
						  MISO1   TX1       1 |               | 3.3V
      X_STOP_PIN                       PWM  2 | *NC     AREF* | 23  A9 PWM                    Y_STOP_PIN
      X_DIR_PIN           SCL2 CAN0TX  PWM  3 | *A26     A10* | 22  A8 PWM                    Y_ENABLE_PIN
	  X_STEP_PIN          SDA2 CAN0RX  PWM  4 | *A25     A11* | 21  A7 PWM  CS0   MOSI1  RX1
	  X_ENABLE_PIN        MISO1   TX1  PWM  5 | *GND * * 57   | 20  A6 PWM  CS0   SCK1
	                                   PWM  6 | *GND * * 56   | 19  A5            SCL0        Y_CS_PIN
      X_CS_PIN      SCL0  MOSI0   RX3  PWM  7 |      * * 55   | 18  A4            SDA0
                    SDA0  MISO0   TX3  PWM  8 |      * * 54   | 17  A3            SDA0
                          CS0     RX2  PWM  9 |               | 16  A2            SCL0        Y_STEP_PIN
                          CS0     TX2  PWM 10 |               | 15  A1      CS0               Y_DIR_PIN
      MOSI_PIN            MOSI0            11 |               | 14  A0 PWM  CS0               SERVO_PIN
      MISO_PIN            MISO0            12 |               | 13 LED            SCK0        SCLK_PIN
                                         3.3V |               | GND
										   24 |   40 * * 53   |    A22 DAC1
										   25 |   41 * * 52   |    A21 DAC0
                          SCL2    TX1      26 |   42 * * 51   | 39 A20      MISO0
						  SCK0    RX1      27 | *  *  *  *  * | 38 A19 PWM        SDA1
                          CAN0TX       PWM 29 |   44 * * 49   | 36 A17 PWM
                          CAN0RX       PWM 30 |   45 * * 48   | 35 A16 PWM
                          CS1     RX4  A12 31 |   46 * * 47   | 34 A15 PWM        SDA0  RX5
                          SCK1    TX4  A13 32 |__GND_*_*_3.3V_| 33 A14 PWM        SCL0  TX5

          Interior pins:
                          *                       40 * * 53   SCK2
                          *                       41 * * 52   MOSI2
                          *                       42 * * 51   MISO2
                          *               CS2     43 * * 50   A24
                          *               MOSI2   44 * * 49   A23
                          *               MISO2   45 * * 48   TX6   SDA0  BTN_ENC
                          BTN_EN1         SCK2    46 * * 47   RX6   SCL0  BTN_EN2
                                                 GND * * 3.3V

*/

#define X_STEP_PIN         4
#define X_DIR_PIN          3
#define X_ENABLE_PIN       5
#define X_CS_PIN           7	// for TMC2130

#define Y_STEP_PIN         16
#define Y_DIR_PIN          15
#define Y_ENABLE_PIN       22
#define Y_CS_PIN           19	// for TMC2130

#define X_STOP_PIN         2
#define Y_STOP_PIN         23

#define SDSS               BUILTIN_SDCARD //39 // 8
//#define LED_PIN            13
#define PS_ON_PIN           1
#define ALARM_PIN          -1
#define SERVO0_PIN			14


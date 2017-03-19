/****************************************************************************************
* Teensy 3.1 Breadboard pin assignments (MK20DX256)
* Requires the Teensyduino software with Teensy 3.1 selected in Arduino IDE!
  http://www.pjrc.com/teensy/teensyduino.html
* CLI build: HARDWARE_MOTHERBOARD=85  make
* 
****************************************************************************************/
#if MOTHERBOARD == 841
#define KNOWN_BOARD 1
#define AT90USB 1286  // Disable MarlinSerial etc.

#ifndef __MK64FX512__
  #error Oops!  Make sure you have 'Teensy 3.5' selected from the 'Tools -> Boards' menu.
#endif

#define BOARD_NAME "Teensy3.5"

#define LARGE_FLASH        true
#define USBCON //1286  // Disable MarlinSerial etc.

/* 
teemuatlut plan for Teensy3.5 and Teensy3.6:

                                                     USB
                                          GND |-----#####-----| VIN 5V
      X_STEP_PIN          MOSI1   RX1       0 |     #####     | Analog GND
      X_DIR_PIN           MISO1   TX1       1 |               | 3.3V
      Y_STEP_PIN                       PWM  2 | *NC     AREF* | 23  A9 PWM
      Y_DIR_PIN           SCL2 CAN0TX  PWM  3 | *A26     A10* | 22  A8 PWM
      Z_STEP_PIN          SDA2 CAN0RX  PWM  4 | *A25     A11* | 21  A7 PWM  CS0   MOSI1  RX1
      Z_DIR_PIN           MISO1   TX1  PWM  5 | *GND * * 57   | 20  A6 PWM  CS0   SCK1
      X_ENABLE_PIN                     PWM  6 | *GND * * 56   | 19  A5            SCL0        E0_STEP_PIN
      Y_ENABLE_PIN  SCL0  MOSI0   RX3  PWM  7 |      * * 55   | 18  A4            SDA0        E0_DIR_PIN
      Z_ENABLE_PIN  SDA0  MISO0   TX3  PWM  8 |      * * 54   | 17  A3            SDA0        E0_ENABLE_PIN
                          CS0     RX2  PWM  9 |               | 16  A2            SCL0        TEMP_0_PIN
                          CS0     TX2  PWM 10 |               | 15  A1      CS0
      X_STOP_PIN          MOSI0            11 |               | 14  A0 PWM  CS0
      Y_STOP_PIN          MISO0            12 |               | 13 LED            SCK0        LED_PIN
                                         3.3V |               | GND
      Z_STOP_PIN                           24 |   40 * * 53   |    A22 DAC1
AUX2                                       25 |   41 * * 52   |    A21 DAC0
AUX2  FAN_PIN             SCL2    TX1      26 |   42 * * 51   | 39 A20      MISO0 SDSS
AUX2  Z-PROBE PWR         SCK0    RX1      27 | *  *  *  *  * | 38 A19 PWM        SDA1
AUX2  SLED_PIN            MOSI0            28 |   43 * * 50   | 37 A18 PWM        SCL1
D10   CONTROLLERFAN_PIN   CAN0TX       PWM 29 |   44 * * 49   | 36 A17 PWM
D9    HEATER_0_PIN        CAN0RX       PWM 30 |   45 * * 48   | 35 A16 PWM
                          CS1     RX4  A12 31 |   46 * * 47   | 34 A15 PWM        SDA0  RX5
                          SCK1    TX4  A13 32 |__GND_*_*_3.3V_| 33 A14 PWM        SCL0  TX5

      Interior E4: 36, INT4
      Interior E5: 37, INT5
      Interior PA0-7: 28-35  -- Printrboard and Teensylu use these pins for step & direction:
             T++ PA Signal  Marlin
    
       Z STEP  32 a4  a0 28 X STEP
       Z DIR   33 a5  a1 29 X DIR
       E STEP  34 a6  a2 30 Y STEP
       E DIR   35 a7  a3 31 Y DIR

                          Interior pins:          40 * * 53   SCK2
                                                  41 * * 52   MOSI2
                                                  42 * * 51   MISO2
                                          CS2     43 * * 50   A24
                                          MOSI2   44 * * 49   A23
                                          MISO2   45 * * 48   TX6   SDA0
                                          SCK2    46 * * 47   RX6   SCL0
                                                 GND * * 3.3V

*/

#define X_STEP_PIN          22
#define X_DIR_PIN           21
#define X_ENABLE_PIN        39

#define Y_STEP_PIN          19
#define Y_DIR_PIN           18
#define Y_ENABLE_PIN        20

#define Z_STEP_PIN          38
#define Z_DIR_PIN           37
#define Z_ENABLE_PIN        17

#define E0_STEP_PIN        31
#define E0_DIR_PIN         30
#define E0_ENABLE_PIN      32

#define E1_STEP_PIN        -1
#define E1_DIR_PIN         -1
#define E1_ENABLE_PIN      -1

#define HEATER_0_PIN        3
#define HEATER_1_PIN       -1
#define HEATER_2_PIN       -1
#define HEATER_BED_PIN     -1
#define FAN_PIN             2

#define X_STOP_PIN         24
#define Y_STOP_PIN         26
#define Z_STOP_PIN         28

#define TEMP_0_PIN          2 // Extruder / Analog pin numbering: 2 => A2
#define TEMP_BED_PIN       -1 // Bed / Analog pin numbering
#define TEMP_1_PIN         -1
#define TEMP_2_PIN         -1

#define SDPOWER            -1
#define SD_DETECT_PIN      -1
#define SDSS               39 // 8
#define LED_PIN            13
#define PS_ON_PIN           1
#define KILL_PIN           -1
#define ALARM_PIN          -1

#define FILWIDTH_PIN       -1
#define SLED_PIN            5

#ifndef SDSUPPORT
// these pins are defined in the SD library if building with SD support
  #define SCK_PIN          13
  #define MISO_PIN         12
  #define MOSI_PIN         11
#endif
/*
#ifdef ULTRA_LCD
#define LCD_PINS_RS         8
#define LCD_PINS_ENABLE     9
#define LCD_PINS_D4        10
#define LCD_PINS_D5        11
#define LCD_PINS_D6        12
#define LCD_PINS_D7        13
#define BTN_EN1            38
#define BTN_EN2            39
#define BTN_ENC            40
#endif
*/
#endif  // MOTHERBOARD == 84 (Teensy++2.0 Breadboard)

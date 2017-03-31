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

#ifndef SIMPLE_LCD_H
#define SIMPLE_LCD_H

//#define LCD_CONSTRUCTOR U8G2_ST7920_128X64_F_SW_SPI u8g2(U8G2_R0, /* clock=*/ 23, /* data=*/ 17, /* CS=*/ 16, /* reset=*/ 4);
#define LCD_CONSTRUCTOR U8G2_UC1608_ERC24064_F_3W_SW_SPI u8g2(U8G2_R0, /*clock*/ 34, /*data*/ 36, /*cs*/ 53, /*[, reset]*/ 49);
#define LCD_FONT u8g2_font_ncenR08_tr // https://github.com/olikraus/u8g2/wiki/fntgrp

#define SHOW_XYZ_POS_H
//#define SHOW_XYZ_POS_V
#define COORDINATE_POS_XY 15, 11
#define SPACING 40 // For SHOW_XYZ_POS_H

#define E0_XY 	26,38 // Coordinates: X, Y. Comment to disable
#define E0_STYLE 	2 // 0, 1, 2, 3, 4
#define E0_DIM1	    9 // Inner diameter of the graphic
#define E0_DIM2    20

#define BED_XY 102,38
#define BED_STYLE 	2 // 0, 1, 2
#define BED_DIM1    9
#define BED_DIM2   20

#define SHOW_PROGRESS
#define PROGRESS_XY 64, 32
#define PROGRESS_STYLE 1 // 0, 1
#define BAR_H 8
#define BAR_L 40
#define RING_R1 9
#define RING_R2 15

#define STATUS_MSG_XY 7,60

// Primitives
//#define FRAME {5,5,123,59},{10,10,118,54} // x1, y1, x2, y2
#define ROUNDED_FRAME {0,0,128,64,5} // x1, y1, x2, y2, corner_radius
#define LINE {2,13,125,13},{2,45,53,45},{125,45,75,45} // x1, y1, x2, y2

void lcd_init();
void lcd_update();
void lcd_setStatus();


#endif // SIMPLE_LCD_H
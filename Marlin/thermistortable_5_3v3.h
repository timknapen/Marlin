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

// 100k ParCan thermistor (104GT-2)
// ATC Semitec 104GT-2 (Used in ParCan)
// Verified by linagee. Source: http://shop.arcol.hu/static/datasheets/thermistors.pdf
// Calculated using 4.7kohm pullup, voltage divider math, and manufacturer provided temp/resistance
// For Teensy3.5 with 3.3 VCC
const short temptable_5[][2] PROGMEM = {    
  {    1 * OVERSAMPLENR,  713 },
  {   17 * OVERSAMPLENR,  300 },
  {   20 * OVERSAMPLENR,  290 },
  {   23 * OVERSAMPLENR,  280 },
  {   27 * OVERSAMPLENR,  270 },
  {   31 * OVERSAMPLENR,  260 },
  {   37 * OVERSAMPLENR,  250 },
  {   43 * OVERSAMPLENR,  240 },
  {   51 * OVERSAMPLENR,  230 },
  {   61 * OVERSAMPLENR,  220 },
  {   73 * OVERSAMPLENR,  210 },
  {   88 * OVERSAMPLENR,  200 },
  {  106 * OVERSAMPLENR,  190 },
  {  128 * OVERSAMPLENR,  180 },
  {  155 * OVERSAMPLENR,  170 },
  {  189 * OVERSAMPLENR,  160 },
  {  230 * OVERSAMPLENR,  150 },
  {  279 * OVERSAMPLENR,  140 },
  {  336 * OVERSAMPLENR,  130 },
  {  402 * OVERSAMPLENR,  120 },
  {  476 * OVERSAMPLENR,  110 },
  {  555 * OVERSAMPLENR,  100 },
  {  635 * OVERSAMPLENR,   90 },
  {  713 * OVERSAMPLENR,   80 },
  {  785 * OVERSAMPLENR,   70 },
  {  847 * OVERSAMPLENR,   60 },
  {  898 * OVERSAMPLENR,   50 },
  {  938 * OVERSAMPLENR,   40 },
  {  967 * OVERSAMPLENR,   30 },
  {  987 * OVERSAMPLENR,   20 },
  { 1001 * OVERSAMPLENR,   10 },
  { 1011 * OVERSAMPLENR,    0 }
};  

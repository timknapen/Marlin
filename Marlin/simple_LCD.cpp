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

#include "Marlin.h"

#if ENABLED(SIMPLE_LCD)

#include <U8g2lib.h>
#include "simple_LCD.h"
#include "macros.h"
#include "temperature.h"
#include "utility.h"

LCD_CONSTRUCTOR

static const uint8_t cosine5deg[] { 100,100,98,97,94,91,87,82,77,71,64,57,50,42,34,26,17,9,0 };

#define DOUBLE_TAP uiEditValue
#define LONG_PRESS uiFunction

#define MIDDLE_Y (64/2)

#define READ_PIN(pin) ((pin ##_port & pin ## _bm) >> pin ## _bp)

static bool A, A_1 = 0,
						B, B_1 = 0,
						btn,
						btn_1 = HIGH,
						blink = false;
static int8_t count = 0;
static millis_t btnDown_1, btnDuration, btnInterval, lastAction, next_lcd_update_ms;
static uint16_t clearBufferTime, drawListTime, uiFunctionTime, sendBufferTime;
int8_t myVal = 33;

struct node {
	char *str = NULL;
	node **target = NULL;
	node *next = NULL;
	node *prev = NULL;
	void 			(*uiFunction)() 		= NULL;
	uint8_t 	(*getVal_uint8)() 	= NULL;
	uint16_t 	(*getVal_uint16)()	= NULL;
	int8_t 		(*getVal_int8)() 		= NULL;
	int16_t 	(*getVal_int16)() 	= NULL;
	float 		(*getVal_float)()		= NULL;
	bool 			(*getVal_bool)()		= NULL;
};

node	**frame,
			*statusScreen = new node,
			*rootMainMenu = new node,
			*rootSubMenu1 = new node,
			*rootSubMenu2 = new node,
			*rootSubMenu3 = new node,
			*rootSubMenu4 = new node,
			*rootSubMenu5 = new node,
			*mainMenu = rootMainMenu,
			*subMenu1 = rootSubMenu1,
			*subMenu2 = rootSubMenu2,
			*subMenu3 = rootSubMenu3,
			*subMenu4 = rootSubMenu4,
			*subMenu5 = rootSubMenu5,
			*uiFunction = new node,
			*uiEditValue = new node;

#define _w_ext 11
#define _h_ext 12
static const unsigned char _ext_bmp[] = {
	0xfe,0x03,0xff,0x07,0xff,0x07,0xff,0x07,0xfe,0x03,0xfe,0x03,0xff,0x07,0xff,
	0x07,0xff,0x07,0xfc,0x01,0xf8,0x00,0x70,0x00 };

#define _w_bed 9
#define _h_bed 12
static const unsigned char _bed_bmp[] = {
	0x92,0x00,0x24,0x01,0x24,0x01,0x24,0x01,0x92,0x00,0x49,0x00,0x49,0x00,0x49,
	0x00,0x92,0x00,0x00,0x00,0xff,0x01,0xff,0x01 };

static const unsigned char _bed_bmp_flip[] = {
	0x92,0x00,0x49,0x00,0x49,0x00,0x49,0x00,0x92,0x00,0x24,0x01,0x24,0x01,0x24,
	0x01,0x92,0x00,0x00,0x00,0xff,0x01,0xff,0x01 };

int16_t getCosine(int16_t a) {
	int16_t _a;

	if 			(a <=  90) _a = a;
	else if (a <= 180) _a = 180 - a;
	else if (a <= 270) _a = a - 180;
	else if (a <= 360) _a = 360 - a;

	if (a > 90 && a <= 270)	return -cosine5deg[_a  / 5 ];
	return cosine5deg[_a  / 5 ];
}

int16_t getSine(uint16_t a) {
	int16_t _a;
	if 			(a <=  90) _a = a;
	else if (a <= 180) _a = 180 - a;
	else if (a <= 270) _a = -(a - 180);
	else if (a <= 360) _a = -(360 - a);
	return getCosine(90 - _a);
}

void drawExtTemp(uint16_t x, uint16_t y, uint8_t e, String id) {
	u8g2.setCursor(x, y);
	u8g2.print(id);
	u8g2.setCursor(x + 20, y);
	u8g2.print((uint16_t)thermalManager.degHotend(e));
}

void drawExtruderIcon(uint16_t x, uint16_t y, String id, uint16_t T, uint16_t T_target, bool heat) {
	u8g2.drawXBM(x, y, _w_ext, _h_ext, _ext_bmp);
	int8_t offset = T >= 100 ? -2 : 1;
	u8g2.setCursor(x+offset, y+21);
	u8g2.print(T);
	if (T_target > 0) {
		offset = T_target >= 100 ? -2 : 1;
		u8g2.setCursor(x+offset, y-1);
		u8g2.print(T_target);
	}
	u8g2.setDrawColor(0);
	u8g2.setCursor(x+3, y+9);
	u8g2.print(id);
	if (heat) u8g2.drawPixel(x+5, y+10);
	u8g2.setDrawColor(1);
}

void drawGaugeStyle2(uint16_t x, uint16_t y, uint8_t r, uint8_t r2, uint16_t T, uint8_t a, String id) {
	#define ANGLE 				a
	#define ORIGIN 				x, y
	#define LEFT_LOWER_CORNER 	x-r2-1, y
	#define LEFT_UPPER_CORNER 	x-r2-1, y-r2-1
	#define MIDDLE_POINT 		x, y-r2-1
	#define RIGHT_UPPER_CORNER 	x+r2+1, y-r2-1
	#define RIGHT_LOWER_CORNER 	x+r2+1, y

	u8g2.drawDisc(ORIGIN, r2, U8G2_DRAW_UPPER_RIGHT);
	u8g2.drawDisc(ORIGIN, r2, U8G2_DRAW_UPPER_LEFT);
	u8g2.setDrawColor(0);
	u8g2.drawDisc(ORIGIN, r, U8G2_DRAW_UPPER_RIGHT);
	u8g2.drawDisc(ORIGIN, r, U8G2_DRAW_UPPER_LEFT);
	
	if (ANGLE >= 0 && ANGLE <=45) {
		u8g2.drawTriangle(ORIGIN, LEFT_UPPER_CORNER, x-r2-1, y-(float)ANGLE/45.0*(float)r2);
		u8g2.drawTriangle(ORIGIN, LEFT_UPPER_CORNER, RIGHT_UPPER_CORNER);
		u8g2.drawTriangle(ORIGIN, RIGHT_LOWER_CORNER, RIGHT_UPPER_CORNER);
	}
	else if (ANGLE <= 135) {
		u8g2.drawTriangle(ORIGIN, RIGHT_UPPER_CORNER, (x-r2-1)+(float)(ANGLE-45.0)/90.0*(float)r2*2.0, y-r2-1);
		u8g2.drawTriangle(ORIGIN, RIGHT_LOWER_CORNER, RIGHT_UPPER_CORNER);
	}
	else if (ANGLE <= 180) {
		u8g2.drawTriangle(ORIGIN, RIGHT_LOWER_CORNER, x+r2+1, y-(float)(180-ANGLE)/45.0*(float)r2);
	}

	if (ANGLE < 90) u8g2.setDrawColor(1);
	u8g2.setCursor(x-2, y-r-(r2-r)/2+4);
	u8g2.print(id);

	u8g2.setDrawColor(1);
	u8g2.drawCircle(ORIGIN, r2, U8G2_DRAW_UPPER_LEFT);
	u8g2.drawCircle(ORIGIN, r2, U8G2_DRAW_UPPER_RIGHT);
	u8g2.drawCircle(ORIGIN,  r, U8G2_DRAW_UPPER_LEFT);
	u8g2.drawCircle(ORIGIN,  r, U8G2_DRAW_UPPER_RIGHT);
	u8g2.drawHLine(x+r, y, r2-r);

	int8_t offset_x = T >= 100 ? -7 : -4;
	int8_t offset_y = r >= 9 ? 3 : r >= 7 ? 6 : 10;
	u8g2.setCursor(x+offset_x, y+offset_y);
	u8g2.print(T);
}

void drawGaugeStyle3(uint16_t x, uint16_t y, int8_t l, int8_t h, uint16_t T, float p) {
	#define GAUGE_LEFT_LOWER_CORNER 	x-l/2, y
	#define GAUGE_RIGHT_LOWER_CORNER	x+l/2, y
	#define GAUGE_LEFT_UPPER_CORNER		x-l/2-10, y-h
	#define GAUGE_RIGHT_UPPER_CORNER	x+l/2+10, y-h
	float lowerLineLength = (float)l;
	float upperLineLength = (float)l+20.0;
	u8g2.drawLine(GAUGE_LEFT_LOWER_CORNER,  GAUGE_RIGHT_LOWER_CORNER); // Lower line
	u8g2.drawLine(GAUGE_LEFT_UPPER_CORNER,  GAUGE_RIGHT_UPPER_CORNER); // Upper line
	u8g2.drawLine(GAUGE_LEFT_LOWER_CORNER,  GAUGE_LEFT_UPPER_CORNER ); // Left line
	u8g2.drawLine(GAUGE_RIGHT_LOWER_CORNER, GAUGE_RIGHT_UPPER_CORNER); // Right line

	u8g2.drawLine(x-l/2+p*lowerLineLength, y, x-l/2-10+p*upperLineLength, y-h);
}

void drawGaugeStyle4(uint16_t x, uint16_t y, uint8_t r1, uint8_t r2, uint16_t T, uint16_t T_target, String id) {
	uint8_t a = T;

	u8g2.drawCircle(x, 		y, 		r2, 	U8G2_DRAW_UPPER_LEFT);
	u8g2.drawCircle(x, 		y, 		r2, 	U8G2_DRAW_UPPER_RIGHT);
	u8g2.drawCircle(x, 		y, 		r1, 	U8G2_DRAW_UPPER_LEFT);
	u8g2.drawCircle(x, 		y, 		r1, 	U8G2_DRAW_UPPER_RIGHT);
	u8g2.drawDisc  (x, 		y, 		3, 		U8G2_DRAW_ALL);
	u8g2.drawHLine (x-20, y, 		r2-r1);
	u8g2.drawHLine (x+10, y, 		r2-r1);
	u8g2.drawLine  (x, 		y, 		x-(float)getCosine(a)/100.0*(r2-2), y-(float)getSine(a)/100.0*(r2-2));
	int8_t offset = T >= 100 ? -r1-12 : -r1-8;
	u8g2.setCursor (x+offset, y+10);
	u8g2.print		 (T);
	if (T_target > 0) {
		offset = T_target >= 100 ? r1 : r1+4;
		u8g2.setCursor (x+offset, 	y+10);
		u8g2.print 		 (T_target);
	}
	u8g2.setCursor (x-5,	y+13);
	u8g2.print 		 (id);
}

void drawGaugeStyle5(uint16_t x, uint16_t y, int8_t r, int8_t r2, uint8_t progress) {
	uint16_t a = map(progress, 0, 100, 0, 360);

	uint16_t _a = a < 90 ? a - 90 + 360 : a - 90; // Rotate ring

	uint8_t px = x - (float)getCosine(_a)/100.0*r2;
	uint8_t py = y - (float)getSine(_a)/100.0*r2;

	#define ANGLE 	a
	#define ORIGIN 	x, y
	#define LEFT 	x-r2, y
	#define UP 		x, y-r2-1
	#define RIGHT 	x+r2+1, y
	#define DOWN 	x, y+r2
	#define POINT 	px, py
	#define Q1		U8G2_DRAW_LOWER_LEFT
	#define Q2		U8G2_DRAW_UPPER_LEFT
	#define Q3		U8G2_DRAW_UPPER_RIGHT
	#define QALL	U8G2_DRAW_ALL

	if (ANGLE == 0) {
//		u8g2.drawLine(ORIGIN, LEFT);
	} else if (ANGLE <= 90) {
		u8g2.drawDisc(ORIGIN, r2, Q1);
	} else if (ANGLE <= 180) {
		u8g2.drawDisc(ORIGIN, r2, Q1);
		u8g2.drawDisc(ORIGIN, r2, Q2);
	} else if (ANGLE <= 270) {
		u8g2.drawDisc(ORIGIN, r2, Q1);
		u8g2.drawDisc(ORIGIN, r2, Q2);
		u8g2.drawDisc(ORIGIN, r2, Q3);
	} else if (ANGLE <= 360) {
		u8g2.drawDisc(ORIGIN, r2, QALL);
	}
	u8g2.setDrawColor(0);
	u8g2.drawDisc(ORIGIN, r, QALL);

	if (ANGLE != 0 && ANGLE < 90) {
		u8g2.drawTriangle(	ORIGIN, 	LEFT, 	px, py);
		u8g2.drawTriangle(	x-r2, py, 	LEFT, 	px, py);
	} else if (ANGLE != 90 && ANGLE < 180) {
		u8g2.drawTriangle(	ORIGIN, 	UP, 	px, py);
		u8g2.drawTriangle(	px, y-r2,	UP, 	px, py);
		u8g2.drawLine(		ORIGIN, 	UP);
	} else if (ANGLE != 180 && ANGLE < 270) {
		u8g2.drawTriangle(	ORIGIN,		RIGHT, 	px, py);
		u8g2.drawTriangle(	x+r2, py,	RIGHT, 	px, py);
		u8g2.drawLine(		ORIGIN, 	RIGHT);
	} else if (ANGLE != 270 && ANGLE < 360) {
		u8g2.drawTriangle(	ORIGIN,		DOWN,  	px, py);
		u8g2.drawTriangle(	px, y+r2, 	DOWN,  	px, py);
	}


	u8g2.setDrawColor(1);
	u8g2.drawLine(DOWN, x, y+r);
	u8g2.drawCircle(ORIGIN, r2, QALL);
	u8g2.drawCircle(ORIGIN,  r, QALL);

	int8_t offset_x = progress == 100 ? -7 : progress >= 10 ? -4 : -2;
	u8g2.setCursor(x+offset_x, y+5);
	u8g2.print(progress);
}

void drawStatusScreen() {
	u8g2.clearBuffer();
	blink = !blink;
	#if defined(E0_XY)
	{
		uint16_t T = thermalManager.degHotend(0);
		uint16_t T_target = thermalManager.degTargetHotend(0);

		#if E0_STYLE==0
			String id = "E0";
			drawExtTemp(E0_XY, 0, id);
		#elif E0_STYLE==1
			String id = "1";
			drawExtruderIcon(E0_XY, id, T, T_target, thermalManager.isHeatingHotend(0));
		#elif E0_STYLE==2
			String id = "1";
			uint8_t a = map(T, HEATER_0_MINTEMP, HEATER_0_MAXTEMP, 0, 180);
			drawGaugeStyle2(E0_XY, E0_DIM1, E0_DIM2, T, a, id);
		#elif E0_STYLE==3
			float p = (float)map(T, HEATER_0_MINTEMP, HEATER_0_MAXTEMP, 0, 100)/100.0;
			drawGaugeStyle3(E0_XY, E0_DIM1, E0_DIM2, T, p);
		#elif E0_STYLE==4
			String id = "E0";
			drawGaugeStyle4(E0_XY, E0_DIM1, E0_DIM2, T, T_target, id);
		#endif
	}
	#endif

	#if defined(BED_XY)
	{
		uint8_t xy[] = {BED_XY};
		uint16_t T = thermalManager.degBed();
		uint16_t T_target = thermalManager.degTargetBed();

		#if BED_STYLE==0
			u8g2.drawStr(BED_XY, "Bed");
			u8g2.setCursor(xy[0]+20, xy[1]);
			u8g2.print(T);
		#elif BED_STYLE==1
			int8_t offset = T >= 100 ? -3 : 1;

			if (blink && T_target > 0) 	u8g2.drawXBM(BED_XY, _w_bed, _h_bed, _bed_bmp);
			else 						u8g2.drawXBM(BED_XY, _w_bed, _h_bed, _bed_bmp_flip);

			u8g2.setCursor(xy[0]+offset, xy[1]+21);
			u8g2.print(T);
			
			if (T_target > 0) {
				offset = T_target >= 100 ? -3 : 1;
				u8g2.setCursor(xy[0]+offset, xy[1]-1);
				u8g2.print(T_target);
			}
			if (thermalManager.isHeatingBed()) {
				u8g2.setDrawColor(0);
				u8g2.drawVLine(xy[0]+4, xy[1]+_h_bed-2, 2);
				u8g2.setDrawColor(1);
			}
		#elif BED_STYLE==2
			String id = "B";
			uint8_t a = map(T, BED_MINTEMP, BED_MAXTEMP, 0, 180);
			drawGaugeStyle2(BED_XY, BED_DIM1, BED_DIM2, T, a, id);
		#endif
		}
	#endif

	#if defined(SHOW_XYZ_POS_H)
	{
		const uint8_t xy[] = {COORDINATE_POS_XY};
		const uint8_t spacing = SPACING;
		u8g2.drawStr(xy[0], 			xy[1],	"X:");
		u8g2.drawStr(xy[0]+spacing, 	xy[1],	"Y:");
		u8g2.drawStr(xy[0]+2*spacing, 	xy[1], 	"Z:");
		u8g2.drawStr(xy[0]+11, 			xy[1],	axis_homed[X_AXIS] ? itostr3(current_position[X_AXIS]) : ( blink ? "?" : "X"));
		u8g2.drawStr(xy[0]+spacing+11, 	xy[1], 	axis_homed[Y_AXIS] ? itostr3(current_position[Y_AXIS]) : ( blink ? "?" : "Y"));
		u8g2.drawStr(xy[0]+2*spacing+11,xy[1], 	axis_homed[Z_AXIS] ? itostr3(current_position[Z_AXIS] + 0.00001) : ( blink ? "?" : "Z"));
	}
	#endif
	#if defined(SHOW_XYZ_POS_V)
	{
		uint8_t xy[] = {COORDINATE_POS_XY};
		u8g2.drawStr(xy[0], 	xy[1], 		"X:");
		u8g2.drawStr(xy[0]-1, 	xy[1]+10, 	"Y:");
		u8g2.drawStr(xy[0]+1, 	xy[1]+20, 	"Z:");
		u8g2.drawStr(xy[0]+13, 	xy[1], 		axis_homed[X_AXIS] ? itostr3(current_position[X_AXIS]) : ( blink ? "?" : "X"));
		u8g2.drawStr(xy[0]+13, 	xy[1]+10, 	axis_homed[Y_AXIS] ? itostr3(current_position[Y_AXIS]) : ( blink ? "?" : "Y"));
		u8g2.drawStr(xy[0]+13, 	xy[1]+20, 	axis_homed[Z_AXIS] ? itostr3(current_position[Z_AXIS] + 0.00001) : ( blink ? "?" : "Z"));
	}
	#endif

	#if defined(PROGRESS_XY)
	{
		#if ENABLED(SDSUPPORT)
			uint8_t prog = IS_SD_PRINTING ? card.percentDone() : 0;
		#else
			uint8_t prog = 0;
		#endif
		
		uint8_t xy[] = {PROGRESS_XY};
		#if PROGRESS_STYLE == 0
			u8g2.drawFrame(xy[0]+10, xy[1], BAR_L, BAR_H);
			u8g2.drawBox(xy[0]+10, xy[1], prog, BAR_H);
			u8g2.setCursor(xy[0], xy[1]+8);
			u8g2.print(prog);
		#elif PROGRESS_STYLE == 1
			drawGaugeStyle5(xy[0], xy[1], RING_R1, RING_R2, prog);
		#endif
	}
	#endif

	#if defined(FRAME)
	{
		uint16_t target[][4] = {FRAME};
		for (uint8_t rect = 0; rect < sizeof(target)/8; rect++) {
			u8g2.drawFrame(	target[rect][0],
							target[rect][1],
							target[rect][2] - target[rect][0],
							target[rect][3] - target[rect][1]);
		}
	}
	#endif
	#if defined(ROUNDED_FRAME)
	{
		uint16_t target[][5] = {ROUNDED_FRAME};
		for (uint8_t rect = 0; rect < sizeof(target)/10; rect++) {
			u8g2.drawRFrame(target[rect][0],
							target[rect][1],
							target[rect][2] - target[rect][0],
							target[rect][3] - target[rect][1],
							target[rect][4]);
		}
	}
	#endif
	#if defined(LINE)
	{
		uint16_t target[][4] = {LINE};
		for (uint8_t rect = 0; rect < sizeof(target)/8; rect++) {
			u8g2.drawLine(	target[rect][0],
							target[rect][1],
							target[rect][2],
							target[rect][3]);
		}
	}
	#endif

	#if defined(STATUS_MSG_XY)
	{
		u8g2.setCursor(STATUS_MSG_XY);
		u8g2.print("lcd_status_message");
	}
	#endif
}

void drawUIElement() {
	static int8_t r = 10;
	static int8_t count_1 = count;
	if (count_1 + 1 < count) {
		if (r<30) r++;
		count_1 = count;
	} else if (count < count_1 - 1) {
		if (r>1) r--;
		count_1 = count;
	}

	u8g2.drawDisc(60,20,r);
}

int8_t returnVal() { return myVal; }

void drawEditMyVal() {
	static int8_t count_1 = count;
	if (count_1 + 1 < count) {
		myVal++;
		count_1 = count;
	} else if (count < count_1 - 1) {
		myVal--;
		count_1 = count;
	}
	constexpr int8_t y_offset = 2;
	int8_t	size = 6;
	uint8_t pos_x = column2_x+11,
					pos_y = MIDDLE_Y+7;
	u8g2.setCursor(column2_x,pos_y);
	u8g2.print(myVal);
	u8g2.drawTriangle(	pos_x-size,	pos_y+y_offset,
											pos_x+size,	pos_y+y_offset,
											pos_x,			pos_y+y_offset+size);
	u8g2.drawTriangle(	pos_x-size,	pos_y-FONT_SIZE-y_offset,
											pos_x+size,	pos_y-FONT_SIZE-y_offset,
											pos_x,			pos_y-FONT_SIZE-y_offset-size);
}

void addMenuItem(char str[], node **target = NULL) {
	if ((*frame)->str == NULL) {
		(*frame)->str = str;
		(*frame)->target = target;
	} else {
		(*frame)->next = new node;
		node *tmp = *frame;
		*frame = (*frame)->next;
		(*frame)->prev = tmp;
		(*frame)->str = str;
		(*frame)->target = target;
	}
}

void populateMenu() {
	frame = &mainMenu;
	addMenuItem("Return", &statusScreen);
	addMenuItem("Sub1", &subMenu1);
	addMenuItem("Sub2", &subMenu2);
	#if defined(HAS_MAIN_ITEM_3)
		addMenuItem("Sub3", &subMenu3);
	#endif
	addMenuItem("Sub4", &subMenu4);
	addMenuItem("Sub5", &subMenu5);
	addMenuItem("Main5");
	addMenuItem("Main6");
	mainMenu = rootMainMenu;

	frame = &subMenu1;
	addMenuItem("Return...", &mainMenu);
	addMenuItem("Sub1_1");
	addMenuItem("Sub1_2");
	addMenuItem("Sub1_3");
	#if defined(HAS_SUB_ITEM_4)
		addMenuItem("Sub1_4", &uiFunction);
	#endif
	addMenuItem("Sub1_5", &uiEditValue); (*frame)->getVal_int8 = returnVal;
	addMenuItem("Sub1_6");
	addMenuItem("Sub1_7");
	addMenuItem("Sub1_8");
	subMenu1 = rootSubMenu1;

	frame = &subMenu2;
	addMenuItem("Return...", &mainMenu);
	addMenuItem("Sub2_1");
	addMenuItem("Sub2_2");
	addMenuItem("Sub2_3");
	addMenuItem("Sub2_4");
	addMenuItem("Sub2_5");
	addMenuItem("Sub2_6");
	addMenuItem("Sub2_7");
	addMenuItem("Sub2_8");
	subMenu2 = rootSubMenu2;

	frame = &subMenu3;
	addMenuItem("Return...", &mainMenu);
	addMenuItem("Sub3_1");
	addMenuItem("Sub3_2");
	addMenuItem("Sub3_3");
	addMenuItem("Sub3_4");
	addMenuItem("Sub3_5");
	addMenuItem("Sub3_6");
	addMenuItem("Sub3_7");
	addMenuItem("Sub3_8");
	subMenu3 = rootSubMenu3;

	frame = &subMenu4;
	addMenuItem("Return...", &mainMenu);
	addMenuItem("Sub4_1");
	addMenuItem("Sub4_2");
	addMenuItem("Sub4_3");
	addMenuItem("Sub4_4");
	addMenuItem("Sub4_5");
	addMenuItem("Sub4_6");
	addMenuItem("Sub4_7");
	addMenuItem("Sub4_8");
	subMenu4 = rootSubMenu4;

	frame = &subMenu5;
	addMenuItem("Return...", &mainMenu);
	addMenuItem("Sub5_1");
	addMenuItem("Sub5_2");
	addMenuItem("Sub5_3");
	addMenuItem("Sub5_4");
	addMenuItem("Sub5_5");
	addMenuItem("Sub5_6");
	addMenuItem("Sub5_7");
	addMenuItem("Sub5_8");
	subMenu5 = rootSubMenu5;

	frame = &statusScreen;addMenuItem("Start", 	&mainMenu);	(*frame)->uiFunction = drawStatusScreen;
	frame = &uiFunction;	addMenuItem("Fun1", 	&subMenu1); (*frame)->uiFunction = drawUIElement;
	frame = &uiEditValue;	addMenuItem("Val", 		&subMenu1); (*frame)->uiFunction = drawEditMyVal;
}

inline void handle_change() {
	A = READ_PIN(CLK);
	B = READ_PIN(DT);

			 if (A == HIGH && A_1 == LOW )	B ? count++ : count--;
	else if (A == LOW  && A_1 == HIGH)	B ? count-- : count++;
	//else if (B == HIGH && B_1 == LOW )	A ? count-- : count++;
	//else if (B == LOW  && B_1 == HIGH)	A ? count++ : count--;

	A_1 = A;
	B_1 = B;
}

inline void printListItem(node *item, uint8_t pos_x, uint8_t pos_y) {
	if (item != NULL) {
		u8g2.setCursor(pos_x, pos_y);	u8g2.print(item->str);
		u8g2.setCursor(column2_x, pos_y);
				 if (item->getVal_bool		!= NULL) u8g2.print(item->getVal_bool());
		else if (item->getVal_uint8 	!= NULL) u8g2.print(item->getVal_uint8());
		else if (item->getVal_uint16 	!= NULL) u8g2.print(item->getVal_uint16());
		else if (item->getVal_float		!= NULL) u8g2.print(item->getVal_float());
		else if (item->getVal_int8 		!= NULL) u8g2.print(item->getVal_int8());
		else if (item->getVal_int16 	!= NULL) u8g2.print(item->getVal_int16());
	}
}

void drawList() {
	constexpr int8_t	size = 6,
										y_offset = -6;
	uint8_t pos_x = 2,
					pos_y = MIDDLE_Y+7;
	printListItem(*frame, 		 	 12, pos_y);
	printListItem((*frame)->prev, 8, pos_y-FONT_SIZE-2);
	printListItem((*frame)->next, 8, pos_y+FONT_SIZE+2);
	node *tmp = (*frame)->next;
	if (tmp != NULL) printListItem(tmp->next, 8, pos_y+2*(FONT_SIZE+2));
	tmp = (*frame)->prev;
	if (tmp != NULL) printListItem(tmp->prev, 8, pos_y-2*(FONT_SIZE+2));
	if ((*frame)->uiFunction == NULL)
		u8g2.drawTriangle(pos_x,pos_y+size+y_offset, pos_x,pos_y-size+y_offset, pos_x+size,pos_y+y_offset);
	//u8g2.setCursor(10, 10);	u8g2.print("test");
}

void lcd_init() {
	pinMode(CLK, INPUT_PULLUP);
	pinMode(DT, INPUT_PULLUP);
	pinMode(SW, INPUT_PULLUP);
	u8g2.begin();
	u8g2.setFont(LCD_FONT);
	//Serial.begin(250000);
	populateMenu();
	btnDown_1 = lastAction = millis();
	btn_1 = !READ_PIN(SW);
	frame = &statusScreen;
	u8g2.clearBuffer();
	if ((*frame)->uiFunction != NULL) (*frame)->uiFunction();
	u8g2.sendBuffer();
}

void lcd_update() {
	static int8_t count_1 = count;
	bool updateFrame = false;
	uint32_t ms = millis();
	handle_change();
	if (count_1 + 1 < count) {
		if ((*frame)->next != NULL) *frame = (*frame)->next;
		count_1 = count;
		updateFrame = true;
		lastAction = ms;
	} else if (count < count_1 - 1) {
		if ((*frame)->prev != NULL) *frame = (*frame)->prev;
		count_1 = count;
		updateFrame = true;
		lastAction = ms;
	}

	btn = !READ_PIN(SW);
	if (btn != btn_1) {
		if (btn) {
			btnInterval = ms - btnDown_1;
			btnDown_1 = ms;
			//u8g2.drawFrame(0,0,127,63);
			if ((*frame)->target == &statusScreen && btnInterval < 250) frame = &DOUBLE_TAP;
			else if ((*frame)->target != NULL) frame = (*frame)->target;
		} else {
			btnDuration = ms - btnDown_1;
			if ((*frame)->target == &statusScreen && btnDuration > 1500) frame = &LONG_PRESS;
			updateFrame = true;
		}

		lastAction = ms;
		btn_1 = btn;
	}

	if (frame != &statusScreen && (ms - lastAction) > LCD_TIMEOUT) { frame = &statusScreen; updateFrame = true; }

	if (updateFrame) {
		ms = millis();
	  u8g2.clearBuffer();
	  clearBufferTime = millis() - ms; ms = millis();
  	drawList();
  	drawListTime = millis() - ms;	ms = millis();
  	if ((*frame)->uiFunction != NULL) (*frame)->uiFunction();
  	uiFunctionTime = millis() - ms; ms = millis();
		u8g2.sendBuffer();
		sendBufferTime = millis() - ms;
	}

	if (ms - next_lcd_update_ms >= 1000) {
		next_lcd_update_ms = ms;
		SERIAL_ECHO("Total "); SERIAL_ECHO(clearBufferTime + drawListTime + uiFunctionTime + sendBufferTime);
		SERIAL_ECHO("\tclearBufferTime "); 	SERIAL_ECHO(clearBufferTime);
		SERIAL_ECHO("\tdrawListTime "); 		SERIAL_ECHO(drawListTime);
		SERIAL_ECHO("\tuiFunctionTime "); 	SERIAL_ECHO(uiFunctionTime);
		SERIAL_ECHO("\tsendBufferTime "); 	SERIAL_ECHO(sendBufferTime);
		SERIAL_EOL;
	}
}


#endif // SIMPLE_LCD

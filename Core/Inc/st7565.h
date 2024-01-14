/*
 * st7565.h
 *
 *  Created on: 5 сент. 2023 г.
 *      Author: Kuprin_IV
 */

#ifndef INC_ST7565_H_
#define INC_ST7565_H_

#include "main.h"
#include "fonts.h"
#include "load_control.h"

#define LCDWIDTH 					128
#define LCDHEIGHT 					64

#define CMD_DISPLAY_OFF   			0xAE
#define CMD_DISPLAY_ON    			0xAF

#define CMD_SET_DISP_START_LINE  	0x40
#define CMD_SET_PAGE  				0xB0

#define CMD_SET_COLUMN_UPPER  		0x10
#define CMD_SET_COLUMN_LOWER  		0x00

#define CMD_SET_ADC_NORMAL  		0xA0
#define CMD_SET_ADC_REVERSE 		0xA1

#define CMD_SET_DISP_NORMAL 		0xA6
#define CMD_SET_DISP_REVERSE 		0xA7

#define CMD_SET_ALLPTS_NORMAL 		0xA4
#define CMD_SET_ALLPTS_ON  			0xA5
#define CMD_SET_BIAS_9 				0xA2
#define CMD_SET_BIAS_7 				0xA3

#define CMD_RMW  					0xE0
#define CMD_RMW_CLEAR 				0xEE
#define CMD_INTERNAL_RESET  		0xE2
#define CMD_SET_COM_NORMAL  		0xC0
#define CMD_SET_COM_REVERSE  		0xC8
#define CMD_SET_POWER_CONTROL  		0x28
#define CMD_SET_RESISTOR_RATIO 		0x20
#define CMD_SET_VOLUME_FIRST  		0x81
#define CMD_SET_VOLUME_SECOND  		0
#define CMD_SET_STATIC_OFF  		0xAC
#define CMD_SET_STATIC_ON  			0xAD
#define CMD_SET_STATIC_REG  		0x0
#define CMD_SET_BOOSTER_FIRST  		0xF8
#define CMD_SET_BOOSTER_234  		0
#define CMD_SET_BOOSTER_5  			1
#define CMD_SET_BOOSTER_6  			3
#define CMD_NOP  					0xE3
#define CMD_TEST  					0xF0

#define ST7565_STARTBYTES 			0

typedef enum {AlignLeft, AlignCenter, AlignRight} Align;
typedef enum {Inverted, NotInverted} IsInverted;
typedef enum {Prev, Next, NoAction} Action;

typedef struct
{
	uint8_t x_pos;
    uint8_t y_pos;
    Align align;
    FontInfo font;
    char Text[24];
    IsInverted inverted;
}String,*pString;

typedef struct Window
{
  String strings[7];
  uint8_t StringsQuantity;
  int (*callback)(struct Window* , pData , Action , Action );
  struct Window* next;
  struct Window* prev;
}Window,*pWindow;

typedef struct
{
	void (*displayInit)(void);
	void (*displayReset)(void);
	void (*clearBuffer)(void);
	void (*updateBuffer)(void);
	void (*setBrightness)(uint8_t brightness);
	void (*setStringInBuffer)(String* string);
	void (*setWindow)(pWindow wnd);
	void (*invertRegion)(uint8_t xn, uint8_t yn, uint8_t xk, uint8_t yk);
	void (*drawPixel)(uint8_t x, uint8_t y);
	void (*drawLine)(int8_t xn, int8_t yn, int8_t xk, int8_t yk);
	void (*drawCircle)(uint8_t x, uint8_t y, uint8_t R);
	void (*drawEllipse)(uint8_t x_pos, uint8_t y_pos, uint8_t rad_x, uint8_t rad_y);
	void (*drawRect)(uint8_t x_pos, uint8_t y_pos,uint8_t width, uint8_t height);
	void (*fillRect)(uint8_t x_pos, uint8_t y_pos,uint8_t width, uint8_t height);
	void (*fillCircle)(uint8_t Xpos, uint8_t Ypos, uint8_t Radius);
	void (*fillEllipse)(uint8_t Xpos, uint8_t Ypos, uint8_t XRadius, uint8_t YRadius);
	void (*drawBatteryIndicator)(uint8_t xn, uint8_t yn, uint8_t percentage);
	void (*drawFanIndicator)(uint8_t xn, uint8_t yn);
	void (*drawOnOffButton)(uint8_t xn, uint8_t yn, uint8_t state);
	void (*drawMessageBoxTemplate)(void);
}ST7565_Drv;

extern ST7565_Drv* st7565_drv;

#endif /* INC_ST7565_H_ */

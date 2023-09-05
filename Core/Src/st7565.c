/*
 * st7565.c
 *
 *  Created on: 5 сент. 2023 г.
 *      Author: Kuprin_IV
 */
#include "st7565.h"
#include <string.h>

#define ABS(x) (x) >= 0 ? (x):(-x)

// inner functions
static void ST7565_writeCommand(uint8_t cmd);
static void ST7565_writeData(uint8_t* data, uint16_t length);
static void ST7565_drawBitmap(uint8_t* bmp, uint8_t x, uint8_t y, uint8_t width, uint8_t height);

// driver functions
static void ST7565_DisplayInit(void);
static void ST7565_ClearBuffer(void);
static void ST7565_UpdateBuffer(void);
static void ST7565_SetBrightness(uint8_t brightness);
static void ST7565_SetStringInBuffer(String* string);
static void ST7565_SetWindow(pWindow wnd);
static void ST7565_InvertRegion(uint8_t xn, uint8_t yn, uint8_t xk, uint8_t yk);
static void ST7565_DrawPixel(uint8_t x, uint8_t y);
static void ST7565_DrawLine(int8_t xn, int8_t yn, int8_t xk, int8_t yk);
static void ST7565_DrawCircle(uint8_t x, uint8_t y, uint8_t R);
static void ST7565_DrawEllipse(uint8_t x_pos, uint8_t y_pos, uint8_t rad_x, uint8_t rad_y);
static void ST7565_DrawRect(uint8_t x_pos, uint8_t y_pos,uint8_t width, uint8_t height);
static void ST7565_FillRect(uint8_t x_pos, uint8_t y_pos,uint8_t width, uint8_t height);
static void ST7565_FillCircle(uint8_t Xpos, uint8_t Ypos, uint8_t Radius);
static void ST7565_FillEllipse(uint8_t Xpos, uint8_t Ypos, uint8_t XRadius, uint8_t YRadius);
static void ST7565_DrawBatteryIndicator(uint8_t xn, uint8_t yn, uint8_t percentage);
static void ST7565_DrawFanIndicator(uint8_t xn, uint8_t yn);

// variables
extern SPI_HandleTypeDef hspi1;
ST7565_Drv st7565_driver = {
		ST7565_DisplayInit,
		ST7565_ClearBuffer,
		ST7565_UpdateBuffer,
		ST7565_SetBrightness,
		ST7565_SetStringInBuffer,
		ST7565_SetWindow,
		ST7565_InvertRegion,
		ST7565_DrawPixel,
		ST7565_DrawLine,
		ST7565_DrawCircle,
		ST7565_DrawEllipse,
		ST7565_DrawRect,
		ST7565_FillRect,
		ST7565_FillCircle,
		ST7565_FillEllipse,
		ST7565_DrawBatteryIndicator,
		ST7565_DrawFanIndicator
};
ST7565_Drv* st7565_drv = &st7565_driver;

uint8_t lcd_framebuffer[LCDWIDTH*LCDHEIGHT/8] = {0};

static void ST7565_writeCommand(uint8_t cmd)
{
	CD_GPIO_Port->BRR = CD_Pin; // command mode
	HAL_SPI_Transmit(&hspi1, &cmd, 1, 1000);
}

static void ST7565_writeData(uint8_t* data, uint16_t length)
{
	CD_GPIO_Port->BSRR = CD_Pin; // data mode
	HAL_SPI_Transmit(&hspi1, data, length, 1000);
}

static void ST7565_drawBitmap(uint8_t* bmp, uint8_t x, uint8_t y, uint8_t width, uint8_t height)
{
    uint8_t bytes_in_col = ((height%8) == 0)?(height>>3):((height>>3) + 1);

    if(x + width > LCDWIDTH - 1) x = LCDWIDTH - 1 - width;
    if(y + height > LCDHEIGHT - 1) y = LCDHEIGHT - 1 - height;

    for(uint8_t i = 0; i < width; i++)
    {
    	for(uint8_t j = 0; j < bytes_in_col; j++)
    	{
    		for(uint8_t k = 0; k < 8; k++)
    		{
				if((bmp[j*width+i]>>k) & 0x01)
					ST7565_DrawPixel(x+i, y+j*8+k);
    		}
    	}
    }
}

static void ST7565_DisplayInit(void)
{
	// make reset
	RST_GPIO_Port->BRR = RST_Pin;
	HAL_Delay(50);
	RST_GPIO_Port->BSRR = RST_Pin;

	// LCD bias select
	ST7565_writeCommand(CMD_SET_BIAS_7);
	// ADC select
	ST7565_writeCommand(CMD_SET_ADC_NORMAL);
	// SHL select
	ST7565_writeCommand(CMD_SET_COM_NORMAL);
	// Initial display line
	ST7565_writeCommand(CMD_SET_DISP_START_LINE);

	// turn on voltage converter (VC=1, VR=0, VF=0)
	ST7565_writeCommand(CMD_SET_POWER_CONTROL | 0x4);
	// wait for 50% rising
	HAL_Delay(50);

	// turn on voltage regulator (VC=1, VR=1, VF=0)
	ST7565_writeCommand(CMD_SET_POWER_CONTROL | 0x6);
	// wait >=50ms
	HAL_Delay(50);

	// turn on voltage follower (VC=1, VR=1, VF=1)
	ST7565_writeCommand(CMD_SET_POWER_CONTROL | 0x7);
	// wait
	HAL_Delay(10);

	// set lcd operating voltage (regulator resistor, ref voltage resistor)
	ST7565_writeCommand(CMD_SET_RESISTOR_RATIO | 0x6);

	// enable LCD
	ST7565_writeCommand(CMD_DISPLAY_ON);
	ST7565_writeCommand(CMD_SET_ALLPTS_NORMAL);
	ST7565_SetBrightness(10);
}

static void ST7565_ClearBuffer(void)
{
	memset(lcd_framebuffer, 0, sizeof(lcd_framebuffer));
}

static void ST7565_UpdateBuffer(void)
{
	for(uint8_t page = 0; page < 8; page++)
	{
		// set page
		ST7565_writeCommand(CMD_SET_PAGE | page);
		// set column
		ST7565_writeCommand(CMD_SET_COLUMN_LOWER | ((ST7565_STARTBYTES) & 0x0F));
		ST7565_writeCommand(CMD_SET_COLUMN_UPPER | (((ST7565_STARTBYTES) >> 4) & 0x0F));
		ST7565_writeCommand(CMD_RMW);
		// write data
		ST7565_writeData(lcd_framebuffer+LCDWIDTH*page, LCDWIDTH);
	}
}

static void ST7565_SetBrightness(uint8_t brightness)
{
	if(brightness > 63) brightness = 63;
	ST7565_writeCommand(CMD_SET_VOLUME_FIRST);
	ST7565_writeCommand(CMD_SET_VOLUME_SECOND | (brightness & 0x3F));
}

static void ST7565_SetStringInBuffer(String* string)
{
	uint8_t x = string->x_pos, y = string->y_pos, x_inv = 0;
	const unsigned char* pStr = (const unsigned char*)string->Text;
	uint8_t currentCharWidth = 0;
	int Length = strlen((char*)string->Text),LengthInv = Length, widthInPixels = 0;
	//calculate width of text in pixels
	const unsigned char* lStr = (const unsigned char*)string->Text;
	while(LengthInv-- > 0)
	{
	   widthInPixels += string->font.descriptor[*(lStr)-' '].width + 1;
	   lStr++;
	}

	if(string->align == AlignCenter) x = (LCDWIDTH - widthInPixels + x)/2;
	if(string->align == AlignRight) x = LCDWIDTH - 1 - widthInPixels;
	x_inv = x;

	while(Length-- > 0)
	{
		currentCharWidth = string->font.descriptor[*(pStr)-' '].width;
		ST7565_drawBitmap((uint8_t*)(string->font.pFont+string->font.descriptor[*(pStr)-' '].offset), x, y, currentCharWidth, string->font.Height);
		x += currentCharWidth+1;
		pStr++;
	}

	if(string->inverted == Inverted)
	{
		ST7565_InvertRegion(x_inv-1, y-1, x_inv+widthInPixels + 1, y+string->font.Height);
	}
}

static void ST7565_SetWindow(pWindow wnd)
{
	int i;

    for(i = 0; i < wnd->StringsQuantity; i++)
    {
    	ST7565_SetStringInBuffer(&wnd->strings[i]);
    }
}

static void ST7565_InvertRegion(uint8_t xn, uint8_t yn, uint8_t xk, uint8_t yk)
{
	uint8_t mask = 0;
	uint8_t i, j;

	// transform y-coordinates
	yn = LCDHEIGHT - 1 - yn;
	yk = LCDHEIGHT - 1 - yk;

	for(j = yk/8; j <= yn/8; j++)
	{
		for(i = xn; i < xk;i++)
		{
			if(j == yn/8)
			{
			  mask = 0xFF>>(7-(yn%8));
			}
			else
			{
			  if(j == yk/8)
			  {
				  mask = 0xFF<<(yk%8);
			  }
			  else
			  {
			  	mask = 0xFF;
			  }
			}
			lcd_framebuffer[LCDWIDTH*j + i] ^= mask;
		}
	}
}

static void ST7565_DrawPixel(uint8_t x, uint8_t y)
{
	// transform y-coordinate
	y = LCDHEIGHT - 1 - y;

	lcd_framebuffer[LCDWIDTH*(y>>3)+x] |= 1<<(y%8);
}

static void ST7565_DrawLine(int8_t xn, int8_t yn, int8_t xk, int8_t yk)
{
	int8_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
	yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
	curpixel = 0;

	deltax = ABS(yk-yn);        /* The difference between the x's */
	deltay = ABS(xk-xn);        /* The difference between the y's */
	x = xn;                       /* Start x off at the first pixel */
	y = yn;                       /* Start y off at the first pixel */

	if (xk >= xn)                 /* The x-values are increasing */
	{
		xinc1 = 1;
		xinc2 = 1;
	}
	else                          /* The x-values are decreasing */
	{
		xinc1 = -1;
		xinc2 = -1;
	}

	if (yk >= yn)                 /* The y-values are increasing */
	{
		yinc1 = 1;
		yinc2 = 1;
	}
	else                          /* The y-values are decreasing */
	{
		yinc1 = -1;
		yinc2 = -1;
	}

	if (deltax >= deltay)         /* There is at least one x-value for every y-value */
	{
		xinc2 = 0;                  /* Don't change the x when numerator >= denominator */
		yinc1 = 0;                  /* Don't change the y for every iteration */
		den = deltax;
		num = deltax / 2;
		numadd = deltay;
		numpixels = deltax;         /* There are more x-values than y-values */
	}
	else                          /* There is at least one y-value for every x-value */
	{
		xinc1 = 0;                  /* Don't change the x for every iteration */
		yinc2 = 0;                  /* Don't change the y when numerator >= denominator */
		den = deltay;
		num = deltay / 2;
		numadd = deltax;
		numpixels = deltay;         /* There are more y-values than x-values */
	}

	for (curpixel = 0; curpixel <= numpixels; curpixel++)
	{
		ST7565_DrawPixel(x,y); /* Draw the current pixel */
		num += numadd;                            /* Increase the numerator by the top of the fraction */
		if (num >= den)                           /* Check if numerator >= denominator */
		{
			num -= den;                             /* Calculate the new numerator value */
			x += xinc1;                             /* Change the x as appropriate */
			y += yinc1;                             /* Change the y as appropriate */
		}
		x += xinc2;                               /* Change the x as appropriate */
		y += yinc2;                               /* Change the y as appropriate */
	}
}

static void ST7565_DrawCircle(uint8_t x, uint8_t y, uint8_t R)
{
	int8_t  decision;       /* Decision Variable */
	uint8_t  curx;   /* Current X Value */
	uint8_t  cury;   /* Current Y Value */

	decision = 3 - (R << 1);
	curx = 0;
	cury = R;

	while (curx <= cury)
	{
		ST7565_DrawPixel((x - cury),(y + curx));
		ST7565_DrawPixel((x - cury),(y - curx));
		ST7565_DrawPixel((x - curx),(y + cury));
		ST7565_DrawPixel((x - curx),(y - cury));
		ST7565_DrawPixel((x + cury),(y + curx));
		ST7565_DrawPixel((x + cury),(y - curx));
		ST7565_DrawPixel((x + curx),(y + cury));
		ST7565_DrawPixel((x + curx),(y - cury));

		if (decision < 0)
		{
			decision += (curx << 2) + 6;
		}
		else
		{
			decision += ((curx - cury) << 2) + 10;
			cury--;
		}
		curx++;
	}
}

static void ST7565_DrawEllipse(uint8_t x_pos, uint8_t y_pos, uint8_t rad_x, uint8_t rad_y)
{
    char x = 0, y = -rad_x, err = 2-2*rad_y, e2;
    float k = 0, rad1 = 0, rad2 = 0;

    rad1 = rad_y;
    rad2 = rad_x;

    k = (float)(rad2/rad1);

    do
    {
    	ST7565_DrawPixel((x_pos+y),(y_pos -(uint8_t)(x/k)));
    	ST7565_DrawPixel((x_pos+y),(y_pos +(uint8_t)(x/k)));
    	ST7565_DrawPixel((x_pos-y), (y_pos +(uint8_t)(x/k)));
    	ST7565_DrawPixel((x_pos-y), (y_pos -(uint8_t)(x/k)));

        e2 = err;
        if (e2 <= x) {
            err += ++x*2+1;
            if (-y == x && e2 <= y) e2 = 0;
        }
        if (e2 > y) err += ++y*2+1;
    }
    while (y <= 0);
}

static void ST7565_DrawRect(uint8_t x_pos, uint8_t y_pos,uint8_t width, uint8_t height)
{
	ST7565_DrawLine(x_pos,y_pos,x_pos+width,y_pos);
	ST7565_DrawLine(x_pos+width,y_pos,x_pos+width,y_pos+height);
	ST7565_DrawLine(x_pos,y_pos+height,x_pos+width,y_pos+height);
	ST7565_DrawLine(x_pos,y_pos,x_pos,y_pos+height);
}

static void ST7565_FillRect(uint8_t x_pos, uint8_t y_pos,uint8_t width, uint8_t height)
{
    for(;height>0;height--)
    {
    	ST7565_DrawLine(x_pos,y_pos,x_pos+width-1,y_pos);
    	y_pos++;
    }
}

static void ST7565_FillCircle(uint8_t Xpos, uint8_t Ypos, uint8_t Radius)
{
	int8_t  decision;        /* Decision Variable */
	uint8_t  curx;    /* Current X Value */
	uint8_t  cury;    /* Current Y Value */

	decision = 3 - (Radius << 1);

	curx = 0;
	cury = Radius;

	while (curx <= cury)
	{
		if(cury > 0)
		{
			ST7565_DrawLine(Xpos + curx, Ypos - cury,Xpos + curx, Ypos + cury);
			ST7565_DrawLine(Xpos - curx, Ypos - cury,Xpos - curx, Ypos + cury);
		}

		if(curx > 0)
		{
			ST7565_DrawLine(Xpos - cury, Ypos - curx,Xpos - cury, Ypos + curx);
			ST7565_DrawLine(Xpos + cury, Ypos - curx,Xpos + cury, Ypos + curx);
		}
		if (decision < 0)
		{
			decision += (curx << 2) + 6;
		}
		else
		{
			decision += ((curx - cury) << 2) + 10;
			cury--;
		}
		curx++;
	}
	ST7565_DrawCircle(Xpos, Ypos, Radius);
}

static void ST7565_FillEllipse(uint8_t Xpos, uint8_t Ypos, uint8_t XRadius, uint8_t YRadius)
{
	char x = 0, y = -XRadius, err = 2-2*YRadius, e2;
	float k = 0, rad1 = 0, rad2 = 0;

	rad1 = YRadius;
	rad2 = XRadius;

	k = (float)(rad2/rad1);

	do
	{
		ST7565_DrawLine((Xpos+y), (Ypos-(uint8_t)(x/k)),(Xpos+y), (Ypos + (uint8_t)(x/k) + 1));
		ST7565_DrawLine((Xpos-y), (Ypos-(uint8_t)(x/k)),(Xpos-y), (Ypos + (uint8_t)(x/k) + 1));

		e2 = err;
		if (e2 <= x)
		{
			err += ++x*2+1;
			if (-y == x && e2 <= y) e2 = 0;
		}
		if (e2 > y) err += ++y*2+1;
	}
	while (y <= 0);
}

static void ST7565_DrawBatteryIndicator(uint8_t xn, uint8_t yn, uint8_t percentage)
{
    uint8_t BatteryBorder[12] = {0x7E,0xC3,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0xFF};
    // fill indicator by percentage
    for(uint8_t i = 0; i < percentage; i++)
    {
    	if(i < 9)
    	{
    		BatteryBorder[10-i] |= 0x7E;
    	}
    	else
    	{
    		BatteryBorder[10-i] |= 0x3C;
    	}
    }
    ST7565_drawBitmap(BatteryBorder, xn, yn, 12, 8);
}

static void ST7565_DrawFanIndicator(uint8_t xn, uint8_t yn)
{
    uint8_t fan_bitmap[24] = {0x1E,0x91,0xA1,0xA1,0xFE,0x90,0x9C,0xF3,0x51,0x51,0x91,0x8E,
    							0x07,0x08,0x08,0x08,0x0C,0x03,0x00,0x07,0x08,0x08,0x08,0x07};
	ST7565_drawBitmap(fan_bitmap, xn, yn, 12, 12);
}


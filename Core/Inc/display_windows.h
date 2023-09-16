/*
 * display_windows.h
 *
 *  Created on: 5 сент. 2023 г.
 *      Author: Kuprin_IV
 */

#ifndef INC_DISPLAY_WINDOWS_H_
#define INC_DISPLAY_WINDOWS_H_

#include "main.h"
#include "fonts.h"
#include "st7565.h"
#include "print_to_string.h"

#define SETTINGS_ITEMS_NUM          6

typedef struct
{
	void (*windowsInit)(void);
	void (*goToNextWindowOrItem)(void);
	void (*updateWindowParameters)(Action encoder_offset_action);
	void (*refreshWindow)(void);
}DisplayWndCtrl;

extern DisplayWndCtrl* display_wnd_ctrl;

#endif /* INC_DISPLAY_WINDOWS_H_ */

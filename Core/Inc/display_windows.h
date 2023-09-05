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
#include <string.h>

#define SETTINGS_ITEMS_NUM          5

int DisplayMainWindow(pWindow wnd, pData data, Action item_action, Action value_action);
int SetMenuWindow(pWindow wnd,pData data, Action item_action, Action value_action);
int SetupModeWindow(pWindow wnd, pData data, Action item_action, Action value_action);
int SetupMaxPowerWindow(pWindow wnd, pData data, Action item_action, Action value_action);
int SetupCalibrationWindow(pWindow wnd, pData data, Action item_action, Action value_action);
int SetupBatteryWindow(pWindow wnd, pData data, Action item_action, Action value_action);

#endif /* INC_DISPLAY_WINDOWS_H_ */

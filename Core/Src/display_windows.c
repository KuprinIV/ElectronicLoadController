/*
 * display_windows.c
 *
 *      Author: Kuprin_IV
 */
#include "display_windows.h"
#include "load_control.h"
#include <stdio.h>

// driver functions
static void windowsInit(void);
static void goToNextWindowOrItem(void);
static void updateWindowParameters(Action encoder_offset_action);
static void refreshWindow(void);

// callbacks
static int DisplayMainWindow(pWindow wnd, pData data, Action item_action, Action value_action);
static int SetMenuWindow(pWindow wnd,pData data, Action item_action, Action value_action);
static int SetupModeWindow(pWindow wnd, pData data, Action item_action, Action value_action);
static int SetupMaxPowerWindow(pWindow wnd, pData data, Action item_action, Action value_action);
static int SetupCalibrationWindow(pWindow wnd, pData data, Action item_action, Action value_action);
static int SetupBatteryWindow(pWindow wnd, pData data, Action item_action, Action value_action);
static int SetupDisplayWindow(pWindow wnd, pData data, Action item_action, Action value_action);

// inner functions
static float GetCurrentChangeStep(float current);

DisplayWndCtrl drv = {
		windowsInit,
		goToNextWindowOrItem,
		updateWindowParameters,
		refreshWindow,
};
DisplayWndCtrl* display_wnd_ctrl = &drv;

// private variables
static Window MenuWnd;
static Window DispMainWnd;
static Window SetupWnds[5];
static pWindow CurrentWnd = &DispMainWnd;
static pWindow temp_wnd;
static uint8_t level = 0;

extern Data loadData;

static uint32_t mode_current_item = 1;
static uint32_t calibration_current_item = 1;
static uint32_t menu_current_item = 1;

/**
  * @brief  Initialize interface windows
  * @param  none
  * @retval none
  */
static void windowsInit(void)
{
	// init display
	st7565_drv->displayInit();
	st7565_drv->setBrightness(loadData.load_settings.display_contrast);

	// init windows
	DispMainWnd.callback = &DisplayMainWindow;
	MenuWnd.callback = &SetMenuWindow;

	SetupWnds[0].callback = &SetupModeWindow;
	SetupWnds[1].callback = &SetupMaxPowerWindow;
	SetupWnds[2].callback = &SetupCalibrationWindow;
	SetupWnds[3].callback = &SetupBatteryWindow;
	SetupWnds[4].callback = &SetupDisplayWindow;

	CurrentWnd = &DispMainWnd;

	// draw main window
	refreshWindow();
}

/**
  * @brief  Change window or go to the next window item
  * @param  none
  * @retval none
  */
static void goToNextWindowOrItem(void)
{
	// clear display buffer data
	st7565_drv->clearBuffer();

	switch(level)
	{
		case 0:
			temp_wnd = CurrentWnd;
			CurrentWnd = &MenuWnd;
			CurrentWnd->prev = temp_wnd;
			CurrentWnd->callback(CurrentWnd,&loadData,NoAction,NoAction);
			level++;
			break;

		case 1:
			if(menu_current_item == SETTINGS_ITEMS_NUM)
			{
				// save load setting in EEPROM
				load_control_drv->saveLoadSettings(&loadData.load_settings);
				// show main window
				CurrentWnd = CurrentWnd->prev;
				CurrentWnd->callback(CurrentWnd,&loadData,NoAction,NoAction);
				menu_current_item = 1;
				level--;
			}
			else
			{
				temp_wnd = CurrentWnd;
				CurrentWnd = &(SetupWnds[menu_current_item-1]);
				CurrentWnd->prev = temp_wnd;
				CurrentWnd->callback(CurrentWnd,&loadData,NoAction,NoAction);
				level++;
			}
			break;

		case 2:
			if(CurrentWnd->callback(CurrentWnd,&loadData,Next,NoAction) == 0)
			{
				level--;
				CurrentWnd = CurrentWnd->prev;
				CurrentWnd->callback(CurrentWnd,&loadData,NoAction,NoAction);
			}
			break;
	}

	// update display
	st7565_drv->updateBuffer();
}

/**
  * @brief  Update window item parameter
  * @param  encoder_offset_action: NoAction - do nothing, Next - increase parameter value or go to the next item, Prev -
  * decrease parameter value or go to the previous item
  * @retval none
  */
static void updateWindowParameters(Action encoder_offset_action)
{
	// clear display buffer data
	st7565_drv->clearBuffer();

	switch(level)
	{
		case 0:
			CurrentWnd->callback(CurrentWnd,&loadData,NoAction,encoder_offset_action);
			break;

		case 1:
			CurrentWnd->callback(CurrentWnd,&loadData,encoder_offset_action,NoAction);
			break;

		case 2:
			CurrentWnd->callback(CurrentWnd,&loadData,NoAction,encoder_offset_action);
			break;
	}

	// update display
	st7565_drv->updateBuffer();
}

/**
  * @brief  Refresh window
  * @param  none
  * @retval none
  */
static void refreshWindow(void)
{
	st7565_drv->clearBuffer();
	CurrentWnd->callback(CurrentWnd, &loadData, NoAction, NoAction);
	st7565_drv->updateBuffer();
}

/**
  * @brief  Callback function for drawing main window
  * @param  wnd - data structure with window parameters
  * @param  data - data structure with electronic load parameters
  * @param  item_action: NoAction - do nothing, Next - go to the next item, Prev - go to previous item
  * @retval 0 - after window drawing go to previous window, 1 - after window drawing stay in it
  */
static int DisplayMainWindow(pWindow wnd, pData data, Action item_action, Action value_action)
{
    float max_current;
    float step = 0.01f;
    uint8_t charge = 0;

    // calculate battery charge percentage
    if(data->vbat > VBAT_LOW)
    {
    	charge = 10*(data->vbat - VBAT_LOW)/(4.2f-VBAT_LOW);
    }
    else
    {
    	charge = 0;
    }


    max_current = (float)data->load_settings.max_power/data->voltage;
    if(max_current > 20.0f) max_current = 20.0f;

    // round max current value to nearest step value
    step = GetCurrentChangeStep(max_current);
    int steps_num = (int)(max_current/step);
    max_current = steps_num*step;

    step = GetCurrentChangeStep(data->set_current);

    switch(value_action)
    {
        case NoAction:
        default:
            break;

        case Prev:
            if(data->set_current > 0.01f) data->set_current -= step;
            else data->set_current = 0;
            break;

        case Next:
            if(data->set_current < max_current) data->set_current += step;
            else data->set_current = max_current;
            break;
    }

    // set current value
    load_control_drv->setCurrentInAmperes(data->set_current);

    wnd->strings[0].x_pos = 3;
    wnd->strings[0].y_pos = 2;
    wnd->strings[0].align = AlignLeft;
    wnd->strings[0].font = font6x8;
    wnd->strings[0].inverted = data->is_ovt == 1 ? Inverted : NotInverted;
    sprintf(wnd->strings[0].Text, "%0.1f°C", data->temperature);

    wnd->strings[1].x_pos = 70;
    wnd->strings[1].y_pos = 2;
    wnd->strings[1].align = AlignLeft;
    wnd->strings[1].font = font6x8;
    wnd->strings[1].inverted = NotInverted;
    sprintf(wnd->strings[1].Text, "%d", data->rpm);

    wnd->strings[2].x_pos = 0;
    wnd->strings[2].y_pos = 13;
    wnd->strings[2].align = AlignCenter;
    wnd->strings[2].font = MSSanSerif_14;
    wnd->strings[2].inverted = NotInverted;
    if(data->measured_current >= 5.0f)
    {
        sprintf(wnd->strings[2].Text, "%0.1f B  %0.1f A", data->voltage, data->measured_current);
    }
    else
    {
        sprintf(wnd->strings[2].Text, "%0.1f B  %0.2f A", data->voltage, data->measured_current);
    }

    wnd->strings[3].x_pos = 0;
    wnd->strings[3].y_pos = 33;
    wnd->strings[3].align = AlignCenter;
    wnd->strings[3].font = font6x8;
    wnd->strings[3].inverted = NotInverted;
    sprintf(wnd->strings[3].Text, "%0.0f mAh  %0.1f Wh", data->mAh, data->Wh);

    wnd->strings[4].x_pos = 0;
    wnd->strings[4].y_pos = 42;
    wnd->strings[4].align = AlignCenter;
    wnd->strings[4].font = font6x8;
    wnd->strings[4].inverted = NotInverted;
    if(data->load_settings.load_work_mode == BatteryDischarge)
    {
        if(data->set_current >= 5.0f)
        {
            sprintf(wnd->strings[4].Text, "Vdis:%0.2fV Iset:%0.1fA", data->load_settings.discharge_voltage, data->set_current);
        }
        else
        {
            sprintf(wnd->strings[4].Text, "Vdis:%0.2fV Iset:%0.2fA", data->load_settings.discharge_voltage, data->set_current);
        }
    }
    else
    {
        if(data->set_current >= 5.0f)
        {
            sprintf(wnd->strings[4].Text, "Iset:%0.1fA", data->set_current);
        }
        else
        {
            sprintf(wnd->strings[4].Text, "Iset:%0.2fA", data->set_current);
        }
    }

    wnd->StringsQuantity = 5;

    st7565_drv->drawBatteryIndicator(105, 1, charge);
    st7565_drv->drawFanIndicator(56, 0);
    st7565_drv->drawOnOffButton(51, 52, data->on_state);

    st7565_drv->setWindow(wnd);
    return 0;
}

/**
  * @brief  Callback function for drawing settings menu window
  * @param  wnd - data structure with window parameters
  * @param  data - data structure with electronic load parameters
  * @param  item_action: NoAction - do nothing, Next - go to the next item, Prev - go to previous item
  * @retval 0 - after window drawing go to previous window, 1 - after window drawing stay in it
  */
static int SetMenuWindow(pWindow wnd,pData data, Action item_action, Action value_action)
{
    const char* items[SETTINGS_ITEMS_NUM]  = {"Mode\0","Max power\0","Calibration\0","Battery\0","Display\0","Quit\0"};
    uint8_t items_num = SETTINGS_ITEMS_NUM;
    int max_item_per_screen = 6;

    wnd->StringsQuantity = (items_num <= max_item_per_screen ? items_num : max_item_per_screen);

    switch(item_action)
    {
        case NoAction:
        default:
            break;

        case Next:
            if(++menu_current_item > items_num)
            {
                menu_current_item = 1;
            }
            break;

        case Prev:
            if(--menu_current_item < 1)
            {
                menu_current_item = items_num;
            }
            break;
    }

    if(menu_current_item <= max_item_per_screen)
    {
        for(uint8_t i = 0; i < (items_num <= max_item_per_screen ? items_num : max_item_per_screen); i++)
        {
            wnd->strings[i].x_pos = 2;
            wnd->strings[i].y_pos = 2+10*i;
            wnd->strings[i].align = AlignLeft;
            wnd->strings[i].font = font6x8;
            wnd->strings[i].inverted = (i == menu_current_item-1)?(Inverted):(NotInverted);
            sprintf(wnd->strings[i].Text, "%s", items[i]);
        }
    }
    else
    {
        for(int i = 0; i < max_item_per_screen; i++)
        {
            wnd->strings[i].x_pos = 2;
            wnd->strings[i].y_pos = 2+10*i;
            wnd->strings[i].align = AlignLeft;
            wnd->strings[i].font = font6x8;
            wnd->strings[i].inverted = (i == max_item_per_screen-1)?(Inverted):(NotInverted);
            sprintf(wnd->strings[i].Text, "%s", items[menu_current_item-max_item_per_screen+i]);
        }
    }
    st7565_drv->setWindow(wnd);
    return 0;
}

/**
  * @brief  Callback function for drawing electronic load working mode window
  * @param  wnd - data structure with window parameters
  * @param  data - data structure with electronic load parameters
  * @param  item_action: NoAction - do nothing, Next - go to the next item, Prev - go to previous item
  * @retval 0 - after window drawing go to previous window, 1 - after window drawing stay in it
  */
static int SetupModeWindow(pWindow wnd, pData data, Action item_action, Action value_action)
{
    const char* modes[3]  = {"Simple\0","Discharge\0","Ramp up\0"};
    uint16_t ramp_lengths[4] = {10, 20, 50, 100};
    int8_t mode_cntr = (int8_t)data->load_settings.load_work_mode;
    int8_t ramps_cntr = 0;
    int8_t modes_num = 3;
    int8_t ramp_lengths_num = sizeof(ramp_lengths)/2;
    int mode_items_num = data->load_settings.load_work_mode == SimpleLoad ? 2 : 3;
    float step = 0.05f;

    // get ramp length counter from saved value
    for(uint8_t i = 0; i < ramp_lengths_num; i++)
    {
    	if(ramp_lengths[i] == data->load_settings.current_ramp_time)
    	{
    		ramps_cntr = i;
    		break;
    	}
    }

    if(data->load_settings.discharge_voltage < 5.0f) step = 0.05f;
    else if(data->load_settings.discharge_voltage < 20.0f) step = 0.1f;
    else step = 0.5f;

    switch(item_action)
    {
        case NoAction:
        case Prev:
        default:
            break;

        case Next:
            if(++mode_current_item > mode_items_num)
            {
                mode_current_item = 1;
                return 0;
            }
            break;
    }

    switch(value_action)
    {
        case NoAction:
        default:
            break;

        case Next:
            if(mode_current_item == 1)
            {
                if(++mode_cntr > modes_num-1)
                {
                	mode_cntr = 0;
                }
                data->load_settings.load_work_mode = (LoadMode)mode_cntr;
            }
            else if(mode_current_item == 2)
            {
            	if(data->load_settings.load_work_mode == BatteryDischarge)
            	{
					if(data->load_settings.discharge_voltage < 50.0f) data->load_settings.discharge_voltage += step;
					else data->load_settings.discharge_voltage = 50.0f;
            	}
            	else if(data->load_settings.load_work_mode == Ramp)
            	{
                    if(++ramps_cntr > ramp_lengths_num-1)
                    {
                    	ramps_cntr = 0;
                    }
                    data->load_settings.current_ramp_time = ramp_lengths[ramps_cntr];
            	}
            }
            break;

        case Prev:
            if(mode_current_item == 1)
            {
                if(--mode_cntr < 0)
                {
                	mode_cntr = modes_num-1;
                }
                data->load_settings.load_work_mode = (LoadMode)mode_cntr;
            }
            else if(mode_current_item == 2)
            {
            	if(data->load_settings.load_work_mode == BatteryDischarge)
            	{
					if(data->load_settings.discharge_voltage > 0.05f) data->load_settings.discharge_voltage -= step;
					else data->load_settings.discharge_voltage = 0.0f;
            	}
            	else if(data->load_settings.load_work_mode == Ramp)
            	{
                    if(--ramps_cntr < 0)
                    {
                    	ramps_cntr = ramp_lengths_num-1;
                    }
                    data->load_settings.current_ramp_time = ramp_lengths[ramps_cntr];
            	}
            }
            break;
    }

    wnd->strings[0].x_pos = 10;
    wnd->strings[0].y_pos = 5;
    wnd->strings[0].align = AlignLeft;
    wnd->strings[0].font = font6x8;
    wnd->strings[0].inverted = NotInverted;
    sprintf(wnd->strings[0].Text, "Mode:");

    wnd->strings[1].x_pos = 50;
    wnd->strings[1].y_pos = 5;
    wnd->strings[1].align = AlignCenter;
    wnd->strings[1].font = font6x8;
    wnd->strings[1].inverted = mode_current_item == 1 ? Inverted : NotInverted;
    sprintf(wnd->strings[1].Text, "%s", modes[data->load_settings.load_work_mode]);

    if(data->load_settings.load_work_mode == BatteryDischarge)
    {
        wnd->strings[2].x_pos = 10;
        wnd->strings[2].y_pos = 15;
        wnd->strings[2].align = AlignLeft;
        wnd->strings[2].font = font6x8;
        wnd->strings[2].inverted = NotInverted;
        sprintf(wnd->strings[2].Text, "Vdisch:");

        wnd->strings[3].x_pos = 50;
        wnd->strings[3].y_pos = 15;
        wnd->strings[3].align = AlignCenter;
        wnd->strings[3].font = font6x8;
        wnd->strings[3].inverted = mode_current_item == 2 ? Inverted : NotInverted;
        sprintf(wnd->strings[3].Text, "%0.2f V", data->load_settings.discharge_voltage);


        wnd->strings[4].x_pos = 0;
        wnd->strings[4].y_pos = 55;
        wnd->strings[4].align = AlignCenter;
        wnd->strings[4].font = font6x8;
        wnd->strings[4].inverted = mode_current_item == mode_items_num ? Inverted : NotInverted;
        sprintf(wnd->strings[4].Text, "  < Back    ");

        wnd->StringsQuantity = 5;
    }
    else if(data->load_settings.load_work_mode == Ramp)
    {
        wnd->strings[2].x_pos = 10;
        wnd->strings[2].y_pos = 15;
        wnd->strings[2].align = AlignLeft;
        wnd->strings[2].font = font6x8;
        wnd->strings[2].inverted = NotInverted;
        sprintf(wnd->strings[2].Text, "Ramp:");

        wnd->strings[3].x_pos = 50;
        wnd->strings[3].y_pos = 15;
        wnd->strings[3].align = AlignCenter;
        wnd->strings[3].font = font6x8;
        wnd->strings[3].inverted = mode_current_item == 2 ? Inverted : NotInverted;
        sprintf(wnd->strings[3].Text, "%d ms", data->load_settings.current_ramp_time);

        wnd->strings[4].x_pos = 0;
        wnd->strings[4].y_pos = 55;
        wnd->strings[4].align = AlignCenter;
        wnd->strings[4].font = font6x8;
        wnd->strings[4].inverted = mode_current_item == mode_items_num ? Inverted : NotInverted;
        sprintf(wnd->strings[4].Text, "  < Back    ");

        wnd->StringsQuantity = 5;
    }
    else
    {
        wnd->strings[2].x_pos = 0;
        wnd->strings[2].y_pos = 55;
        wnd->strings[2].align = AlignCenter;
        wnd->strings[2].font = font6x8;
        wnd->strings[2].inverted = mode_current_item == mode_items_num ? Inverted : NotInverted;
        sprintf(wnd->strings[2].Text, "  < Back    ");

        wnd->StringsQuantity = 3;
    }

    st7565_drv->setWindow(wnd);
    return 1;
}

/**
  * @brief  Callback function for drawing electronic load max power settings window
  * @param  wnd - data structure with window parameters
  * @param  data - data structure with electronic load parameters
  * @param  item_action: NoAction - do nothing, Next - go to the next item, Prev - go to previous item
  * @retval 0 - after window drawing go to previous window, 1 - after window drawing stay in it
  */
static int SetupMaxPowerWindow(pWindow wnd, pData data, Action item_action, Action value_action)
{
    const char* max_power_space[3] = {" \0", "  \0", "   \0"};
    int index = 2;
    int steps_num = 0;
    float step = 0.01f;

    switch(item_action)
    {
        case NoAction:
        case Prev:
        default:
            break;

        case Next:
            return 0;
            break;
    }

    switch(value_action)
    {
        case NoAction:
        default:
            break;

        case Prev:
            if(data->load_settings.max_power > 10) data->load_settings.max_power--;
            else data->load_settings.max_power = 10;
            break;

        case Next:
            if(data->load_settings.max_power < 250) data->load_settings.max_power++;
            else data->load_settings.max_power = 250;
            break;
    }

    if(data->load_settings.max_power >= 100)
    {
        index = 2;
    }
    else if(data->load_settings.max_power >= 10)
    {
        index = 1;
    }
    else
    {
        index = 0;
    }

    if(data->set_current > (float)data->load_settings.max_power/data->voltage+0.01f)
    {
        data->set_current = (float)data->load_settings.max_power/data->voltage;
        // round current value to nearest step value
        step = GetCurrentChangeStep(data->set_current);
        steps_num = (int)(data->set_current/step);
        data->set_current = steps_num*step;
        // limit current value
        load_control_drv->setCurrentInAmperes(data->set_current);
    }

    wnd->strings[0].x_pos = 15;
    wnd->strings[0].y_pos = 25;
    wnd->strings[0].align = AlignLeft;
    wnd->strings[0].font = font6x8;
    wnd->strings[0].inverted = NotInverted;
    sprintf(wnd->strings[0].Text, "Max power = %sW", max_power_space[index]);

    wnd->strings[1].x_pos = 85;
    wnd->strings[1].y_pos = 25;
    wnd->strings[1].align = AlignLeft;
    wnd->strings[1].font = font6x8;
    wnd->strings[1].inverted = Inverted;
    sprintf(wnd->strings[1].Text, "%d", data->load_settings.max_power);

    wnd->strings[2].x_pos = 0;
    wnd->strings[2].y_pos = 55;
    wnd->strings[2].align = AlignCenter;
    wnd->strings[2].font = font6x8;
    wnd->strings[2].inverted = Inverted;
    sprintf(wnd->strings[2].Text, "   Ok   ");

    wnd->StringsQuantity = 3;

    st7565_drv->setWindow(wnd);
    return 1;
}

/**
  * @brief  Callback function for drawing calibration mode window
  * @param  wnd - data structure with window parameters
  * @param  data - data structure with electronic load parameters
  * @param  item_action: NoAction - do nothing, Next - go to the next item, Prev - go to previous item
  * @retval 0 - after window drawing go to previous window, 1 - after window drawing stay in it
  */
static int SetupCalibrationWindow(pWindow wnd, pData data, Action item_action, Action value_action)
{
    const char* modes[2]  = {"Current\0","Voltage\0"};
    const char* refs[6] = {"0,1A", "1A", " 5A", "1V", "10V", "25V"};
    int calibration_items_num = 7;
    uint16_t* calibration_data_ptr = (uint16_t*)&data->calibration_data;

    // enable load
    load_control_drv->setEnabled(1);

    switch(item_action)
    {
        case NoAction:
            if(calibration_current_item < 4)
            {
                load_control_drv->setCurrentInDiscreets(calibration_data_ptr[calibration_current_item - 1]);
            }
        	break;

        case Prev:
        default:
            break;

        case Next:
            if(++calibration_current_item > calibration_items_num)
            {
                calibration_current_item = 1;
                // disable load
                load_control_drv->setEnabled(0);
                // save calibration data
                load_control_drv->saveCalibrationData(&data->calibration_data);
                return 0;
            }
            else if(calibration_current_item < 4)
            {
                load_control_drv->setCurrentInDiscreets(calibration_data_ptr[calibration_current_item - 1]);
            }
            break;
    }

    switch(value_action)
    {
        case NoAction:
        default:
            break;

        case Next:
            if(calibration_current_item < 4)
            {
                calibration_data_ptr[calibration_current_item - 1]++;
                load_control_drv->setCurrentInDiscreets(calibration_data_ptr[calibration_current_item - 1]);
            }
            break;

        case Prev:
            if(calibration_current_item < 4)
            {
                calibration_data_ptr[calibration_current_item - 1]--;
                load_control_drv->setCurrentInDiscreets(calibration_data_ptr[calibration_current_item - 1]);
            }
            break;
    }

    wnd->strings[0].x_pos = 0;
    wnd->strings[0].y_pos = 5;
    wnd->strings[0].align = AlignCenter;
    wnd->strings[0].font = font6x8;
    wnd->strings[0].inverted = NotInverted;

    if(calibration_current_item < 4)
    {
        sprintf(wnd->strings[0].Text, "%s calibration", modes[(calibration_current_item-1)>>2]);

        wnd->strings[1].x_pos = 0;
        wnd->strings[1].y_pos = 18;
        wnd->strings[1].align = AlignCenter;
        wnd->strings[1].font = font6x8;
        wnd->strings[1].inverted = NotInverted;
        sprintf(wnd->strings[1].Text, "Set current %s", refs[calibration_current_item-1]);

        wnd->strings[2].x_pos = 40;
		wnd->strings[2].y_pos = 28;
		wnd->strings[2].align = AlignLeft;
		wnd->strings[2].font = font6x8;
		wnd->strings[2].inverted = NotInverted;
		sprintf(wnd->strings[2].Text, "Set:");

        wnd->strings[3].x_pos = 40;
        wnd->strings[3].y_pos = 28;
        wnd->strings[3].align = AlignCenter;
        wnd->strings[3].font = font6x8;
        wnd->strings[3].inverted = Inverted;
        sprintf(wnd->strings[3].Text, "%d", calibration_data_ptr[calibration_current_item - 1]);

        wnd->strings[4].x_pos = 0;
        wnd->strings[4].y_pos = 38;
        wnd->strings[4].align = AlignCenter;
        wnd->strings[4].font = font6x8;
        wnd->strings[4].inverted = NotInverted;
//        calibration_data_ptr[calibration_current_item + 2] = data->measured_current_raw; // TODO: uncomment after device assembly
        sprintf(wnd->strings[4].Text, "Read: %d", calibration_data_ptr[calibration_current_item + 2]);

        wnd->strings[5].x_pos = 0;
		wnd->strings[5].y_pos = 55;
		wnd->strings[5].align = AlignCenter;
		wnd->strings[5].font = font6x8;
		wnd->strings[5].inverted = Inverted;
		sprintf(wnd->strings[5].Text, "   Ok   ");

        wnd->StringsQuantity = 6;
    }
    else
    {
        if(calibration_current_item == calibration_items_num)
        {
            wnd->strings[1].x_pos = 0;
            wnd->strings[1].y_pos = 25;
            wnd->strings[1].align = AlignCenter;
            wnd->strings[1].font = font6x8;
            wnd->strings[1].inverted = NotInverted;
            sprintf(wnd->strings[1].Text, "Calibration is ended");

            wnd->strings[2].x_pos = 0;
            wnd->strings[2].y_pos = 55;
            wnd->strings[2].align = AlignCenter;
            wnd->strings[2].font = font6x8;
            wnd->strings[2].inverted = Inverted;
            sprintf(wnd->strings[2].Text, "   Save   ");

            wnd->StringsQuantity = 3;
        }
        else
        {
            sprintf(wnd->strings[0].Text, "%s calibration", modes[calibration_current_item>>2]);

            wnd->strings[1].x_pos = 0;
            wnd->strings[1].y_pos = 18;
            wnd->strings[1].align = AlignCenter;
            wnd->strings[1].font = font6x8;
            wnd->strings[1].inverted = NotInverted;
            sprintf(wnd->strings[1].Text, "Set voltage %s", refs[calibration_current_item-1]);

            wnd->strings[2].x_pos = 0;
            wnd->strings[2].y_pos = 28;
            wnd->strings[2].align = AlignCenter;
            wnd->strings[2].font = font6x8;
            wnd->strings[2].inverted = NotInverted;
//            calibration_data_ptr[calibration_current_item + 2] = data->voltage_raw; // TODO: uncomment after device assembly
            sprintf(wnd->strings[2].Text, "Read: %d", calibration_data_ptr[calibration_current_item + 2]);

            wnd->strings[3].x_pos = 0;
            wnd->strings[3].y_pos = 55;
            wnd->strings[3].align = AlignCenter;
            wnd->strings[3].font = font6x8;
            wnd->strings[3].inverted = Inverted;
            sprintf(wnd->strings[3].Text, "   Ok   ");

            wnd->StringsQuantity = 4;
        }
    }

    st7565_drv->setWindow(wnd);
    return 1;
}

/**
  * @brief  Callback function for drawing battery voltage window
  * @param  wnd - data structure with window parameters
  * @param  data - data structure with electronic load parameters
  * @param  item_action: NoAction - do nothing, Next - go to the next item, Prev - go to previous item
  * @retval 0 - after window drawing go to previous window, 1 - after window drawing stay in it
  */
static int SetupBatteryWindow(pWindow wnd, pData data, Action item_action, Action value_action)
{
    switch(item_action)
    {
        case NoAction:
        case Prev:
        default:
            break;

        case Next:
            return 0;
            break;
    }

    wnd->strings[0].x_pos = 0;
    wnd->strings[0].y_pos = 25;
    wnd->strings[0].align = AlignCenter;
    wnd->strings[0].font = font6x8;
    wnd->strings[0].inverted = NotInverted;
    sprintf(wnd->strings[0].Text, "Vbat = %0.2f B", data->vbat);

    wnd->strings[1].x_pos = 0;
    wnd->strings[1].y_pos = 55;
    wnd->strings[1].align = AlignCenter;
    wnd->strings[1].font = font6x8;
    wnd->strings[1].inverted = Inverted;
    sprintf(wnd->strings[1].Text, "   Ok   ");

    wnd->StringsQuantity = 2;

    st7565_drv->setWindow(wnd);
    return 1;
}

/**
  * @brief  Callback function for drawing display settings window
  * @param  wnd - data structure with window parameters
  * @param  data - data structure with electronic load parameters
  * @param  item_action: NoAction - do nothing, Next - go to the next item, Prev - go to previous item
  * @retval 0 - after window drawing go to previous window, 1 - after window drawing stay in it
  */
static int SetupDisplayWindow(pWindow wnd, pData data, Action item_action, Action value_action)
{
    switch(item_action)
    {
        case NoAction:
        case Prev:
        default:
            break;

        case Next:
            return 0;
            break;
    }

    switch(value_action)
    {
        case NoAction:
        default:
            break;

        case Prev:
            if(data->load_settings.display_contrast > 0) data->load_settings.display_contrast--;
            else data->load_settings.display_contrast = 0;
            break;

        case Next:
            if(data->load_settings.display_contrast < 32) data->load_settings.display_contrast++;
            else data->load_settings.display_contrast = 32;
            break;
    }

    // set display contrast
    st7565_drv->setBrightness(data->load_settings.display_contrast);

    wnd->strings[0].x_pos = 15;
    wnd->strings[0].y_pos = 25;
    wnd->strings[0].align = AlignLeft;
    wnd->strings[0].font = font6x8;
    wnd->strings[0].inverted = NotInverted;
    sprintf(wnd->strings[0].Text, "Contrast:");

    wnd->strings[1].x_pos = 80;
    wnd->strings[1].y_pos = 25;
    wnd->strings[1].align = AlignLeft;
    wnd->strings[1].font = font6x8;
    wnd->strings[1].inverted = Inverted;
    sprintf(wnd->strings[1].Text, "%d", data->load_settings.display_contrast);

    wnd->strings[2].x_pos = 0;
    wnd->strings[2].y_pos = 55;
    wnd->strings[2].align = AlignCenter;
    wnd->strings[2].font = font6x8;
    wnd->strings[2].inverted = Inverted;
    sprintf(wnd->strings[2].Text, "   Ok   ");

    wnd->StringsQuantity = 3;

    st7565_drv->setWindow(wnd);
    return 1;
}

/**
  * @brief  This function returns change step of setting current
  * @param  current - setting current value
  * @retval step - step value for the given current
  */
static float GetCurrentChangeStep(float current)
{
    float current_change_step;

    if(current < 0.499f) current_change_step = 0.01f;
    else if(current < 4.999f) current_change_step = 0.05f;
    else current_change_step = 0.1f;

    return current_change_step;
}


/*
 * display_windows.c
 *
 *  Created on: 5 сент. 2023 г.
 *      Author: Kuprin_IV
 */
#include "display_windows.h"
#include <stdio.h>

static float GetCurrentChangeStep(float current);

int DisplayMainWindow(pWindow wnd, pData data, Action item_action, Action value_action)
{
    float max_current;
    float step = 0.01f;


    max_current = (float)data->max_power/data->voltage;
    if(max_current > 20.0f) max_current = 20.0f;

    // round max current value to nearest step value
    step = GetCurrentChangeStep(max_current);
    int steps_num = (int)(max_current/step);
    max_current = steps_num*step;

    step = GetCurrentChangeStep(data->current);

    switch(value_action)
    {
        case NoAction:
        default:
            break;

        case Prev:
            if(data->current > 0.01f) data->current -= step;
            else data->current = 0;
            break;

        case Next:
            if(data->current < max_current) data->current += step;
            else data->current = max_current;
            break;
    }

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
    wnd->strings[2].y_pos = 15;
    wnd->strings[2].align = AlignCenter;
    wnd->strings[2].font = MSSanSerif_14;
    wnd->strings[2].inverted = NotInverted;
    if(data->current >= 5.0f)
    {
        sprintf(wnd->strings[2].Text, "%0.1f B  %0.1f A", data->voltage, data->current);
    }
    else
    {
        sprintf(wnd->strings[2].Text, "%0.1f B  %0.2f A", data->voltage, data->current);
    }

    wnd->strings[3].x_pos = 0;
    wnd->strings[3].y_pos = 35;
    wnd->strings[3].align = AlignCenter;
    wnd->strings[3].font = font6x8;
    wnd->strings[3].inverted = NotInverted;
    sprintf(wnd->strings[3].Text, "%0.0f mAh  %0.1f Wh", data->mAh, data->Wh);

    wnd->strings[4].x_pos = 0;
    wnd->strings[4].y_pos = 55;
    wnd->strings[4].align = AlignCenter;
    wnd->strings[4].font = font6x8;
    wnd->strings[4].inverted = Inverted;
    sprintf(wnd->strings[4].Text, "   Settings   ");

    wnd->strings[5].x_pos = 0;
    wnd->strings[5].y_pos = 44;
    wnd->strings[5].align = AlignCenter;
    wnd->strings[5].font = font6x8;
    wnd->strings[5].inverted = NotInverted;
    if(data->load_work_mode == BatteryDischarge)
    {
        if(data->current >= 5.0f)
        {
            sprintf(wnd->strings[5].Text, "Vdis:%0.2fV Iset:%0.1fA", data->discharge_voltage, data->current);
        }
        else
        {
            sprintf(wnd->strings[5].Text, "Vdis:%0.2fV Iset:%0.2fA", data->discharge_voltage, data->current);
        }
    }
    else
    {
        if(data->current >= 5.0f)
        {
            sprintf(wnd->strings[5].Text, "Iset:%0.1fA", data->current);
        }
        else
        {
            sprintf(wnd->strings[5].Text, "Iset:%0.2fA", data->current);
        }
    }

    wnd->StringsQuantity = 6;

    st7565_drv->drawBatteryIndicator(105, 2, 8);
    st7565_drv->drawFanIndicator(56, 0);

    st7565_drv->setWindow(wnd);
    return 0;
}

int SetMenuWindow(pWindow wnd,pData data, Action item_action, Action value_action)
{
    const char* items[SETTINGS_ITEMS_NUM]  = {"Mode\0","Max power\0","Calibration\0","Battery\0","Quit\0"};
    uint8_t items_num = SETTINGS_ITEMS_NUM;
    int max_item_per_screen = 6;

    wnd->StringsQuantity = (items_num <= max_item_per_screen ? items_num : max_item_per_screen);

    switch(item_action)
    {
        case NoAction:
        default:
            break;

        case Next:
            if(++data->menu_current_item > items_num)
            {
                data->menu_current_item = 1;
            }
            break;

        case Prev:
            if(--data->menu_current_item < 1)
            {
                data->menu_current_item = items_num;
            }
            break;
    }

    if(data->menu_current_item <= max_item_per_screen)
    {
        for(uint8_t i = 0; i < (items_num <= max_item_per_screen ? items_num : max_item_per_screen); i++)
        {
            wnd->strings[i].x_pos = 2;
            wnd->strings[i].y_pos = 2+10*i;
            wnd->strings[i].align = AlignLeft;
            wnd->strings[i].font = font6x8;
            wnd->strings[i].inverted = (i == data->menu_current_item-1)?(Inverted):(NotInverted);
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
            sprintf(wnd->strings[i].Text, "%s", items[data->menu_current_item-max_item_per_screen+i]);
        }
    }
    st7565_drv->setWindow(wnd);
    return 0;
}

int SetupModeWindow(pWindow wnd, pData data, Action item_action, Action value_action)
{
    const char* modes[2]  = {"Simple\0","Discharge\0"};
    int mode_items_num = data->load_work_mode == BatteryDischarge ? 3 : 2;
    float step = 0.05f;

    if(data->discharge_voltage < 5.0f) step = 0.05f;
    else if(data->discharge_voltage < 20.0f) step = 0.1f;
    else step = 0.5f;

    switch(item_action)
    {
        case NoAction:
        case Prev:
        default:
            break;

        case Next:
            if(++data->mode_current_item > mode_items_num)
            {
                data->mode_current_item = 1;
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
            if(data->mode_current_item == 1)
            {
                data->load_work_mode = (LoadMode)((data->load_work_mode + 1) & 0x01);
            }
            else if(data->mode_current_item == 2)
            {
                if(data->discharge_voltage < 50.0f) data->discharge_voltage += step;
                else data->discharge_voltage = 50.0f;
            }
            break;

        case Prev:
            if(data->mode_current_item == 1)
            {
                data->load_work_mode = (LoadMode)((data->load_work_mode - 1) & 0x01);
            }
            else if(data->mode_current_item == 2)
            {
                if(data->discharge_voltage > 0.05f) data->discharge_voltage -= step;
                else data->discharge_voltage = 0.0f;
            }
            break;
    }

    wnd->strings[0].x_pos = 10;
    wnd->strings[0].y_pos = 5;
    wnd->strings[0].align = AlignLeft;
    wnd->strings[0].font = font6x8;
    wnd->strings[0].inverted = NotInverted;
    sprintf(wnd->strings[1].Text, "Mode:");

    wnd->strings[1].x_pos = 50;
    wnd->strings[1].y_pos = 5;
    wnd->strings[1].align = AlignCenter;
    wnd->strings[1].font = font6x8;
    wnd->strings[1].inverted = data->mode_current_item == 1 ? Inverted : NotInverted;
    sprintf(wnd->strings[1].Text, "%s", modes[data->load_work_mode]);

    if(data->load_work_mode == BatteryDischarge)
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
        wnd->strings[3].inverted = data->mode_current_item == 2 ? Inverted : NotInverted;
        sprintf(wnd->strings[3].Text, "%0.2f V", data->discharge_voltage);


        wnd->strings[4].x_pos = 0;
        wnd->strings[4].y_pos = 55;
        wnd->strings[4].align = AlignCenter;
        wnd->strings[4].font = font6x8;
        wnd->strings[4].inverted = data->mode_current_item == mode_items_num ? Inverted : NotInverted;
        sprintf(wnd->strings[4].Text, "  < Back    ");

        wnd->StringsQuantity = 5;
    }
    else
    {
        wnd->strings[2].x_pos = 0;
        wnd->strings[2].y_pos = 55;
        wnd->strings[2].align = AlignCenter;
        wnd->strings[2].font = font6x8;
        wnd->strings[2].inverted = data->mode_current_item == mode_items_num ? Inverted : NotInverted;
        sprintf(wnd->strings[2].Text, "  < Back    ");

        wnd->StringsQuantity = 3;
    }


    st7565_drv->setWindow(wnd);

    return 1;
}

int SetupMaxPowerWindow(pWindow wnd, pData data, Action item_action, Action value_action)
{
    const char* max_power_space[3] = {" ", "  ", "   "};
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
            if(data->max_power > 10) data->max_power--;
            else data->max_power = 10;
            break;

        case Next:
            if(data->max_power < 250) data->max_power++;
            else data->max_power = 250;
            break;
    }

    if(data->max_power >= 100)
    {
        index = 2;
    }
    else if(data->max_power >= 10)
    {
        index = 1;
    }
    else
    {
        index = 0;
    }

    if(data->current > (float)data->max_power/data->voltage+0.01f)
    {
        data->current = (float)data->max_power/data->voltage;
        // round current value to nearest step value
        step = GetCurrentChangeStep(data->current);
        steps_num = (int)(data->current/step);
        data->current = steps_num*step;
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
    sprintf(wnd->strings[1].Text, "%d", data->max_power);

    wnd->strings[2].x_pos = 0;
    wnd->strings[2].y_pos = 55;
    wnd->strings[2].align = AlignCenter;
    wnd->strings[2].font = font6x8;
    wnd->strings[2].inverted = Inverted;
    sprintf(wnd->strings[1].Text, "   Ok   ");

    wnd->StringsQuantity = 3;

    st7565_drv->setWindow(wnd);
    return 1;
}

int SetupCalibrationWindow(pWindow wnd, pData data, Action item_action, Action value_action)
{
    const char* modes[2]  = {"Current\0","Voltage\0"};
    const char* refs[6] = {"0,1A", "1A", " 5A", "1V", "10V", "25V"};
    int calibration_items_num = 7;
    uint16_t* calibration_data_ptr = (uint16_t*)&data->calibration_data;
    uint16_t coef_temp = 0;

    switch(item_action)
    {
        case NoAction:
        case Prev:
        default:
            break;

        case Next:
            if(++data->calibration_current_item > calibration_items_num)
            {
                data->calibration_current_item = 1;
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
            if(data->calibration_current_item < 4)
            {
                calibration_data_ptr[data->calibration_current_item - 1]++;
            }
            break;

        case Prev:
            if(data->calibration_current_item < 4)
            {
                calibration_data_ptr[data->calibration_current_item - 1]--;
            }
            break;
    }

    coef_temp = calibration_data_ptr[data->calibration_current_item - 1];

    wnd->strings[0].x_pos = 0;
    wnd->strings[0].y_pos = 5;
    wnd->strings[0].align = AlignCenter;
    wnd->strings[0].font = font6x8;
    wnd->strings[0].inverted = NotInverted;

    if(data->calibration_current_item < 4)
    {
        sprintf(wnd->strings[0].Text, "%s calibration", modes[(data->calibration_current_item-1)>>2]);

        wnd->strings[1].x_pos = 0;
        wnd->strings[1].y_pos = 25;
        wnd->strings[1].align = AlignCenter;
        wnd->strings[1].font = font6x8;
        wnd->strings[1].inverted = NotInverted;
        sprintf(wnd->strings[1].Text, "Set current %s", refs[data->calibration_current_item-1]);

        wnd->strings[2].x_pos = 0;
        wnd->strings[2].y_pos = 35;
        wnd->strings[2].align = AlignCenter;
        wnd->strings[2].font = font6x8;
        wnd->strings[2].inverted = Inverted;
        sprintf(wnd->strings[2].Text, "%d", coef_temp);

        wnd->strings[3].x_pos = 0;
		wnd->strings[3].y_pos = 55;
		wnd->strings[3].align = AlignCenter;
		wnd->strings[3].font = font6x8;
		wnd->strings[3].inverted = Inverted;
		sprintf(wnd->strings[3].Text, "   Ok   ");

        wnd->StringsQuantity = 4;
    }
    else
    {
        if(data->calibration_current_item == calibration_items_num)
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
            sprintf(wnd->strings[0].Text, "%s calibration", modes[data->calibration_current_item>>2]);

            wnd->strings[1].x_pos = 0;
            wnd->strings[1].y_pos = 25;
            wnd->strings[1].align = AlignCenter;
            wnd->strings[1].font = font6x8;
            wnd->strings[1].inverted = NotInverted;
            sprintf(wnd->strings[1].Text, "Set voltage %s", refs[data->calibration_current_item-1]);

            wnd->strings[2].x_pos = 0;
            wnd->strings[2].y_pos = 35;
            wnd->strings[2].align = AlignCenter;
            wnd->strings[2].font = font6x8;
            wnd->strings[2].inverted = NotInverted;
            sprintf(wnd->strings[2].Text, "%d", coef_temp);

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

int SetupBatteryWindow(pWindow wnd, pData data, Action item_action, Action value_action)
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

static float GetCurrentChangeStep(float current)
{
    float current_change_step;

    if(current < 0.499f) current_change_step = 0.01f;
    else if(current < 4.999f) current_change_step = 0.05f;
    else current_change_step = 0.1f;

    return current_change_step;
}


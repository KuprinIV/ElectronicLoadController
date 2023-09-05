/*
 * load_control.h
 *
 *  Created on: 5 сент. 2023 г.
 *      Author: Kuprin_IV
 */

#ifndef INC_LOAD_CONTROL_H_
#define INC_LOAD_CONTROL_H_

#include "main.h"

typedef enum {SimpleLoad, BatteryDischarge} LoadMode;

typedef struct
{
   uint16_t current_0A1;
   uint16_t current_1A;
   uint16_t current_5A;
   uint16_t voltage_1V;
   uint16_t voltage_10V;
   uint16_t voltage_25V;
}CalibrationData;

typedef struct
{
    float current;
    float voltage;
    float mAh;
    float Wh;
    float temperature;
    int rpm;
    int menu_current_item;
    LoadMode load_work_mode;
    float discharge_voltage;
    int mode_current_item;
    int calibration_current_item;
    CalibrationData calibration_data;
    uint8_t is_ovt;
    float vbat;
    int max_power;
}Data,*pData;

#endif /* INC_LOAD_CONTROL_H_ */

/*
 * load_control.h
 *
 *  Created on: 5 сент. 2023 г.
 *      Author: Kuprin_IV
 */

#ifndef INC_LOAD_CONTROL_H_
#define INC_LOAD_CONTROL_H_

#include "main.h"

#define NUM_OF_SAMPLES 			100
#define EEPROM_CAL_DATA_ADDR 	0x08080000
#define MAH_CALC_LIMIT 			0.01f

typedef enum {SimpleLoad, BatteryDischarge} LoadMode;

typedef struct
{
   uint16_t current_set_0A1;
   uint16_t current_set_1A;
   uint16_t current_set_5A;
   uint16_t current_read_0A1;
   uint16_t current_read_1A;
   uint16_t current_read_5A;
   uint16_t voltage_1V;
   uint16_t voltage_10V;
   uint16_t voltage_25V;
}CalibrationData;

typedef struct
{
	volatile uint8_t is_update_event;
	volatile uint8_t is_conversion_ended;
    float measured_current;
    float set_current;
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
    uint16_t vdac0;
    uint16_t vref;
    uint16_t measured_current_raw;
    uint16_t set_current_raw;
    uint16_t voltage_raw;

}Data,*pData;

typedef struct
{
	void (*loadInit)(void);
	void (*setCurrent)(uint16_t val);
	int16_t (*getEncoderOffset)(void);
	void (*saveCalibrationData)(CalibrationData* cd);
}LoadController;

extern LoadController* load_control_drv;

#endif /* INC_LOAD_CONTROL_H_ */

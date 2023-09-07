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
	// flags
	volatile uint8_t is_update_event;
	volatile uint8_t is_conversion_ended;
	uint8_t is_ovt;
	// set and measured parameters
	float set_current;
    float measured_current;
    float voltage;
    float mAh;
    float Wh;
    float temperature;
    float vbat;
    uint16_t rpm;
    // calibration data
    CalibrationData calibration_data;
    // settings
    LoadMode load_work_mode;
    float discharge_voltage;
    uint16_t max_power;
    // raw ADC measured data
    uint16_t vdac0_raw;
    uint16_t vref_raw;
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
	void (*calcMeasuredParams)(void);
	void (*setFanSpeed)(uint8_t fs);
	void (*powerControl)(uint8_t is_on);
	uint8_t (*checkOvertemperature)(void);
}LoadController;

extern LoadController* load_control_drv;

#endif /* INC_LOAD_CONTROL_H_ */

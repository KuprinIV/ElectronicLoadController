/*
 * load_control.h
 *
 *  Created on: 5 сент. 2023 г.
 *      Author: Kuprin_IV
 */

#ifndef INC_LOAD_CONTROL_H_
#define INC_LOAD_CONTROL_H_

#include "main.h"
#include "ds18b20.h"

#define NUM_OF_SAMPLES 			100
#define EEPROM_CAL_DATA_ADDR 	0x08080000
#define EEPROM_LOAD_SET_ADDR 	0x08080020
#define IS_EEPROM_WRITTEN_SIGN	0x55555555
#define MAH_CALC_LIMIT 			0.01f
#define VBAT_LOW				3.4f
#define POWER_OFF_TICKS			30
#define TEMP_UPDATE_TICKS		20
#define FAN_SPEED_CTRL_TICKS	200
#define FAN_SPEED_AVG_TICKS		10

typedef enum {SimpleLoad, BatteryDischarge, Ramp} LoadMode;

typedef struct
{
   uint16_t current_set_0A1;
   uint16_t current_set_1A;
   uint16_t current_set_5A;
   uint16_t current_read_0A1;
   uint16_t current_read_1A;
   uint16_t current_read_5A;
   uint16_t voltage_2V;
   uint16_t voltage_10V;
   uint16_t voltage_25V;
}CalibrationData;

typedef struct
{
    LoadMode load_work_mode;
    float discharge_voltage;
    uint16_t max_power;
    uint8_t display_contrast;
    uint16_t current_ramp_time;
}LoadSettings;

typedef struct
{
	// flags
	volatile uint8_t is_update_event;
	volatile uint8_t is_conversion_ended;
	uint8_t is_ovt;
	uint8_t on_state;
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
    LoadSettings load_settings;
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
	void (*readCalibrationData)(void);
	void (*readSettingsData)(void);
	void (*setCurrentInDiscreets)(uint16_t val);
	void (*setCurrentInAmperes)(float val);
	void (*setEnabled)(uint8_t state);
	int16_t (*getEncoderOffset)(void);
	void (*saveCalibrationData)(CalibrationData* cd);
	void (*saveLoadSettings)(LoadSettings* ls);
	void (*calcMeasuredParams)(void);
	void (*setFanSpeed)(uint8_t fs);
	void (*powerControl)(uint8_t is_on);
	uint8_t (*checkOvertemperature)(void);
	void (*checkPowerButton)(void);
	void (*updateTemperature)(void);
	void (*fanSpeedControl)(void);
}LoadController;

extern LoadController* load_control_drv;

#endif /* INC_LOAD_CONTROL_H_ */

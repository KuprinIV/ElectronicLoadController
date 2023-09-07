/*
 * load_control.c
 *
 *  Created on: 6 сент. 2023 г.
 *      Author: Kuprin_IV
 */
#include "load_control.h"
#include <string.h>

// driver functions
static void loadInit(void);
static void setCurrent(uint16_t val);
static int16_t getEncoderOffset(void);
static void saveCalibrationData(CalibrationData* cd);
static void calcMeasuredParams(void);
static void setFanSpeed(uint8_t fs);

// inner functions
static float calcCurrent(uint16_t adc_val);
static float calcVoltage(uint16_t adc_val);

LoadController lc_driver = {
		loadInit,
		setCurrent,
		getEncoderOffset,
		saveCalibrationData,
		calcMeasuredParams,
		setFanSpeed,
};
LoadController* load_control_drv =  &lc_driver;

extern ADC_HandleTypeDef hadc;
extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim6;

static uint16_t encoder_cntr = 32767;
static uint16_t encoder_cntr_prev = 32767;
static volatile uint32_t rpm_cntr = 0;
static uint16_t adcSamples[5*NUM_OF_SAMPLES] = {0};

Data loadData = {0, 0, 0.0f, 0.1f, 0.0f, 0.0f, 0.0f, 20.0f, 0, 1, SimpleLoad, 3.0f, 1, 1, {50, 500, 2500, 50, 500, 2500, 80, 800, 2000},
		0, 4.2f, 250, 0, 0, 0, 0, 0};

/**
  * @brief  Electronic load control initialization
  * @param  none
  * @retval none
  */
static void loadInit(void)
{
	// read calibration data, if it was saved
	if((*(__IO uint32_t *)EEPROM_CAL_DATA_ADDR) != 0)
	{
		memcpy(&loadData.calibration_data, (uint32_t *)EEPROM_CAL_DATA_ADDR, sizeof(loadData.calibration_data));
	}

	// start DAC
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0); // set default value

	// start ADC conversion
	HAL_TIM_Base_Start(&htim6);
	HAL_ADC_Start_DMA(&hadc, (uint32_t*)adcSamples, sizeof(adcSamples)/sizeof(uint16_t));
}

/**
  * @brief  Electronic load set current
  * @param  val - current value in DAC discreets
  * @retval none
  */
static void setCurrent(uint16_t val)
{
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, val); // set default value
}

/**
  * @brief  This function returns encoder offset value
  * @param  none
  * @retval encoder offset value (0 - no offset, <0 - turned by counterclockwise, >0 - turned by clockwise)
  */
static int16_t getEncoderOffset(void)
{
	int16_t offset = 0;
	encoder_cntr = TIM2->CNT;
	if(encoder_cntr > encoder_cntr_prev+1 || encoder_cntr < encoder_cntr_prev-1)
	{
		offset = (int16_t)(encoder_cntr_prev - encoder_cntr);
		encoder_cntr_prev = encoder_cntr;
	}
	return offset;
}

/**
  * @brief  Save calibration data in EEPROM
  * @param  cd - pointer to data structure with calibration data
  * @retval none
  */
static void saveCalibrationData(CalibrationData* cd)
{
	HAL_StatusTypeDef flash_ok = HAL_ERROR;
	uint8_t num_of_words = (sizeof(CalibrationData)>>2)+1;
	uint32_t* cd_ptr = (uint32_t*)cd;

	HAL_FLASHEx_DATAEEPROM_Unlock();

	// erase EEPROM
	for(uint8_t i = 0; i < num_of_words; i++)
	{
		flash_ok = HAL_FLASHEx_DATAEEPROM_Erase(EEPROM_CAL_DATA_ADDR+4*i);
	}

	// save calibration coefficients
	if(flash_ok == HAL_OK)
	{
		for(uint8_t i = 0; i < num_of_words; i++)
		{
			flash_ok = HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, EEPROM_CAL_DATA_ADDR+4*i, cd_ptr[i]);
		}

		if(flash_ok != HAL_OK)
		{
			Error_Handler();
		}
	}

	HAL_FLASHEx_DATAEEPROM_Lock();
}

/**
  * @brief  Calculate displayed parameters after ADC conversion end
  * @param  none
  * @retval none
  */
static void calcMeasuredParams(void)
{
	uint32_t adc_averaged_data[5] = {0};
	// calc averaged ADC data
	for(uint16_t i = 0; i < NUM_OF_SAMPLES; i++)
	{
		adc_averaged_data[i%5] += adcSamples[i];
	}
	for(uint16_t i = 0; i < 5; i++)
	{
		adc_averaged_data[i] /= NUM_OF_SAMPLES;
	}
	// convert them
	loadData.vbat = 2*adc_averaged_data[0]*3.3f/4096;
	loadData.vdac0 = adc_averaged_data[1];
	loadData.vref = adc_averaged_data[2];
	loadData.measured_current_raw = adc_averaged_data[3];
	loadData.voltage_raw = (adc_averaged_data[4]);

	loadData.measured_current = calcCurrent(adc_averaged_data[3]);
	loadData.voltage = calcVoltage(adc_averaged_data[4]);

	// calc mAh and Wh
	if(loadData.measured_current >= MAH_CALC_LIMIT)
	{
		loadData.mAh += loadData.measured_current/36;
		loadData.Wh += loadData.measured_current*loadData.voltage/36000;
	}
}

/**
  * @brief  Set fan speed
  * @param  fs - fan speed duty cycle (0...99)
  * @retval none
  */
static void setFanSpeed(uint8_t fs)
{
	if(fs > 99) fs = 99;
	TIM21->CCR2 = fs;
}

/**
  * @brief  Calculate current value in amperes from ADC discreets
  * @param  adc_val - current value in ADC discreets
  * @retval current value in amperes
  */
static float calcCurrent(uint16_t adc_val)
{
	float current = 0.0f;

	if(adc_val < loadData.calibration_data.current_read_0A1)
	{
		current = 0.1f - 0.9f*(loadData.calibration_data.current_read_0A1 - adc_val)/(loadData.calibration_data.current_read_1A-loadData.calibration_data.current_read_0A1);
	}
	else if(adc_val >= loadData.calibration_data.current_read_0A1 && adc_val < loadData.calibration_data.current_read_1A)
	{
		current = 0.1f + 0.9f*(adc_val - loadData.calibration_data.current_read_0A1)/(loadData.calibration_data.current_read_1A-loadData.calibration_data.current_read_0A1);
	}
	else if(adc_val >= loadData.calibration_data.current_read_1A && adc_val < loadData.calibration_data.current_read_5A)
	{
		current = 1.0f + 4.0f*(adc_val - loadData.calibration_data.current_read_1A)/(loadData.calibration_data.current_read_5A-loadData.calibration_data.current_read_1A);
	}
	else
	{
		current = 5.0f + 4.0f*(adc_val - loadData.calibration_data.current_read_5A)/(loadData.calibration_data.current_read_5A-loadData.calibration_data.current_read_1A);
	}

	return current;
}

/**
  * @brief  Calculate voltage value in volts from ADC discreets
  * @param  adc_val - voltage value in ADC discreets
  * @retval voltage value in volts
  */
static float calcVoltage(uint16_t adc_val)
{
	float voltage = 0.0f;

	if(adc_val < loadData.calibration_data.voltage_1V)
	{
		voltage = 1.0f - 9.0f*(loadData.calibration_data.voltage_1V - adc_val)/(loadData.calibration_data.voltage_10V-loadData.calibration_data.voltage_1V);
	}
	else if(adc_val >= loadData.calibration_data.voltage_1V && adc_val < loadData.calibration_data.voltage_10V)
	{
		voltage = 1.0f + 9.0f*(adc_val - loadData.calibration_data.voltage_1V)/(loadData.calibration_data.voltage_10V-loadData.calibration_data.voltage_1V);
	}
	else if(adc_val >= loadData.calibration_data.voltage_10V && adc_val < loadData.calibration_data.voltage_25V)
	{
		voltage = 10.0f + 15.0f*(adc_val - loadData.calibration_data.voltage_10V)/(loadData.calibration_data.voltage_25V-loadData.calibration_data.voltage_10V);
	}
	else
	{
		voltage = 25.0f + 15.0f*(adc_val - loadData.calibration_data.voltage_25V)/(loadData.calibration_data.voltage_25V-loadData.calibration_data.voltage_10V);
	}

	return voltage;
}

/**
 * @brief ADC conversion end callback
 * @param: hadc - ADC handle
 * @return: None
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	UNUSED(hadc);
	// set flag
	loadData.is_conversion_ended = 1;
}

/**
  * @brief  Input Capture callback in non-blocking mode
  * @param  htim TIM IC handle
  * @retval None
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	/* Captured Values */
	static uint32_t uwIC2Value1 = 0;
	static uint32_t uwIC2Value2 = 0;
	static uint32_t uwDiffCapture = 0;

	/* Capture index */
	static uint16_t uhCaptureIndex = 0;

	if(htim->Instance == TIM21)
	{
		if(uhCaptureIndex == 0)
		{
			/* Get the 1st Input Capture value */
			uwIC2Value1 = TIM21->CCR1;
			uhCaptureIndex = 1;

			/* Capture computation */
			if (uwIC2Value1 > uwIC2Value2)
			{
				uwDiffCapture = (uwIC2Value1 - uwIC2Value2);
			}
			else if (uwIC2Value1 < uwIC2Value2)
			{
				/* 0xFFFF is max TIM1_CCRx value */
				uwDiffCapture = uwIC2Value2 + uwIC2Value1;
			}
			else
			{
				/* If capture values are equal, we have reached the limit of frequency
					 measures */
				uwDiffCapture = 1;
			}

			uwDiffCapture += rpm_cntr*(TIM21->ARR+1);
		}
		else if(uhCaptureIndex == 1)
		{
			/* Get the 2nd Input Capture value */
			uwIC2Value2 = TIM21->CCR1;

			/* Capture computation */
			if (uwIC2Value2 > uwIC2Value1)
			{
				uwDiffCapture = (uwIC2Value2 - uwIC2Value1);
			}
			else if (uwIC2Value2 < uwIC2Value1)
			{
				/* 0xFFFF is max TIM1_CCRx value */
				uwDiffCapture = uwIC2Value1 + uwIC2Value2;
			}
			else
			{
				/* If capture values are equal, we have reached the limit of frequency
					 measures */
				uwDiffCapture = 1;
			}

			uwDiffCapture += rpm_cntr*(TIM21->ARR+1);
			uhCaptureIndex = 0;
		}
		/* Frequency computation: for this example TIMx (TIM1) is clocked by APB1Clk */
		loadData.rpm = 19200000/uwDiffCapture;
		uwDiffCapture = 0;
		rpm_cntr = 0;
	}
}

/**
  * @brief  Period elapsed callback in non-blocking mode
  * @param  htim TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM21)
	{
		if(rpm_cntr < 3500)
			rpm_cntr++;
		else
		{
			loadData.rpm = 0;
		}
	}
	else if(htim->Instance == TIM22)
	{
		loadData.is_update_event = 1;
	}
}


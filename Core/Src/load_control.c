/*
 * load_control.c
 *
 *  Created on: 6 сент. 2023 г.
 *      Author: Kuprin_IV
 */
#include "load_control.h"
#include "st7565.h"
#include <string.h>

#define ABS(x) (x) >= 0 ? (x):(-x)

// driver functions
static void loadInit(void);
static void setCurrentInDiscretes(uint16_t val);
static void setCurrentInAmperes(float val);
static void setEnabled(uint8_t state);
static int16_t getEncoderOffset(void);
static void saveCalibrationData(CalibrationData* cd);
static void saveLoadSettings(LoadSettings* ls);
static void calcMeasuredParams(void);
static void setFanSpeed(uint8_t fs);
static void powerControl(uint8_t is_on);
static uint8_t checkOvertemperature(void);
static void checkPowerButton(void);
static void updateTemperatureValue(void);
static void fanSpeedControl(void);

// inner functions
static void setDacValue(uint16_t val);
static float calcCurrent2Float(uint16_t adc_val);
static uint16_t calcCurrent2Discrete(float ampere_val);
static float calcVoltage(uint16_t adc_val);

LoadController lc_driver = {
		loadInit,
		setCurrentInDiscretes,
		setCurrentInAmperes,
		setEnabled,
		getEncoderOffset,
		saveCalibrationData,
		saveLoadSettings,
		calcMeasuredParams,
		setFanSpeed,
		powerControl,
		checkOvertemperature,
		checkPowerButton,
		updateTemperatureValue,
		fanSpeedControl
};
LoadController* load_control_drv =  &lc_driver;

extern ADC_HandleTypeDef hadc;
extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim6;
extern UART_HandleTypeDef huart1;

static uint16_t encoder_cntr = 32767;
static uint16_t encoder_cntr_prev = 32767;
static volatile uint32_t rpm_cntr = 0;
static uint16_t adcSamples[5*NUM_OF_SAMPLES] = {0};
static uint16_t rampVals[101] = {0};

Data loadData = {0, 0, 0, 0, 0.1f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 4.0f, 0, {205, 380, 1150, 10, 100, 500, 120, 600, 1500},
				{SimpleLoad, 3.0f, 250, 10, 50}, 0, 0, 0, 0, 0};

/**
  * @brief  Electronic load control initialization
  * @param  none
  * @retval none
  */
static void loadInit(void)
{
	// read calibration data, if it was saved
	if((*(__IO uint32_t *)EEPROM_CAL_DATA_ADDR) == IS_EEPROM_WRITTEN_SIGN)
	{
		memcpy(&loadData.calibration_data, (uint8_t*)EEPROM_CAL_DATA_ADDR+4, sizeof(CalibrationData));
	}

	// read load settings data, if it was saved
	if((*(__IO uint32_t *)EEPROM_LOAD_SET_ADDR) == IS_EEPROM_WRITTEN_SIGN)
	{
		memcpy(&loadData.load_settings, (uint8_t*)EEPROM_LOAD_SET_ADDR+4, sizeof(LoadSettings));
	}

	// start DAC
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	setCurrentInAmperes(0.0f); // set default value

	// start ADC conversion
	HAL_TIM_Base_Start(&htim6);
	HAL_ADC_Start_DMA(&hadc, (uint32_t*)adcSamples, sizeof(adcSamples)/sizeof(uint16_t));

	// init DS18B20 temperature sensor
	ds18b20_drv->Init(&huart1);
}

/**
  * @brief  Electronic load set current in DAC discretes
  * @param  val - current value in DAC discretes
  * @retval none
  */
static void setCurrentInDiscretes(uint16_t val)
{
	if(loadData.on_state)
	{
		setDacValue(val);
	}
}

/**
  * @brief  Electronic load set current in amperes
  * @param  val - current value in amperes
  * @retval none
  */
static void setCurrentInAmperes(float val)
{
	uint16_t dac_val = calcCurrent2Discrete(val);
	setCurrentInDiscretes(dac_val);
}

/**
  * @brief  Electronic load set enabled state
  * @param  state: 0 - disable, 1 - enable
  * @retval none
  */
static void setEnabled(uint8_t state)
{
	loadData.on_state = state;
	if(loadData.on_state)
	{
		// reset mAh and Wh counters
		loadData.mAh = 0.0f;
		loadData.Wh = 0.0f;

		setDacValue(loadData.set_current_raw);
	}
	else
	{
		setDacValue(0);
	}
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
	uint8_t num_of_words = sizeof(CalibrationData);
	uint8_t* cd_ptr = (uint8_t*)cd;

	HAL_FLASHEx_DATAEEPROM_Unlock();

	// erase EEPROM
	flash_ok = HAL_FLASHEx_DATAEEPROM_Erase(EEPROM_CAL_DATA_ADDR);
	for(uint8_t i = 0; i < num_of_words/4+1; i++)
	{
		flash_ok = HAL_FLASHEx_DATAEEPROM_Erase(EEPROM_CAL_DATA_ADDR+4*i+4);
	}

	// save calibration coefficients
	if(flash_ok == HAL_OK)
	{
		// write signature
		flash_ok = HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, EEPROM_CAL_DATA_ADDR, IS_EEPROM_WRITTEN_SIGN);
		// write data
		for(uint8_t i = 0; i < num_of_words; i++)
		{
			flash_ok = HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, EEPROM_CAL_DATA_ADDR+i+4, cd_ptr[i]);
		}

		if(flash_ok != HAL_OK)
		{
			Error_Handler();
		}
	}

	HAL_FLASHEx_DATAEEPROM_Lock();
}

/**
  * @brief  Save load settings data in EEPROM
  * @param  ls - pointer to data structure with load settings data
  * @retval none
  */
static void saveLoadSettings(LoadSettings* ls)
{
	HAL_StatusTypeDef flash_ok = HAL_ERROR;
	uint8_t num_of_words = sizeof(LoadSettings);
	uint8_t* ls_ptr = (uint8_t*)ls;

	HAL_FLASHEx_DATAEEPROM_Unlock();

	// erase EEPROM
	flash_ok = HAL_FLASHEx_DATAEEPROM_Erase(EEPROM_LOAD_SET_ADDR);
	for(uint8_t i = 0; i < num_of_words/4+1; i++)
	{
		flash_ok = HAL_FLASHEx_DATAEEPROM_Erase(EEPROM_LOAD_SET_ADDR+4*i+4);
	}

	// save calibration coefficients
	if(flash_ok == HAL_OK)
	{
		// write signature
		flash_ok = HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, EEPROM_LOAD_SET_ADDR, IS_EEPROM_WRITTEN_SIGN);

		// write data
		for(uint8_t i = 0; i < num_of_words; i++)
		{
			flash_ok = HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE, EEPROM_LOAD_SET_ADDR+i+4, ls_ptr[i]);
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
	for(uint16_t i = 0; i < 5*NUM_OF_SAMPLES; i++)
	{
		adc_averaged_data[i%5] += adcSamples[i];
	}
	for(uint16_t i = 0; i < 5; i++)
	{
		adc_averaged_data[i] /= NUM_OF_SAMPLES;
	}
	// convert them
	loadData.vbat = 0.00168f*adc_averaged_data[0]-0.33f;
	loadData.vdac0_raw = adc_averaged_data[1];
	loadData.vref_raw = adc_averaged_data[2];
	loadData.measured_current_raw = adc_averaged_data[3];
	loadData.voltage_raw = (adc_averaged_data[4]);

	loadData.measured_current = calcCurrent2Float(adc_averaged_data[3]);
	if(loadData.measured_current < 0.0f) loadData.measured_current = 0.0f;

	loadData.voltage = calcVoltage(adc_averaged_data[4]);
	if(loadData.voltage < 0.0f) loadData.voltage = 0.0f;

	// calc mAh and Wh
	if(loadData.measured_current >= MAH_CALC_LIMIT && loadData.on_state)
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
  * @brief  Electronic load power control
  * @param  is_on: 0 power off, 1 - power on
  * @retval none
  */
static void powerControl(uint8_t is_on)
{
	__IO uint32_t* reg = is_on ? &PWR_ON_GPIO_Port->BSRR : &PWR_ON_GPIO_Port->BRR;
	*reg = PWR_ON_Pin;
}

/**
  * @brief  Check overtemperature event
  * @param  none
  * @retval 0 - no overtemperature, 1 - overtemperature
  */
static uint8_t checkOvertemperature(void)
{
	return (OVT_GPIO_Port->IDR & OVT_Pin) ? 1 : 0;
}

/**
  * @brief  Check power button state
  * @param  none
  * @retval none
  */
static void checkPowerButton(void)
{
	static uint8_t tick_cntr;
	static uint8_t button_prev_state;
	static uint8_t is_powered_on;

	if(!(PWR_BTN_GPIO_Port->IDR & PWR_BTN_Pin) && !button_prev_state && is_powered_on) // button was pressed
	{
		button_prev_state = 1;
		loadData.on_state ^= 0x01; // toggle load enabled state
		setEnabled(loadData.on_state);
	}
	else if((PWR_BTN_GPIO_Port->IDR & PWR_BTN_Pin) && (button_prev_state || !is_powered_on)) // button was released
	{
		button_prev_state = 0;
		tick_cntr = 0;
		if(!is_powered_on) is_powered_on = 1;
	}
	else if(!(PWR_BTN_GPIO_Port->IDR & PWR_BTN_Pin) && button_prev_state && is_powered_on) // button is holding on
	{
		tick_cntr++;
	}
	// if button is holding more 1.5 sec or Vbat is lower VBAT_LOW value, make power off
	if(tick_cntr == POWER_OFF_TICKS || loadData.vbat < VBAT_LOW)
	{
		setEnabled(0); // reset current
		st7565_drv->displayReset(); // reset display
		powerControl(0); // power off
	}
}

/**
  * @brief  Update DS18B20 temperature
  * @param  none
  * @retval none
  */
static void updateTemperatureValue(void)
{
	static uint8_t tick_cntr;
	if(tick_cntr++ >= TEMP_UPDATE_TICKS)
	{
		loadData.temperature = ds18b20_drv->GetTemperature();
		tick_cntr = 0;
	}
}

/**
  * @brief  Fan speed control
  * @param  none
  * @retval none
  */
static void fanSpeedControl(void)
{
	static uint16_t tick_cntr;
	static float temp_prev;
	float Kp = 5.0f, Ki = 0.1f;
	int8_t fan_speed = 0;

	if(tick_cntr++ >= FAN_SPEED_CTRL_TICKS)
	{
		tick_cntr = 0;
		if(loadData.temperature < 40.0f)
		{
			setFanSpeed(0);
		}
		else
		{
			fan_speed = (uint8_t)(Kp*(loadData.temperature-40.0f)+Ki*(loadData.temperature-temp_prev));
			if(fan_speed > 100) fan_speed = 100;
			if(fan_speed < 0) fan_speed = 0;
			setFanSpeed((uint8_t)fan_speed);
		}
		temp_prev = loadData.temperature;
	}
}

/**
  * @brief  Write current value in discretes to DAC
  * @param  val - current value in DAC discretes
  * @retval none
  */
static void setDacValue(uint16_t val)
{
	uint16_t delta_val = ABS(val - loadData.set_current_raw);
	uint16_t delta_samples = sizeof(rampVals)/2-1;
	int16_t y = 0, yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, mul = 0;

	if(val == loadData.set_current_raw) return;

	if(val > 4095) val = 4095; // limit max value by DAC 12-bit scale

	if(loadData.load_settings.load_work_mode == Ramp)
	{
		// fill ramp values array
		y = loadData.set_current_raw;

		if (val >= loadData.set_current_raw)
		{
			yinc1 = 1;
			yinc2 = 1;
		}
		else
		{
			yinc1 = -1;
			yinc2 = -1;
		}

		if (delta_samples >= delta_val)
		{
			yinc2 = 0;
			den = delta_samples;
			num = delta_samples/2;
			numadd = delta_val;
		}
		else
		{
	        mul = delta_val/delta_samples;
	        yinc1 *= mul;
	        yinc2 *= mul;
	        den = mul > 1 ? delta_val : delta_samples;
	        num = den/2;
	        numadd = delta_val%delta_samples;
		}

		for (uint16_t i = 0; i < delta_samples; i++)
		{
			rampVals[i] = y;
			num += numadd;                        /* Increase the numerator by the top of the fraction */
			if (num >= den)                       /* Check if numerator >= denominator */
			{
				num -= den;                       /* Calculate the new numerator value */
				y += yinc1;                       /* Change the y as appropriate */
			}
			y += yinc2;
		}
		rampVals[delta_samples] = val;
		// write ramp values to DAC
		HAL_DAC_Start_DMA(&hdac, DAC_CHANNEL_1, (uint32_t*)rampVals, 500, DAC_ALIGN_12B_R);
	}
	else
	{
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, val); // set default value
	}
	// update set current raw value
	loadData.set_current_raw = val;
}

/**
  * @brief  Calculate current value in amperes from ADC discretуs
  * @param  adc_val - current value in ADC discretes
  * @retval current value in amperes
  */
static float calcCurrent2Float(uint16_t adc_val)
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
  * @brief  Calculate current value in DAC discretes from amperes
  * @param  adc_val - current value in amperes
  * @retval current value in DAC discretes
  */
static uint16_t calcCurrent2Discrete(float ampere_val)
{
	uint16_t result = 0;

	if(ampere_val < 0.1f)
	{
		result = (uint16_t)(loadData.calibration_data.current_set_0A1-(0.1f-ampere_val)*(loadData.calibration_data.current_set_1A-loadData.calibration_data.current_set_0A1)/0.9f);
	}
	else if(ampere_val >= 0.1f && ampere_val < 1.0f)
	{
		result = (uint16_t)(loadData.calibration_data.current_set_0A1+(ampere_val-0.1f)*(loadData.calibration_data.current_set_1A-loadData.calibration_data.current_set_0A1)/0.9f);
	}
	else if(ampere_val >= 1.0f && ampere_val < 5.0f)
	{
		result = (uint16_t)(loadData.calibration_data.current_set_1A+(ampere_val-1.0f)*(loadData.calibration_data.current_set_5A-loadData.calibration_data.current_set_1A)/4.0f);
	}
	else
	{
		result = (uint16_t)(loadData.calibration_data.current_set_5A+(ampere_val-5.0f)*(loadData.calibration_data.current_set_5A-loadData.calibration_data.current_set_1A)/4.0f);
	}
	return result;
}

/**
  * @brief  Calculate voltage value in volts from ADC discretes
  * @param  adc_val - voltage value in ADC discretes
  * @retval voltage value in volts
  */
static float calcVoltage(uint16_t adc_val)
{
	float voltage = 0.0f;

	if(adc_val < loadData.calibration_data.voltage_2V)
	{
		voltage = 2.0f - 8.0f*(loadData.calibration_data.voltage_2V - adc_val)/(loadData.calibration_data.voltage_10V-loadData.calibration_data.voltage_2V);
	}
	else if(adc_val >= loadData.calibration_data.voltage_2V && adc_val < loadData.calibration_data.voltage_10V)
	{
		voltage = 2.0f + 8.0f*(adc_val - loadData.calibration_data.voltage_2V)/(loadData.calibration_data.voltage_10V-loadData.calibration_data.voltage_2V);
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
	if(htim->Instance == TIM22)
	{
		loadData.is_update_event = 1;
	}
}


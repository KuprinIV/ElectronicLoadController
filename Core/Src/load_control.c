/*
 * load_control.c
 *
 *  Created on: 6 сент. 2023 г.
 *      Author: Kuprin_IV
 */
#include "load_control.h"

static uint16_t encoder_cntr = 32767;
static uint16_t encoder_cntr_prev = 32767;

extern Data loadData;

int16_t getEncoderOffset(void)
{
	int16_t offset = 0;
	if(encoder_cntr != encoder_cntr_prev)
	{
		offset = (int16_t)(encoder_cntr - encoder_cntr_prev);
		encoder_cntr_prev = encoder_cntr;
	}
	return offset;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		if(GPIOA->IDR & GPIO_PIN_1)
		{
			encoder_cntr++;
		}
		else
		{
			encoder_cntr--;
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM22)
	{
		loadData.is_update_event = 1;
	}
}


/*
 * one_wire_uart.c
 *
 *  Created on: 8 сент. 2023 г.
 *      Author: Kuprin_IV
 */

#include "one_wire_uart.h"

/* Functions -----------------------------------------------------------------*/
static ONEWIRE_Status OneWire_Reset(UART_HandleTypeDef *huart);
static uint8_t OneWire_ProcessByte(UART_HandleTypeDef *huart, uint8_t byte);
static uint8_t OneWire_ProcessBit(UART_HandleTypeDef *huart, uint8_t bit);

OneWireDriver one_wire_driver = {
		OneWire_Reset,
		OneWire_ProcessByte,
		OneWire_ProcessBit
};

OneWireDriver* one_wire_drv = &one_wire_driver;

/**
  * @brief  Set UART baudrate for 1-wire reset and process bytes
  * @param  huart - UART handle
  * @param  baudrate - desired baudrate value
  * @retval none
  */
static void SetBaudrate(UART_HandleTypeDef *huart, uint32_t baudrate)
{
	uint32_t pclk = 0;
	huart->Init.BaudRate = baudrate;
	#if defined(USART6) && defined(UART9) && defined(UART10)
	if ((huart->Instance == USART1) || (huart->Instance == USART6) ||
	(huart->Instance == UART9)  || (huart->Instance == UART10))
	{
		pclk = HAL_RCC_GetPCLK2Freq();
	}
	#elif defined(USART6)
	if ((huart->Instance == USART1) || (huart->Instance == USART6))
	{
		pclk = HAL_RCC_GetPCLK2Freq();
	}
	#else
	if (huart->Instance == USART1)
	{
		pclk = HAL_RCC_GetPCLK2Freq();
	}
	#endif /* USART6 */
	else
	{
		pclk = HAL_RCC_GetPCLK1Freq();
	}
	if (huart->Init.OverSampling == UART_OVERSAMPLING_8)
	{
		huart->Instance->BRR = UART_DIV_SAMPLING8(pclk, huart->Init.BaudRate);
	}
	else
	{
		huart->Instance->BRR = UART_DIV_SAMPLING16(pclk, huart->Init.BaudRate);
	}
}

/**
  * @brief  Make a reset of 1-wire bus
  * @param  huart - UART handle
  * @retval ONEWIRE_OK - result is successful, ONEWIRE_ERROR - error is occured
  */
static ONEWIRE_Status OneWire_Reset(UART_HandleTypeDef *huart)
{
	ONEWIRE_Status status = ONEWIRE_OK;
	uint8_t txByte = ONEWIRE_RESET_BYTE;
	uint8_t rxByte = 0x00;
	SetBaudrate(huart, ONEWIRE_RESET_BAUDRATE);
	HAL_UART_Transmit(huart, &txByte, 1, ONEWIRE_UART_TIMEOUT);
	HAL_UART_Receive(huart, &rxByte, 1, ONEWIRE_UART_TIMEOUT);
	SetBaudrate(huart, ONEWIRE_BAUDRATE);
	if (rxByte == txByte)
	{
		status = ONEWIRE_ERROR;
	}
	return status;
}

/**
  * @brief  Make a 1-byte transfer by 1-wire
  * @param  huart - UART handle
  * @param  bit - transfered byte value
  * @retval received byte
  */
static uint8_t OneWire_ProcessByte(UART_HandleTypeDef *huart, uint8_t byte)
{
	uint8_t rxByte = 0x00;
	for (uint8_t i = 0; i < ONEWIRE_BITS_NUM; i++)
	{
		uint8_t txBit = (byte >> i) & 0x01;
		uint8_t rxBit = 0;
		uint8_t tempRxData = OneWire_ProcessBit(huart, txBit);
		if (tempRxData == 0xFF)
		{
			rxBit = 1;
		}
		rxByte |= (rxBit << i);
	}
	return rxByte;
}

/**
  * @brief  Make a 1-bit transfer by 1-wire
  * @param  huart - UART handle
  * @param  bit - transfered bit value (0 or 1)
  * @retval received bit
  */
static uint8_t OneWire_ProcessBit(UART_HandleTypeDef *huart, uint8_t bit)
{
	uint8_t txData = 0xFF;
	uint8_t rxData = 0x00;
	if (bit == 0)
	{
		txData = 0x00;
	}
	HAL_UART_Transmit(huart, &txData, 1, ONEWIRE_UART_TIMEOUT);
	HAL_UART_Receive(huart, &rxData, 1, ONEWIRE_UART_TIMEOUT);
	return rxData;
}

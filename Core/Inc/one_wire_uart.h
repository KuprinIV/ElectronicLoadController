/*
 * one_wire_uart.h
 *
 *  Created on: 8 сент. 2023 г.
 *      Author: Kuprin_IV
 */

#ifndef INC_ONE_WIRE_UART_H_
#define INC_ONE_WIRE_UART_H_

#include "main.h"

/* Declarations and definitions ----------------------------------------------*/
#define ONEWIRE_BAUDRATE                                              115200
#define ONEWIRE_RESET_BAUDRATE                                        9600
#define ONEWIRE_RESET_BYTE                                            0xF0
#define ONEWIRE_UART_TIMEOUT                                          10
#define ONEWIRE_BITS_NUM                                              8

typedef enum
{
  ONEWIRE_OK              = 0x00,
  ONEWIRE_ERROR           = 0x01,
} ONEWIRE_Status;

typedef struct
{
	ONEWIRE_Status (*reset)(UART_HandleTypeDef *huart);
	uint8_t (*processByte)(UART_HandleTypeDef *huart, uint8_t byte);
	uint8_t (*processBit)(UART_HandleTypeDef *huart, uint8_t bit);
}OneWireDriver;

extern OneWireDriver* one_wire_drv;

#endif /* INC_ONE_WIRE_UART_H_ */

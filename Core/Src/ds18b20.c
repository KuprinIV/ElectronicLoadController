/*
 * ds18b20.c
 *
 *  Created on: 8 сент. 2023 г.
 *      Author: Kuprin_IV
 */

#include "ds18b20.h"
#include "one_wire_uart.h"

/* Declarations and definitions ----------------------------------------------*/
// ROM commands
static DS18B20_Command readRom = {.code = 0x33, .rxBytesNum = 8, .txBytesNum = 0};
static DS18B20_Command skipRom = {.code = 0xCC, .rxBytesNum = 0, .txBytesNum = 0};
// Function commands
static DS18B20_Command readScratchpad = {.code = 0xBE, .rxBytesNum = 9, .txBytesNum = 0};
static DS18B20_Command writeScratchpad = {.code = 0x4E, .rxBytesNum = 0, .txBytesNum = 3};
static DS18B20_Command convertT = {.code = 0x44, .rxBytesNum = 0, .txBytesNum = 0};

// driver functions
static void DS18B20_Initialization(UART_HandleTypeDef* huart);
static float DS18B20_GetTemperature(void);

/* Functions -----------------------------------------------------------------*/
static uint8_t WaitForConversionFinished(DS18B20 *sensor);
static DS18B20_Status DS18B20_ConvertT(DS18B20 *sensor, DS18B20_WaitCondition waitCondition);
static DS18B20_Status DS18B20_ReadScratchpad(DS18B20 *sensor);
static DS18B20_Status DS18B20_WriteScratchpad(DS18B20 *sensor, uint8_t *data);
static DS18B20_Status DS18B20_InitializationCommand(DS18B20 *sensor);
static DS18B20_Status DS18B20_ReadRom(DS18B20 *sensor);
static DS18B20_Status DS18B20_SkipRom(DS18B20 *sensor);
static void DS18B20_Init(DS18B20 *sensor, UART_HandleTypeDef *huart);

// private variables
DS18B20_Driver ds18b20_driver = {
		DS18B20_Initialization,
		DS18B20_GetTemperature
};
DS18B20_Driver* ds18b20_drv = &ds18b20_driver;

static DS18B20 temperatureSensor;
static DS18B20_ConversionState conversion_state = DS18B20_START_NEXT_CONV;

static void DS18B20_Initialization(UART_HandleTypeDef* huart)
{
	DS18B20_Init(&temperatureSensor, huart);

	DS18B20_InitializationCommand(&temperatureSensor);
	DS18B20_ReadRom(&temperatureSensor);
	DS18B20_ReadScratchpad(&temperatureSensor);

	uint8_t settings[3];
	settings[0] = temperatureSensor.temperatureLimitHigh;
	settings[1] = temperatureSensor.temperatureLimitLow;
	settings[2] = DS18B20_12_BITS_CONFIG; // set 0,5 °C resolution

	DS18B20_InitializationCommand(&temperatureSensor);
	DS18B20_SkipRom(&temperatureSensor);
	DS18B20_WriteScratchpad(&temperatureSensor, settings);
}

static float DS18B20_GetTemperature(void)
{
	static float temp;

	switch(conversion_state)
	{
		case DS18B20_START_NEXT_CONV:
		    DS18B20_InitializationCommand(&temperatureSensor);
		    DS18B20_SkipRom(&temperatureSensor);
		    DS18B20_ConvertT(&temperatureSensor, DS18B20_DATA);
		    conversion_state = DS18B20_WAIT_CONV_END;
			break;

		case DS18B20_WAIT_CONV_END:
			if(WaitForConversionFinished(&temperatureSensor))
			{
			    DS18B20_InitializationCommand(&temperatureSensor);
			    DS18B20_SkipRom(&temperatureSensor);
			    DS18B20_ReadScratchpad(&temperatureSensor);

			    temp = temperatureSensor.temperature;

			    conversion_state = DS18B20_START_NEXT_CONV;
			}
			break;
	}

	return temp;
}

/**
  * @brief  Calculate transfer data checksum
  * @param  data - transfer data array
  * @param  length - transfer data array length
  * @retval Checksum value (1 byte)
  */
static uint8_t CalculateChecksum(uint8_t *data, uint8_t length)
{
  uint8_t checksum = 0;
  while (length--)
  {
    uint8_t currentByte = *data++;
    for (uint8_t i = 8; i; i--)
    {
      uint8_t temp = (checksum ^ currentByte) & 0x01;
      checksum >>= 1;
      if (temp)
      {
        checksum ^= 0x8C;
      }
      currentByte >>= 1;
    }
  }
  return checksum;
}

/**
  * @brief  Send command to DS18B20
  * @param  sensor - DS18B20 parameters data structure
  * @param  command - command code
  * @param  data - data received from sensor after command executing
  * @retval DS18B20_OK - result is successful, DS18B20_ERROR - error is occured
  */
static DS18B20_Status ExecuteCommand(DS18B20 *sensor, DS18B20_Command command, uint8_t *data)
{
  if (sensor->isConnected == 0)
  {
    return DS18B20_ERROR;
  }
  one_wire_drv->processByte(sensor->uart, command.code);
  if (command.rxBytesNum != 0)
  {
    for (uint8_t i = 0; i < command.rxBytesNum; i++)
    {
      data[i] = one_wire_drv->processByte(sensor->uart, 0xFF);
    }
    uint8_t checkSum = CalculateChecksum(data, command.rxBytesNum - 1);
    if (checkSum != data[command.rxBytesNum - 1])
    {
      return DS18B20_ERROR;
    }
  }
  else
  {
    for (uint8_t i = 0; i < command.txBytesNum; i++)
    {
      one_wire_drv->processByte(sensor->uart, data[i]);
    }
  }
  return DS18B20_OK;
}

/**
  * @brief  Waut temperature data conversion
  * @param  sensor - DS18B20 parameters data structure
  * @retval none
  */
static uint8_t WaitForConversionFinished(DS18B20 *sensor)
{
  uint8_t data = one_wire_drv->processBit(sensor->uart, 1);
  return (data == 0xFF);
//  while(data != 0xFF)
//  {
//    data = one_wire_drv->processBit(sensor->uart, 1);
//  }
}

/**
  * @brief  Send command ConvertT to DS18B20 for starting temperature conversion
  * @param  sensor - DS18B20 parameters data structure
  * @param  waitCondition: DS18B20_DATA - check end of conversion by busy state on bus,
  *  DS18B20_DELAY - wait fixed delay in ms
  * @retval DS18B20_OK - result is successful, DS18B20_ERROR - error is occured
  */
static DS18B20_Status DS18B20_ConvertT(DS18B20 *sensor, DS18B20_WaitCondition waitCondition)
{
  DS18B20_Status result;
  uint8_t rxDummyData;
  result = ExecuteCommand(sensor, convertT, &rxDummyData);
  if (waitCondition == DS18B20_DATA)
  {
    WaitForConversionFinished(sensor);
  }
  if (waitCondition == DS18B20_DELAY)
  {
    uint32_t delayValueMs = 0;
    switch (sensor->configRegister)
    {
      case DS18B20_9_BITS_CONFIG:
        delayValueMs = DS18B20_9_BITS_DELAY_MS;
        break;
      case DS18B20_10_BITS_CONFIG:
        delayValueMs = DS18B20_10_BITS_DELAY_MS;
        break;
      case DS18B20_11_BITS_CONFIG:
        delayValueMs = DS18B20_11_BITS_DELAY_MS;
        break;
      case DS18B20_12_BITS_CONFIG:
        delayValueMs = DS18B20_12_BITS_DELAY_MS;
        break;
      default:
        break;
    }
    HAL_Delay(delayValueMs);
  }
  return result;
}

/**
  * @brief  Send command ReadScatchpad to DS18B20 for reading temperature conversion results
  * @param  sensor - DS18B20 parameters data structure
  * @retval DS18B20_OK - result is successful, DS18B20_ERROR - error is occured
  */
static DS18B20_Status DS18B20_ReadScratchpad(DS18B20 *sensor)
{
  DS18B20_Status result;
  uint8_t rxData[DS18B20_READ_SCRATCHPAD_RX_BYTES_NUM];
  result = ExecuteCommand(sensor, readScratchpad, rxData);
  if (result != DS18B20_OK)
  {
    return result;
  }
  sensor->temperatureLimitHigh = rxData[DS18B20_SCRATCHPAD_T_LIMIT_H_BYTE_IDX];
  sensor->temperatureLimitLow = rxData[DS18B20_SCRATCHPAD_T_LIMIT_L_BYTE_IDX];
  sensor->configRegister = rxData[DS18B20_SCRATCHPAD_CONFIG_BYTE_IDX];
  uint16_t tRegValue = (rxData[DS18B20_SCRATCHPAD_T_MSB_BYTE_IDX] << 8) | rxData[DS18B20_SCRATCHPAD_T_LSB_BYTE_IDX];
  uint16_t sign = tRegValue & DS18B20_SIGN_MASK;
  if (sign != 0)
  {
    tRegValue = (0xFFFF - tRegValue + 1);
  }
  switch (sensor->configRegister)
  {
    case DS18B20_9_BITS_CONFIG:
      tRegValue &= DS18B20_9_BITS_DATA_MASK;
      break;
    case DS18B20_10_BITS_CONFIG:
      tRegValue &= DS18B20_10_BITS_DATA_MASK;
      break;
    case DS18B20_11_BITS_CONFIG:
      tRegValue &= DS18B20_11_BITS_DATA_MASK;
      break;
    case DS18B20_12_BITS_CONFIG:
      tRegValue &= DS18B20_12_BITS_DATA_MASK;
      break;
    default:
      tRegValue &= DS18B20_12_BITS_DATA_MASK;
      break;
  }
  sensor->temperature = (float)tRegValue * DS18B20_T_STEP;
  if (sign != 0)
  {
    sensor->temperature *= (-1);
  }
  return DS18B20_OK;
}

/**
  * @brief  Send command WriteScatchpad to DS18B20 to set sensor settings
  * @param  sensor - DS18B20 parameters data structure
  * @param  data - sensor settings data
  * @retval DS18B20_OK - result is successful, DS18B20_ERROR - error is occured
  */
static DS18B20_Status DS18B20_WriteScratchpad(DS18B20 *sensor, uint8_t *data)
{
  DS18B20_Status result;
  result = ExecuteCommand(sensor, writeScratchpad, data);
  if (result != DS18B20_OK)
  {
    return result;
  }
  sensor->temperatureLimitHigh = data[0];
  sensor->temperatureLimitLow = data[1];
  sensor->configRegister = data[2];
  return result;
}

/**
  * @brief  Set reset state on 1-wire bus to check DS18B20 presence
  * @param  sensor - DS18B20 parameters data structure
  * @retval DS18B20_OK - result is successful, DS18B20_ERROR - error is occured
  */
static DS18B20_Status DS18B20_InitializationCommand(DS18B20 *sensor)
{
  if (sensor->isInitialized == 0)
  {
    return DS18B20_ERROR;
  }
  ONEWIRE_Status status = one_wire_drv->reset(sensor->uart);
  if (status == ONEWIRE_OK)
  {
    sensor->isConnected = 1;
    return DS18B20_OK;
  }
  else
  {
    sensor->isConnected = 0;
    return DS18B20_ERROR;
  }
}

/**
  * @brief  Send command ReadROM to read DS18B20 family code, serial number and CRC
  * @param  sensor - DS18B20 parameters data structure
  * @retval DS18B20_OK - result is successful, DS18B20_ERROR - error is occured
  */
static DS18B20_Status DS18B20_ReadRom(DS18B20 *sensor)
{
  DS18B20_Status result;
  uint8_t rxData[DS18B20_READ_ROM_RX_BYTES_NUM];
  result = ExecuteCommand(sensor, readRom, rxData);
  if (result != DS18B20_OK)
  {
    return result;
  }
  for (uint8_t i = 0; i < DS18B20_SERIAL_NUMBER_LEN_BYTES; i++)
  {
    sensor->serialNumber[i] = rxData[DS18B20_SERIAL_NUMBER_OFFSET_BYTES + i];
  }
  return DS18B20_OK;
}

/**
  * @brief  Send command SkipROM to skip DS18B20 ROM read operation
  * @param  sensor - DS18B20 parameters data structure
  * @retval DS18B20_OK - result is successful, DS18B20_ERROR - error is occured
  */
static DS18B20_Status DS18B20_SkipRom(DS18B20 *sensor)
{
  DS18B20_Status result;
  uint8_t rxDummyData;
  result = ExecuteCommand(sensor, skipRom, &rxDummyData);
  if (result != DS18B20_OK)
  {
    return result;
  }
  return DS18B20_OK;
}

/**
  * @brief  Initialize sensor parameter data structure and UART handle
  * @param  sensor - DS18B20 parameters data structure
  * @param  huart - UART handle
  * @retval none
  */
static void DS18B20_Init(DS18B20 *sensor, UART_HandleTypeDef *huart)
{
  sensor->isConnected = 0;
  sensor->uart = huart;
  sensor->isInitialized = 1;
}

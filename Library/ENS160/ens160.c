/**
 * @file: ens160.c
 * @Created on: 10.03.2024
 * @Author: rRaufToprak
 * @Description: ENS160 gas sensor driver source file
 * @Copyright: MIT License
*/

#include "ens160.h"

HAL_StatusTypeDef ENS160_Init(ENS160 *dev, I2C_HandleTypeDef *i2cHandle)
{
	dev->devID = 0;
	dev->status = 0;
	dev->opMode = STANDARD_MODE;
	dev->data.AQI_data = 0u;
	dev->data.ECO2_data = 0u;
	dev->data.TVOC_data= 0u;
	dev->i2cHandle = i2cHandle;

	HAL_StatusTypeDef retStatus = ENS160_Control_ID(dev);

	if(dev->devID == 0x160)
	{
		retStatus = ENS160_SetOpMode(dev, dev->opMode);
	}

	return retStatus;
}

HAL_StatusTypeDef ENS160_Control_ID(ENS160 *dev)
{
	uint8_t buffer[2];
	uint16_t sensorID = 0;

	HAL_StatusTypeDef id_status = HAL_I2C_Mem_Read(dev->i2cHandle, 0x53<<1, ENS160_PART_ID, I2C_MEMADD_SIZE_8BIT, buffer, 2, 1000);

	sensorID = ((uint16_t)buffer[1]<<8)| buffer[0];
	dev->devID = sensorID;

	return id_status;
}
HAL_StatusTypeDef ENS160_Write_Register(ENS160 *dev, uint8_t register_address, uint8_t data)
{
	HAL_StatusTypeDef status;

	status = HAL_I2C_Mem_Write(dev->i2cHandle, 0x53<<1, register_address, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);

	return status;
}
uint8_t ENS160_Read_Single_Register(ENS160 *dev, uint8_t register_address)
{
	HAL_StatusTypeDef status;
	uint8_t value;
	
	status = HAL_I2C_Mem_Read(dev->i2cHandle, 0x53<<1, register_address, I2C_MEMADD_SIZE_8BIT, &value, 1, 1000);
	
	if(status == HAL_OK)
		return value;
	else
		return 0;
	
}
uint16_t ENS160_Read_Register(ENS160 *dev, uint8_t register_address)
{
	uint8_t data[2];
	
	HAL_I2C_Mem_Read(dev->i2cHandle, 0x53<<1, register_address, I2C_MEMADD_SIZE_8BIT, data, 2, 1000);
	
	uint16_t raw_value = ((data[1]<<8)| data[0]);

	return raw_value;
}


HAL_StatusTypeDef ENS160_SetOpMode(ENS160 *dev, uint8_t command)
{
	HAL_StatusTypeDef status;
	
	status = HAL_I2C_Mem_Write(dev->i2cHandle, 0x53<<1, ENS160_OPMODE, I2C_MEMADD_SIZE_8BIT, &command, 1, 1000);

	return status;
}
HAL_StatusTypeDef ENS160_Read_Datas(ENS160 *dev)
{
	HAL_StatusTypeDef status = HAL_OK;

	dev->data.AQI_data 	= ENS160_Read_Single_Register(dev, ENS160_DATA_AQI);
	dev->data.ECO2_data =	ENS160_Read_Register(dev, ENS160_DATA_ECO2);
	dev->data.TVOC_data = ENS160_Read_Register(dev, ENS160_DATA_TVOC);

	return status;
}
HAL_StatusTypeDef ENS160_Update_Comp_Val(ENS160 *dev, uint16_t temp, uint16_t hum)
{
	HAL_StatusTypeDef status;
	uint8_t data_buffer[2];
	temp = (temp + 273.15f) * 64;
	hum = hum * 512;
	
	data_buffer[0] = (uint8_t)temp & 0xFF;
	data_buffer[1] = ((uint16_t)temp >> 8 & 0xFF);
	status = HAL_I2C_Mem_Write(dev->i2cHandle, 0x53<<1, ENS160_TEMP_IN, I2C_MEMADD_SIZE_8BIT, data_buffer, 2, 1000);
	
	data_buffer[0] = (uint8_t)hum & 0xFF;
	data_buffer[1] = ((uint16_t)hum >> 8 & 0xFF);

	status = HAL_I2C_Mem_Write(dev->i2cHandle, 0x53<<1, ENS160_RH_IN, I2C_MEMADD_SIZE_8BIT, data_buffer, 2, 1000);
	return status;
}

HAL_StatusTypeDef ENS160_Read_DataStatus(ENS160 *dev)
{
	HAL_StatusTypeDef status = HAL_OK;

	dev->status = ENS160_Read_Single_Register(dev, ENS160_DEV_STATUS);

	return status;
}


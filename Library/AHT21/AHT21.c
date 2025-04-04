/*
 * AHT21.c
 *
 *  Created on: Nov 23, 2023
 *      Author: lucas
 */

#include "AHT21.h"



/* Initialization ------------------------------------------------------------*/

/**
  * @brief	Initializes the sensor.
  * @param	dev Pointer to a AHT21 structure that contains the configuration
  *             information for the specified AHT21.
  * @param	i2cHandle Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval HAL status.
  */
HAL_StatusTypeDef AHT21_Init(AHT21 *dev, I2C_HandleTypeDef *i2cHandle){
	dev->status = 0;
	dev->opMode = AHT21_MODE_NORMAL;
	dev->temp_C = 0.0f;
	dev->humid_100 = 0.0f;
	dev->i2cHandle = i2cHandle;

	HAL_StatusTypeDef retStatus = AHT21_GetStatus(dev);

	if ((dev->status & 0x08) == 0){
		uint8_t AHT21_i2c_tx[3];

		AHT21_i2c_tx[0] = AHT21_I2C_CMD_INIT;
		AHT21_i2c_tx[1] = AHT21_ENABLE_CAL;
		AHT21_i2c_tx[2] = AHT21_MODE_NORMAL;

		retStatus = HAL_I2C_Master_Transmit(dev->i2cHandle, AHT21_I2C_ADDR, (uint8_t *)AHT21_i2c_tx, 3, 100);
	}

	return retStatus;
}



/* Data Acquisition ------------------------------------------------------------*/

/**
  * @brief	Send command and parameters to request a new measurement.
  * @note	Measurement values are ready after 75ms.
  * @param	dev Pointer to a AHT21 structure that contains the configuration
  *             information for the specified AHT21.
  * @retval HAL status.
  */
HAL_StatusTypeDef AHT21_RequestMeasurement(AHT21 *dev){
	uint8_t AHT21_i2c_tx[3];

	AHT21_i2c_tx[0] = AHT21_I2C_CMD_START_MEAS;
	AHT21_i2c_tx[1] = AHT21_I2C_CMD_DATA_MEAS;
	AHT21_i2c_tx[2] = AHT21_I2C_CMD_DATA_NOP;

	return HAL_I2C_Master_Transmit(dev->i2cHandle, AHT21_I2C_ADDR, (uint8_t *)AHT21_i2c_tx, 3, 100);
}

/**
  * @brief	Read the measurement data.
  * @note	Must have previously sent a measurement request.
  * @param	dev Pointer to a AHT21 structure that contains the configuration
  *             information for the specified AHT21.
  * @retval HAL status.
  */
HAL_StatusTypeDef AHT21_ReadTempHumid(AHT21 *dev){
	uint8_t AHT21_i2c_rawData[7];
	uint32_t AHT21_i2c_rx;

	HAL_StatusTypeDef retStatus = HAL_I2C_Master_Receive(dev->i2cHandle, (AHT21_I2C_ADDR | 0x01), AHT21_i2c_rawData, 7, 100);

	if (retStatus == HAL_OK){
		dev->status = AHT21_i2c_rawData[0];

		AHT21_i2c_rx = ((uint32_t)AHT21_i2c_rawData[1] << 12) | ((uint32_t)AHT21_i2c_rawData[2] << 4) | (AHT21_i2c_rawData[3] >> 4);
		dev->humid_100 = (float) (AHT21_i2c_rx / 1048576.0) * 100.0;

		AHT21_i2c_rx = (((uint32_t)AHT21_i2c_rawData[3] & 0xF) << 16) | ((uint32_t)AHT21_i2c_rawData[4] << 8) | AHT21_i2c_rawData[5];;
		dev->temp_C = (float) ((AHT21_i2c_rx / 1048576.0) * 200.0) - 50.0;
	}

	return retStatus;
}

/**
  * @brief	Send command and parameters to request a new measurement, wait
  * 		75ms and read the measurement data.
  * @param	dev Pointer to a AHT21 structure that contains the configuration
  *             information for the specified AHT21.
  * @retval HAL status.
  */
HAL_StatusTypeDef AHT21_RequestAndReadTempHumid(AHT21 *dev){
	HAL_StatusTypeDef retStatus = AHT21_RequestMeasurement(dev);

	if (retStatus != HAL_OK){
		return retStatus;
	}

	HAL_Delay(AHT21_DELAY_MEASURMENT);

	return AHT21_ReadTempHumid(dev);
}

/**
  * @brief	Get sensor status byte.
  * @note	Bit[7]:   Busy (1) or not busy (0).
  *			Bit[6:5]: Operation Mode (00 = Normal, 01 = Cycle, 1x = CMD).
  *			Bit[4]:   Reserved.
  *			Bit[3]:   Calibration enabled (1) or disabled (0).
  *			Bit[2:0]: Reserved.
  * @param	dev Pointer to a AHT21 structure that contains the configuration
  *             information for the specified AHT21.
  * @retval HAL status.
  */
HAL_StatusTypeDef AHT21_GetStatus(AHT21 *dev){
	uint8_t AHT21_i2c_rawData[7];

	HAL_StatusTypeDef retStatus = HAL_I2C_Master_Receive(dev->i2cHandle, (AHT21_I2C_ADDR | 0x01), AHT21_i2c_rawData, 7, 100);

	dev->status = AHT21_i2c_rawData[0];

	return retStatus;
}



/* Operation ------------------------------------------------------------*/

/**
  * @brief	Send soft reset command.
  * @param	dev Pointer to a AHT21 structure that contains the configuration
  *             information for the specified AHT21.
  * @retval HAL status.
  */
HAL_StatusTypeDef AHT21_SoftReset(AHT21 *dev){
	uint8_t AHT21_i2c_tx = AHT21_I2C_CMD_SRST;
	return HAL_I2C_Master_Transmit(dev->i2cHandle, AHT21_I2C_ADDR, &AHT21_i2c_tx, 1, 100);
}

/**
  * @brief	Change Operation Mode.
  * @param	dev Pointer to a AHT21 structure that contains the configuration
  *             information for the specified AHT21.
  * @param	mode Desired Operation Mode (00 = Normal, 01 = Cycle, 1x = CMD).
  * @retval HAL status.
  */
HAL_StatusTypeDef AHT21_SetMode(AHT21 *dev, uint8_t mode){
	dev->opMode = mode;

	uint8_t AHT21_i2c_tx[3];

	if (mode == 0){
		AHT21_i2c_tx[0] = AHT21_I2C_CMD_NORMAL;
		AHT21_i2c_tx[1] = AHT21_I2C_CMD_DATA_NOP;
		AHT21_i2c_tx[2] = AHT21_I2C_CMD_DATA_NOP;
	}
	else{
		AHT21_i2c_tx[0] = AHT21_I2C_CMD_INIT;
		AHT21_i2c_tx[1] = AHT21_MODE_CYCLE | AHT21_ENABLE_CAL;
		AHT21_i2c_tx[2] = AHT21_I2C_CMD_DATA_NOP;
	}

	return HAL_I2C_Master_Transmit(dev->i2cHandle, AHT21_I2C_ADDR, (uint8_t *)AHT21_i2c_tx, 3, 100);
}

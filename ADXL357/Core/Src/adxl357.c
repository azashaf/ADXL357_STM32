#include "adxl357.h"

uint8_t ADXL357_INIT(ADXL357_t *dev, I2C_HandleTypeDef *i2cHandle, ADXL357_FS_t range, ADXL357_DR_t val)
{
	dev->i2cHandle = i2cHandle;
	dev->acc_mps2[0] = 0.0f;
	dev->acc_mps2[1] = 0.0f;
	dev->acc_mps2[2] = 0.0f;
	
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;
	
	uint8_t regData;
	
	status = ADXL357_ReadRegister(dev, ADXL357_REG_DEVID_AD, &regData); // This register contains the Analog Devices ID, 0xAD
	errNum += (status != HAL_OK);
	
	if (regData != ADXL357_DEVICE_ID)
	{
		return 1;
	}
	
	status = ADXL357_ReadRegister(dev, ADXL357_REG_DEVID_MST, &regData); // This register contains the Analog Devices MEMS ID, 0x1D
	errNum += (status != HAL_OK);
	
	if (regData != ADXL357_MEMS_ID)
	{
		return 2;
	}

	status = ADXL357_ReadRegister(dev, ADXL357_REG_PARTID, &regData); // This register contains the device ID, 0xED
	errNum += (status != HAL_OK);
	
	if (regData != ADXL357_PART_ID)
	{
		return 3;
	}	
	
	regData = (uint8_t)val;
	status = ADXL357_WriteRegister(dev, ADXL357_REG_FILTER, &regData);
	errNum += (status != HAL_OK);
	
	regData = 0x00;	
	status = ADXL357_WriteRegister(dev, ADXL357_REG_POWER_CTL, &regData); // measurement mode
	errNum += (status != HAL_OK);	

	switch (range)
	{
		case ADXL357_10g: 
			dev->Scale_Factor = 0.0000195f;
			break;
		
		case ADXL357_20g: 
			dev->Scale_Factor = 0.000039f;
			break;
		
		case ADXL357_40g: 
			dev->Scale_Factor = 0.000078f;
			break;
		
		default:	
			dev->Scale_Factor = 0.000039f;
			break;
	}	
	
	regData = (uint8_t)range;
	status = ADXL357_WriteRegister(dev, ADXL357_REG_RANGE, &regData); 
	errNum += (status != HAL_OK);	
	
	return errNum;
}


HAL_StatusTypeDef ADXL357_ReadRegister(ADXL357_t *dev, uint8_t reg, uint8_t *data)
{
	return HAL_I2C_Mem_Read (dev->i2cHandle, ADXL357_DEF_ADD, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef ADXL357_ReadRegisters(ADXL357_t *dev, uint8_t reg, uint8_t *data, uint8_t len)
{
	return HAL_I2C_Mem_Read (dev->i2cHandle, ADXL357_DEF_ADD, reg, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY);
}

HAL_StatusTypeDef ADXL357_WriteRegister(ADXL357_t *dev, uint8_t reg, uint8_t *data)
{
	return HAL_I2C_Mem_Write (dev->i2cHandle, ADXL357_DEF_ADD, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef ADXL357_ReadAcc(ADXL357_t *dev)
{
	uint8_t regData[9];
	float Acceleration_of_gravity = 9.81f;
	
	HAL_StatusTypeDef status = ADXL357_ReadRegisters(dev, ADXL357_REG_XDATA3, regData, 9);
	
	volatile uint32_t accRawSigned[3];
	accRawSigned[0] = ((( regData[0] << 24) | ( regData[1] << 16) | ( (regData[2] & 0xF0)<<8))>>12);
	accRawSigned[1] = ((( regData[3] << 24) | ( regData[4] << 16) | ( (regData[5] & 0xF0)<<8))>>12);	
	accRawSigned[2] = ((( regData[6] << 24) | ( regData[7] << 16) | ( (regData[8] & 0xF0)<<8))>>12);		

	accRawSigned[0] = (accRawSigned[0] & 0x80000) == 0x80000 ? -accRawSigned[0] : accRawSigned[0];
	accRawSigned[1] = (accRawSigned[1] & 0x80000) == 0x80000 ? -accRawSigned[1] : accRawSigned[1];
	accRawSigned[2] = (accRawSigned[2] & 0x80000) == 0x80000 ? -accRawSigned[2] : accRawSigned[2];
	
	dev->acc_mps2[0] = Acceleration_of_gravity * dev->Scale_Factor * accRawSigned[0];
	dev->acc_mps2[1] = Acceleration_of_gravity * dev->Scale_Factor * accRawSigned[1];
	dev->acc_mps2[2] = Acceleration_of_gravity * dev->Scale_Factor * accRawSigned[2];

	return status;
}


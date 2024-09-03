#ifndef __ADXL_357__
#define __ADXL_357__

#include "main.h"

////    Defines    ////
// I2C settings
//RESET
#define ADXL357_DEVICE_ID						0xAD
#define ADXL357_MEMS_ID							0x1D
#define ADXL357_PART_ID							0xED

// Device adresses
#define ADXL357_DEF_ADD             (0x1D << 1)    // When MISO is set to 0
#define ADXL357_ALT_ADD             0x53    // When MISO is set to 1

// Ranges                                   // Datasheet P.39
#define ADXL357_TEN_G               0b01
#define ADXL357_TWENTY_G            0b10
#define ADXL357_FOUTY_G             0b11

// Power modes
#define ADXL357_DRDY_OFF            0x04
#define ADXL357_TEMP_OFF            0x02
#define ADXL357_STANDBY             0x01
#define ADXL357_ALL_ON              0x00

// Register addresses                       // Datasheet P.32
#define ADXL357_REG_DEVID_AD        0x00    
#define ADXL357_REG_DEVID_MST       0x01    
#define ADXL357_REG_PARTID          0x02
#define ADXL357_REG_REVID           0x03
#define ADXL357_REG_STATUS          0x04
#define ADXL357_REG_FIFO_ENTRIES    0x05
#define ADXL357_REG_TEMP2           0x06
#define ADXL357_REG_TEMP1           0x07
#define ADXL357_REG_XDATA3          0x08
#define ADXL357_REG_XDATA2          0x09
#define ADXL357_REG_XDATA1          0x0A
#define ADXL357_REG_YDATA3          0x0B
#define ADXL357_REG_YDATA2          0x0C
#define ADXL357_REG_YDATA1          0x0D
#define ADXL357_REG_ZDATA3          0x0E
#define ADXL357_REG_ZDATA2          0x0F
#define ADXL357_REG_ZDATA1          0x10
#define ADXL357_REG_FIFO_DATA       0x11
#define ADXL357_REG_OFFSET_X_H      0x1E
#define ADXL357_REG_OFFSET_X_L      0x1F
#define ADXL357_REG_OFFSET_Y_H      0x20
#define ADXL357_REG_OFFSET_Y_L      0x21
#define ADXL357_REG_OFFSET_Z_H      0x22
#define ADXL357_REG_OFFSET_Z_L      0x23
#define ADXL357_REG_ACT_EN          0x24
#define ADXL357_REG_ACT_THRESH_H    0x25
#define ADXL357_REG_ACT_THRESH_L    0x26
#define ADXL357_REG_ACT_COUNT       0x27
#define ADXL357_REG_FILTER          0x28
#define ADXL357_REG_FIFO_SAMPLES    0x29
#define ADXL357_REG_INT_MAP         0x2A
#define ADXL357_REG_SYNC            0x2B
#define ADXL357_REG_RANGE           0x2C
#define ADXL357_REG_POWER_CTL       0x2D
#define ADXL357_REG_SELF_TEST       0x2E
#define ADXL357_REG_RESET           0x2F


//Set i2c Handler here
extern I2C_HandleTypeDef hi2c1;

typedef struct
{
	I2C_HandleTypeDef *i2cHandle;
	
	float acc_mps2[3];
	
	float Scale_Factor;
	
} ADXL357_t;

typedef enum
{
  ADXL357_ODR_4kHz   = 0x00,
  ADXL357_ODR_2kHz   = 0x01,
  ADXL357_ODR_1kHz   = 0x02,
  ADXL357_ODR_500Hz  = 0x03,
  ADXL357_ODR_250Hz  = 0x04,
  ADXL357_ODR_125Hz = 0x05,
  ADXL357_ODR_62_5Hz = 0x06,
  ADXL357_ODR_31_25kHz  = 0x07,
  ADXL357_ODR_15_625Hz = 0x08,
  ADXL357_ODR_7_813Hz = 0x09,
  ADXL357_ODR_3_906kHz  = 0x0A,	
} ADXL357_DR_t;

typedef enum
{
  ADXL357_10g  = 0x01,
  ADXL357_20g  = 0x02,
  ADXL357_40g  = 0x03,
} ADXL357_FS_t;

uint8_t ADXL357_INIT(ADXL357_t *dev, I2C_HandleTypeDef *i2cHandle, ADXL357_FS_t range, ADXL357_DR_t val);

HAL_StatusTypeDef ADXL357_ReadAcc(ADXL357_t *dev);

HAL_StatusTypeDef ADXL357_ReadRegister(ADXL357_t *dev, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef ADXL357_ReadRegisters(ADXL357_t *dev, uint8_t reg, uint8_t *data, uint8_t len);
HAL_StatusTypeDef ADXL357_WriteRegister(ADXL357_t *dev, uint8_t reg, uint8_t *data);

#endif

#ifndef GYRO_L3GD20_H
#define GYRO_L3GD20_H

#include "stm32l4xx_hal.h"

// as configured on the Adafruit 10 DOF IMU board #1604
#define GYRO_WRITE_ADDR	0xD6
#define GYRO_READ_ADDR   0xD7

// this will be fleshed out to support DMA and/or Interrupt driven transfer
// from the sensor module.
typedef struct _Gyro_t {
	I2C_HandleTypeDef *hi2c;
	uint16_t write;
	uint16_t read;
	uint8_t data[16];
} Gyro_t;

enum GyroRegisters{
	AUTO_INCREMENT = 0x80,
	WHO_AM_I = 0x0F,
	CTRL_REG1 = 0x20,
	CTRL_REG2,
	CTRL_REG3,
	CTRL_REG4,
	CTRL_REG5,
	REFERENCE,
	OUT_TEMP,
	STATUS_REG,
	OUT_X_L,
	OUT_X_H,
	OUT_Y_L,
	OUT_Y_H,
	OUT_Z_L,
	OUT_Z_H,
	FIFO_CTRL_REG,
	FIFO_SRC_REG,
	INT1_CFG,
	INT1_SRC,
	INT1_TSH_XH,
	INT1_TSH_XL,
	INT1_TSH_YH,
	INT1_TSH_YL,
	INT1_TSH_ZH,
	INT1_TSH_ZL,
	INT1_DURATION
};

extern uint8_t GyroInit(I2C_HandleTypeDef *);
extern void GyroRead();

#endif // GYRO_L3GD20_H

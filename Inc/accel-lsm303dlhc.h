#ifndef ACCEL_LSM303DLHC_H
#define ACCEL_LSM303DLHC_H

#include "stm32l4xx_hal.h"

// as configured on the Adafruit 10 DOF IMU board #1604
#define ACCEL_WRITE_ADDR	0x32
#define ACCEL_READ_ADDR		0x33

// this will be fleshed out to support DMA and/or Interrupt driven transfer
// from the sensor module.
typedef struct _Accel_t {
	I2C_HandleTypeDef *hi2c;
	uint16_t write;
	uint16_t read;
	uint8_t data[16];
} Accel_t;

extern uint8_t AccelInit(I2C_HandleTypeDef *);
extern void AccelRead();


#endif // ACCEL_LSM303DLHC_H

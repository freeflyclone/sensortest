#ifndef ACCEL_LSM303DLHC_H
#define ACCEL_LSM303DLHC_H

#include "stm32l4xx_hal.h"

// as configured on the Adafruit 10 DOF IMU board #1604
#define ACCEL_WRITE_ADDR	0x32
#define ACCEL_READ_ADDR		0x33

#define MAG_WRITE_ADDR		0x3C
#define MAG_READ_ADDR		0x3D

enum AccelRegisters {
	CTRL1=0x20,
	CTRL2,
	CTRL3,
	CTRL4,
	CTRL5,
	CTRL6,
	A_REFERENCE,
	STATUS,
	A_OUT_X_L,
	A_OUT_X_H,
	A_OUT_Y_L,
	A_OUT_Y_H,
	A_OUT_Z_L,
	A_OUT_Z_H,
	FIFO_CTRL,
	FIFO_SRC,
	INT_CFG,
	INT_SOURCE,
	INT_THS,
	INT_DURATION,
	INT2_CFG,
	INT2_SOURCE,
	INT2_DURATION,
	CLICK_CFG,
	CLICK_SRC,
	CLICK_THS,
	TIME_LIMIT,
	TIME_LATENCY,
	TIME_WINDOW
};

enum MagRegisters {
	CRA=0x00,
	CRB,
	MR,
	M_OUT_X_H,
	M_OUT_X_L,
	M_OUT_Z_H,
	M_OUT_Z_L,
	M_OUT_Y_H,
	M_OUT_Y_L,
	SR,
	IRA,
	IRB,
	IRC,
	TEMP_H=0x31,
	TEMP_L
};
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

typedef struct _Mag_t {
	I2C_HandleTypeDef *hi2c;
	uint16_t write;
	uint16_t read;
	uint8_t data[16];
} Mag_t;

extern uint8_t MagInit(I2C_HandleTypeDef *);
extern void MagRead();

#endif // ACCEL_LSM303DLHC_H

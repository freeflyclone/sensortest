#include "FreeRTOS.h"
#include "task.h"

#include <accel-lsm303dlhc.h>

Accel_t accel;
Mag_t mag;

uint8_t AccelInit(I2C_HandleTypeDef *hi2c){
	HAL_StatusTypeDef status = HAL_OK;

	accel.write = ACCEL_WRITE_ADDR;
	accel.read = ACCEL_READ_ADDR;
	accel.hi2c = hi2c;
	accel.data[0] = CTRL1;
	accel.data[1] = 0x9F;		// 1.34Khz, normal power, XYZ enabled;

	while ( (status = HAL_I2C_Master_Transmit(accel.hi2c, accel.write, accel.data, 2, 20)) != HAL_OK)
		HAL_Delay(1);

	while ( (status = HAL_I2C_Master_Receive(accel.hi2c, accel.read, accel.data, 1, 20)) != HAL_OK)
		HAL_Delay(1);

	if (accel.data[0] != 0x9F)
		return HAL_ERROR;

	return status;
}

void AccelReadStart() {
	accel.data[0] = A_OUT_X_L | 0x80; // AUTO_INCREMENT;
	HAL_I2C_Master_Transmit_IT(accel.hi2c, accel.write, accel.data, 1);
}

void AccelReadEnd() {
	HAL_I2C_Master_Receive_DMA(accel.hi2c, accel.read, accel.data, 6);
}

void AccelRead(){
	const TickType_t blockTime = 10;

	// Read gyro starts with transmitting device register address
	AccelReadStart();
	// ... wait for transmitting of register address to complete ...
	ulTaskNotifyTake(pdFALSE, blockTime);

	// ... then issue I2C DMA-based read request ...
	AccelReadEnd();
	// ... and wait for it to complete.
	ulTaskNotifyTake(pdFALSE, blockTime);
}

uint8_t MagInit(I2C_HandleTypeDef *hi2c) {
	HAL_StatusTypeDef status = HAL_OK;

	mag.write = MAG_WRITE_ADDR;
	mag.read = MAG_READ_ADDR;
	mag.hi2c = hi2c;
	mag.data[0] = CRA;
	mag.data[1] = 0x1C;		// No Temp Sensor, Mininum data rate 220Hz
	mag.data[2] = 0x20;		// CRB: +- 1.3
	mag.data[3] = 0x00;		// continuous conversion mode.

	while ( (status = HAL_I2C_Master_Transmit(mag.hi2c, mag.write, mag.data, 4, 20)) != HAL_OK)
		HAL_Delay(1);

	while ( (status = HAL_I2C_Master_Receive(mag.hi2c, mag.read, mag.data, 1, 20)) != HAL_OK)
		HAL_Delay(1);

	if (mag.data[1] != 0x1c)
		return HAL_ERROR;

	return status;
}

void MagReadStart() {
	mag.data[0] = M_OUT_X_H | 0x80; //AUTO_INCREMENT;

	HAL_I2C_Master_Transmit_IT(mag.hi2c, mag.write, mag.data, 1);
}

void MagReadEnd() {
	HAL_I2C_Master_Receive_DMA(mag.hi2c, mag.read, mag.data, 6);
}

void MagRead() {
	const TickType_t blockTime = 10;

	// Read gyro starts with transmitting device register address
	MagReadStart();
	// ... wait for transmitting of register address to complete ...
	ulTaskNotifyTake(pdFALSE, blockTime);

	// ... then issue I2C DMA-based read request ...
	MagReadEnd();
	// ... and wait for it to complete.
	ulTaskNotifyTake(pdFALSE, blockTime);
}

#include "FreeRTOS.h"
#include "task.h"

#include <accel-lsm303dlhc.h>

Accel_t accel;
Mag_t mag;

uint8_t AccelInit(I2C_HandleTypeDef *hi2c){
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t cr1 = 0x97;		// 1.34Khz, NOT Low power, XYZ enabled;

	accel.write = ACCEL_WRITE_ADDR;
	accel.read = ACCEL_READ_ADDR;
	accel.hi2c = hi2c;
	accel.data[0] = CTRL1 | 0x80; // AUTO_INCREMENT;
	accel.data[1] = cr1;
	accel.data[2] = 0;
	accel.data[3] = 0;
	accel.data[4] = 0xB0;		// BDU enabled, +-16G
	accel.data[5] = 0;

	while ( (status = HAL_I2C_Master_Transmit(accel.hi2c, accel.write, accel.data, 6, 20)) != HAL_OK)
		HAL_Delay(1);

	while ( (status = HAL_I2C_Master_Receive(accel.hi2c, accel.read, accel.data, 1, 20)) != HAL_OK)
		HAL_Delay(1);

	if (accel.data[1] != cr1)
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
	uint8_t cra = 0x1C;		// No Temp Sensor, Minimum data rate 220Hz

	mag.write = MAG_WRITE_ADDR;
	mag.read = MAG_READ_ADDR;
	mag.hi2c = hi2c;
	mag.data[0] = CRA;
	mag.data[1] = cra;		// No Temp Sensor, Mininum data rate 220Hz
	mag.data[2] = 0x20;		// CRB: +- 1.3
	mag.data[3] = 0x00;		// continuous conversion mode.

	while ( (status = HAL_I2C_Master_Transmit(mag.hi2c, mag.write, mag.data, 4, 20)) != HAL_OK)
		HAL_Delay(1);

	while ( (status = HAL_I2C_Master_Receive(mag.hi2c, mag.read, mag.data, 1, 20)) != HAL_OK)
		HAL_Delay(1);

	if (mag.data[1] != cra)
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

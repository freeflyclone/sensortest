#include "FreeRTOS.h"
#include "task.h"

#include <gyro-l3gd20.h>

Gyro_t gyro;

uint8_t GyroInit(I2C_HandleTypeDef *hi2c) {
	HAL_StatusTypeDef status = HAL_OK;

	gyro.write = GYRO_WRITE_ADDR;
	gyro.read = GYRO_READ_ADDR;
	gyro.hi2c = hi2c;
	gyro.data[0][0] = CTRL_REG1;
	gyro.data[0][1] = 0xFF;	// configure device: ODR:760, Cut-off:100, PD:disabled, X,Y,Z: enabled
	gyro.pingPong = 0;
	gyro.readInProgress = 0;

	while ( (status = HAL_I2C_Master_Transmit(gyro.hi2c, gyro.write, gyro.data[0], 2, 20)) != HAL_OK)
		HAL_Delay(1);

	while ( (status = HAL_I2C_Master_Receive(gyro.hi2c, gyro.read, gyro.data[0], 1, 20)) != HAL_OK)
		HAL_Delay(1);

	if (gyro.data[0][0] != 0xFF)
		return HAL_ERROR;

	return status;
}

void GyroRead() {
	const TickType_t blockTime = 10;

	// Read gyro starts with transmitting device register address
	GyroReadStart();
	// ... wait for transmitting of register address to complete ...
	ulTaskNotifyTake(pdFALSE, blockTime);

	// ... then issue I2C DMA-based read request ...
	GyroReadEnd();
	// ... and wait for it to complete.
	ulTaskNotifyTake(pdFALSE, blockTime);
}

void GyroReadStart() {

	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

	gyro.data[gyro.pingPong&1][0] = OUT_X_L | AUTO_INCREMENT;
	gyro.readInProgress = 1;
	HAL_I2C_Master_Transmit_IT(gyro.hi2c, gyro.write, gyro.data[gyro.pingPong&1], 1);

	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
}

void GyroReadEnd() {

	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

	HAL_I2C_Master_Receive_DMA(gyro.hi2c, gyro.read, gyro.data[gyro.pingPong&1], 6);
	gyro.pingPong++;
	gyro.readInProgress = 0;

	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
}

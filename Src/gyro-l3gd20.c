#include "FreeRTOS.h"
#include "task.h"

#include <gyro-l3gd20.h>

Gyro_t gyro;

uint8_t GyroInit(I2C_HandleTypeDef *hi2c) {
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t cr1 = 0xFF; 	// configure device: ODR:760, Cut-off:30Hz, PD:normal, X,Y,Z: enabled

	gyro.write = GYRO_WRITE_ADDR;
	gyro.read = GYRO_READ_ADDR;
	gyro.hi2c = hi2c;
	gyro.data[0] = CTRL_REG1 | AUTO_INCREMENT;
	gyro.data[1] = cr1;
	gyro.data[2] = 0x0;
	gyro.data[3] = 0x0;
	gyro.data[4] = 0xB0;		// BDU enabled, full scale: 2000 degrees/second
	gyro.data[5] = 0x0;

	while ( (status = HAL_I2C_Master_Transmit(gyro.hi2c, gyro.write, gyro.data, 6, 20)) != HAL_OK)
		HAL_Delay(1);

	while ( (status = HAL_I2C_Master_Receive(gyro.hi2c, gyro.read, gyro.data, 1, 20)) != HAL_OK)
		HAL_Delay(1);

	if (gyro.data[1] != cr1)
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

	gyro.data[0] = OUT_X_L | AUTO_INCREMENT;
	HAL_I2C_Master_Transmit_IT(gyro.hi2c, gyro.write, gyro.data, 1);

	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
}

void GyroReadEnd() {

	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

	HAL_I2C_Master_Receive_DMA(gyro.hi2c, gyro.read, gyro.data, 6);

	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
}

#include <gyro-l3gd20.h>

Gyro_t gyro;

uint8_t GyroInit(I2C_HandleTypeDef *hi2c) {
	HAL_StatusTypeDef status = HAL_OK;

	gyro.write = GYRO_WRITE_ADDR;
	gyro.read = GYRO_READ_ADDR;
	gyro.hi2c = hi2c;
	gyro.data[0] = CTRL_REG1;
	gyro.data[1] = 0xFF;	// configure device: ODR:760, Cut-off:100, PD:disabled, X,Y,Z: enabled

	while ( (status = HAL_I2C_Master_Transmit(gyro.hi2c, gyro.write, gyro.data, 2, 20)) != HAL_OK)
		HAL_Delay(1);

	while ( (status = HAL_I2C_Master_Receive(gyro.hi2c, gyro.read, gyro.data, 1, 20)) != HAL_OK)
		HAL_Delay(1);

	if (gyro.data[0] != 0xFF)
		return HAL_ERROR;

	return status;
}

void GyroRead() {
	HAL_StatusTypeDef status = HAL_OK;

	gyro.data[0] = OUT_X_L | AUTO_INCREMENT;

	while ( (status = HAL_I2C_Master_Transmit(gyro.hi2c, gyro.write, gyro.data, 1, 20)) != HAL_OK)
		HAL_Delay(1);

	while ( (status = HAL_I2C_Master_Receive(gyro.hi2c, gyro.read, gyro.data, 6, 20)) != HAL_OK)
		HAL_Delay(1);

}

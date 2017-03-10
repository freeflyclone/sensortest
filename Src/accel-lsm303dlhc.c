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

void AccelRead(){
	HAL_StatusTypeDef status = HAL_OK;

	accel.data[0] = A_OUT_X_L | 0x80; //AUTO_INCREMENT;

	while ( (status = HAL_I2C_Master_Transmit(accel.hi2c, accel.write, accel.data, 1, 20)) != HAL_OK)
		HAL_Delay(1);

	while ( (status = HAL_I2C_Master_Receive(accel.hi2c, accel.read, accel.data, 6, 20)) != HAL_OK)
		HAL_Delay(1);

}

uint8_t MagInit(I2C_HandleTypeDef *hi2c) {
	HAL_StatusTypeDef status = HAL_OK;

	return status;
}

void MagRead() {

}

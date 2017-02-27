#include <accel-lsm303dlhc.h>

Accel_t accel;

uint8_t AccelInit(I2C_HandleTypeDef *hi2c){
	HAL_StatusTypeDef status = HAL_OK;

	accel.write = ACCEL_WRITE_ADDR;
	accel.read = ACCEL_READ_ADDR;
	accel.hi2c = hi2c;

	return status;
}

void AccelRead(){

}

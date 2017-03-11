#include "FreeRTOS.h"
#include "task.h"

#include "main.h"
#include "usart.h"
#include "stm32l4xx_hal.h"

#include "gyro-l3gd20.h"
#include "accel-lsm303dlhc.h"

TaskHandle_t *imuTask;

uint8_t ImuInit(I2C_HandleTypeDef *hi2c) {
	uint8_t status = HAL_OK;

	imuTask = xTaskGetCurrentTaskHandle();

	if ( (status = GyroInit(hi2c)) != HAL_OK)
		return status;

	if ( (status = AccelInit(hi2c)) != HAL_OK)
		return status;

	if ( (status = MagInit(hi2c)) != HAL_OK)
		return status;

	HAL_UART_Transmit(&huart1, (uint8_t*)"ImuInit Complete\r\n", 18, 10);

	return status;
}

void ImuRead() {
	const TickType_t blockTime = 10;

	GyroReadStart();
	ulTaskNotifyTake(pdFALSE, blockTime);
	GyroReadEnd();
	ulTaskNotifyTake(pdFALSE, blockTime);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	BaseType_t taskWoken;
	taskWoken = pdFALSE;

	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

	vTaskNotifyGiveFromISR(imuTask, &taskWoken);
	portYIELD_FROM_ISR(taskWoken);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	BaseType_t taskWoken;
	taskWoken = pdFALSE;

	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

	vTaskNotifyGiveFromISR(imuTask, &taskWoken);
	portYIELD_FROM_ISR(taskWoken);
}

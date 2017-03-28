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

/*
 * ImuRead() called from ImuTask (defined in freertos.c) infinite loop
 */
void ImuRead() {
	const TickType_t blockTime = 10;

	// Read gyro starts with transmitting device register address
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	GyroReadStart();
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	// ... wait for transmitting of register address to complete ...
	ulTaskNotifyTake(pdFALSE, blockTime);

	// ... then issue I2C interrupt-based read request ...
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	GyroReadEnd();
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	// ... and wait for it to complete.
	ulTaskNotifyTake(pdFALSE, blockTime);

	// signal ImuRead() complete
	//HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	BaseType_t taskWoken;
	taskWoken = pdFALSE;

	//HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

	vTaskNotifyGiveFromISR(imuTask, &taskWoken);
	portYIELD_FROM_ISR(taskWoken);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	BaseType_t taskWoken;
	taskWoken = pdFALSE;

	//HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

	vTaskNotifyGiveFromISR(imuTask, &taskWoken);
	portYIELD_FROM_ISR(taskWoken);
}

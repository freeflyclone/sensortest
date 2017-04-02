#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "main.h"
#include "usart.h"
#include "stm32l4xx_hal.h"

#include "gyro-l3gd20.h"
#include "accel-lsm303dlhc.h"

TaskHandle_t *imuTask;
extern QueueHandle_t usartQueue;

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
 *   Each sensor read function causes this task to pause waiting for
 *   the I2C / DMA interrupt that signals the completion of each
 *   request for I2C data from the respective sensor.
 */
void ImuRead() {
	uint8_t imuBuff[18];
	int i;

	GyroRead();
	AccelRead();
	MagRead();

	// byte swap the mag, mag is MSB first
	for(i=0; i<6; i+=2) {
		imuBuff[i] = gyro.data[i];
		imuBuff[i+1] = gyro.data[i+1];

		imuBuff[i+6] = accel.data[i];
		imuBuff[i+6+1] = accel.data[i+1];

		imuBuff[i+12] = mag.data[i+1];
		imuBuff[i+12+1] = mag.data[i];
	}

	xQueueSend(usartQueue, imuBuff, 10);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	BaseType_t taskWoken;
	taskWoken = pdFALSE;

	vTaskNotifyGiveFromISR(imuTask, &taskWoken);
	portYIELD_FROM_ISR(taskWoken);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	BaseType_t taskWoken;
	taskWoken = pdFALSE;

	vTaskNotifyGiveFromISR(imuTask, &taskWoken);
	portYIELD_FROM_ISR(taskWoken);
}

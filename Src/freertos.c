/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "gpio.h"
#include "i2c.h"
#include "usart.h"
#include "imu.h"

#include "gyro-l3gd20.h"
#include "accel-lsm303dlhc.h"

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId imuTaskHandle;

/* USER CODE BEGIN Variables */
TaskHandle_t ledTaskHandle;
TaskHandle_t imuTaskHandle;
TaskHandle_t usartTaskHandle;

QueueHandle_t ahrsOutputQueue;
QueueHandle_t usartQueue;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void ImuTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
void LEDTask(void *);
void USARTTask(void *);
/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of imuTask */
  osThreadDef(imuTask, ImuTask, osPriorityHigh, 0, 128);
  imuTaskHandle = osThreadCreate(osThread(imuTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  xTaskCreate(USARTTask, "USARTTask", 128, (void *)1, tskIDLE_PRIORITY+1, &usartTaskHandle);
  //xTaskCreate(LEDTask, "LedBlink", 128, (void *)1, tskIDLE_PRIORITY, &ledTaskHandle);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* ImuTask function */
void ImuTask(void const * argument)
{

  /* USER CODE BEGIN ImuTask */
  ImuInit(&hi2c1);

  while(1) {
	ImuRead();
	//osDelay(1);
  }
  /* USER CODE END ImuTask */
}

/* USER CODE BEGIN Application */
void LEDTask(void *pvParameters) {
  while(1) {
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	vTaskDelay(1);
  }
}

void USARTTask(void *pvParameters) {
	uint8_t queueItem[18];
	uint8_t outBuff[36+2];
	static uint8_t hexChars[] = "0123456789ABCDEF";
	int i,j;

	if ( (usartQueue = xQueueCreate(4, 18)) == NULL)
		return;

	if ( (ahrsOutputQueue = xQueueCreate(4, 4*sizeof(float))) == NULL)
		return;

	HAL_UART_Transmit(&huart1, (uint8_t*)"USARTTask init complete\r\n", 25, 10);

	while(1) {
		if (xQueueReceive(usartQueue, queueItem, 10) != pdPASS)
			continue;

		for (i=0,j=0; i<18; i++) {
			uint16_t t = queueItem[i];
			outBuff[j++] = hexChars[(t>>4)&0xF];
			outBuff[j++] = hexChars[t & 0xf];
		}
		outBuff[j++] = '\r';
		outBuff[j++] = '\n';
		HAL_UART_Transmit(&huart1, outBuff, 36+2, 10);
	}
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

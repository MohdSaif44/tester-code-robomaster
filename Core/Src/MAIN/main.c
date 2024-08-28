/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "main.h"



int main(void) {
	set();
	Uarts=huart2;
	usbrxBuffer = xMessageBufferCreate(68);

	const osThreadAttr_t MainTask_attributes = {
				.name = "MainTask",
				.stack_size = 512 * 4,
				.priority = (osPriority_t) osPriorityNormal,
		};

		const osThreadAttr_t EmergencyTask_attributes = {
				.name = "EmergencyTask",
				.stack_size = 256 * 4,
				.priority = (osPriority_t) osPriorityNormal,
		};

		const osThreadAttr_t SecondTask_attributes = {
				.name = "SecondTask",
				.stack_size = 512 * 4,
				.priority = (osPriority_t) osPriorityNormal,
		};

		const osThreadAttr_t ThirdTask_attributes = {
				.name = "ThirdTask",
				.stack_size = 512 * 4,
				.priority = (osPriority_t) osPriorityNormal,
		};

		const osThreadAttr_t CalculationTask_attributes = {
				.name = "CalculationTask",
				.stack_size = 256 * 10,
				.priority =	(osPriority_t) osPriorityNormal,
		};

		const osSemaphoreAttr_t CalcSemaphore_attributes = {
				.name = "CalcSemaphore"
		};

	osKernelInitialize();

	MainTaskHandle = osThreadNew(MainTask, NULL, &MainTask_attributes);
	SecondTaskHandle = osThreadNew(usbTask, NULL, &SecondTask_attributes);
	ThirdTaskHandle = osThreadNew(ThirdTask, NULL, &ThirdTask_attributes);
	CalculationTaskHandle = osThreadNew(Calculation, NULL, &CalculationTask_attributes);
	CalcSemaphore = osSemaphoreNew(1, 0, &CalcSemaphore_attributes);
	EmergencyTaskHandle = osThreadNew(EmergencyTask, NULL, &EmergencyTask_attributes);
	osKernelStart();
	while (1) {

	}
}


void TIM7_IRQHandler(void) { //500ms
#ifdef perfect
	sys.transmit = 1;
#endif
	if (state == GPIOS || state == ENCODER || state == CHANGEUART) {
		sys.transmit = 1;
	}
	if (sys.i2c == 1||sys.can==1) {
		count++;
	} else {
		count = 0;
		led2=!led2;
	}
	if (count > 2 && sys.i2c == 1) {
		sprintf(buff_transmit, "\nFailed!\n\n");
		UART_Send();
		sys.transmit = 1;
		sys.i2c = 0;
	} else if (count > 2 && sys.can == 1) {
		sprintf(buff_transmit, "\nFailed!\n\n");
		UART_Send();
		sys.transmit = 1;
		sys.can = 0;
	}

	HAL_TIM_IRQHandler(&htim7);
}


void TIM6_DAC_IRQHandler(void) { //20ms

	osSemaphoreRelease(CalcSemaphore);
	led1 = !led1;
	HAL_TIM_IRQHandler(&htim6);
}

void MainTask(void *argument) {
#ifdef newpin
	HAL_UART_Receive_IT(&huart1,buff_receive,1);
#endif
	HAL_UART_Receive_IT(&huart2,buff_receive,1);
	HAL_UART_Receive_IT(&huart3,buff_receive,1);
	HAL_UART_Receive_IT(&huart4,buff_receive,1);
	HAL_UART_Receive_IT(&huart5,buff_receive,1);
#ifndef perfect
	HAL_SPI_Receive_IT(&hspi1,(uint8_t*) buff_transmit, 1);
#endif
	memcpy(&complete,"S",1);

	while(1)
	{
		if (sys.i2c == 1) {
			switch (i2c_state) {
			case 1:
				HAL_I2C_Slave_Receive_IT(&hi2c2, (uint8_t*) buff_transmit, 1);
				I2CSend(&hi2c1, main_board_2, 1, complete);
				break;
			case 2:
				HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t*) buff_transmit, 1);
				I2CSend(&hi2c2, main_board_1, 1, complete);
				break;
#ifndef newboard
			case 3:
				HAL_I2C_Slave_Receive_IT(&hi2c3, (uint8_t*) buff_transmit, 1);
				I2CSend(&hi2c1, main_board_3, 1, complete);
				break;
			case 4:
				HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t*) buff_transmit, 1);
				I2CSend(&hi2c3, main_board_1, 1, complete);
				break;
			case 5:
				HAL_I2C_Slave_Receive_IT(&hi2c3, (uint8_t*) buff_transmit, 1);
				I2CSend(&hi2c2, main_board_3, 1, complete);
				break;
			case 6:
				HAL_I2C_Slave_Receive_IT(&hi2c2, (uint8_t*) buff_transmit, 1);
				I2CSend(&hi2c3, main_board_2, 1, complete);
				break;
#endif

			}
			i2c_state = 0;
		}

		if (sys.uart == 1) {
			sprintf(buff_transmit, "UART channel has been changed to %d\r\n",UART_state);
			UART_Send();
			switch (UART_state) {
#ifdef newpin
			case 1:
				Uarts=huart1;
				break;
#endif
			case 2:
				Uarts = huart2;
				break;
			case 3:
				Uarts = huart3;
				break;
			case 4:
				Uarts = huart4;
				break;
			case 5:
				Uarts = huart5;
				break;
			}
			sys.uart = 0;
			state = CHANGEUART;
		}

	}
}

void usbTask(void *argument) {
#ifdef newpin
	while(1)
	{

	}
#else
	uint8_t UsbDataOut[64];
	MX_USB_DEVICE_Init();
	//	usbParserinit();
	//	usbPrint();
	for (;;) {

		if(sys.usb == 1){

			sprintf(UsbDataOut,"USB port is working properly\r\n");

			CDC_Transmit_FS(UsbDataOut, sizeof(UsbDataOut));

			sprintf(UsbDataOut,"\0");

			//		}
			//
			//
			//
			//
			//		size_t bytesRead = xMessageBufferReceive(usbrxBuffer, receivedData, 64,
			//				portMAX_DELAY);
			//
			//		// Process the received data here
			//		if (bytesRead == sizeof(receivedData)) {
			//
			//			out_usbmsg.flags = in_usbmsg.flags;
			//			usbHandler(receivedData);
			//			static int counter = 0;
			//			counter++;
			//			if (counter % 400 == 0) {
			//				led3 = !led3;
			//				counter = 0;
			//
			//			}
		}
	}
#endif
}
void ThirdTask(void *argument) {
	sys.transmit = 1;
	led3=1;
	while (1) {
		//prompt messages
#ifndef perfect
		Prompt();
#else
		if(sys.transmit){
			HAL_SPI_Transmit(&hspi1, complete, 1, 100);
			CAN_TxMsg(&hcan1, 33, complete, 1);
			CAN_TxMsg(&hcan1, 32, complete, 1);
			CAN_TxMsg(&hcan2, 33, complete, 1);
			CAN_TxMsg(&hcan2, 32, complete, 1);
		}
#endif
	}
}
void Calculation(void *argument) { //20ms
	while (1) {

		osSemaphoreAcquire(CalcSemaphore, osWaitForever);
		RBMS_5ms(&rbms1);

	}
}
void EmergencyTask(void *argument) {

	for (;;) {

		if (ps4.button == TOUCH) {
			sys.flags=0;
			osThreadTerminate(MainTaskHandle);
			osThreadTerminate(SecondTaskHandle);
			osThreadTerminate(ThirdTaskHandle);
			osDelay(5);

			const osThreadAttr_t MainTask_attributes = { .name = "MainTask",
					.stack_size = 512 * 4, .priority =
							(osPriority_t) osPriorityNormal, };

			const osThreadAttr_t SecondTask_attributes = { .name = "SecondTask",
					.stack_size = 512 * 4, .priority =
							(osPriority_t) osPriorityNormal, };

			const osThreadAttr_t ThirdTask_attributes = { .name = "ThirdTask",
					.stack_size = 512 * 4, .priority =
							(osPriority_t) osPriorityNormal, };

			MainTaskHandle = osThreadNew(MainTask, NULL, &MainTask_attributes);
			SecondTaskHandle = osThreadNew(usbTask, NULL,
					&SecondTask_attributes);
			ThirdTaskHandle = osThreadNew(ThirdTask, NULL,
					&ThirdTask_attributes);
		}
	}
}

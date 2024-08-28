/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "interrupt.h"
#include "common.h"
/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
#define USED_QEI1
#define USED_QEI4
#define USED_QEI6

int count = 0;
int count2 = 0;
int count3 = 0;
int _counter = 0;

void SysTick_Handler(void) {

	HAL_IncTick();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
	if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
#endif /* INCLUDE_xTaskGetSchedulerState */
		xPortSysTickHandler();
#if (INCLUDE_xTaskGetSchedulerState == 1 )
	}
#endif /* INCLUDE_xTaskGetSchedulerState */

}

/**
 * * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void) {

}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void) {

}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void) {

	while (1) {

	}
}

/**
 * @brief This function handles Pre-fetch fault, memory access fault.
 */
void BusFault_Handler(void) {

}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void) {

}

/**
 * @brief This function handles System service call via SWI instruction.
 */
//void SVC_Handler(void)
//{
//
//}
/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void) {

}

/**
 * @brief This function handles Pendable request for system service.
 */
//void PendSV_Handler(void)
//{
//
//}
/**
 * @brief This function handles System tick timer.
 */
//void SysTick_Handler(void)
//{
//
//	HAL_IncTick();
//
//}
void TIM1_UP_TIM10_IRQHandler(void) {
#ifdef USED_QEI1
	if (htim1.Instance->CR1 == 129) {
		BIOS_QEI1.signbit += 1;
	} else if (htim1.Instance->CR1 == 145) {
		BIOS_QEI1.signbit -= 1;
	}
	htim1.Instance->SR = 0;
	QEIDelay(200);
#else
	HAL_TIM_IRQHandler(&htim1);
#endif
//	HAL_TIM_IRQHandler(&htim10);
	return;

}

void TIM4_IRQHandler(void) {

#ifdef USED_QEI4
	if (htim4.Instance->CR1 == 129) {
		BIOS_QEI4.signbit += 1;
	} else if (htim4.Instance->CR1 == 145) {
		BIOS_QEI4.signbit -= 1;
	}
	htim4.Instance->SR = 0;
	QEIDelay(100);

#else
	HAL_TIM_IRQHandler(&htim4);

	return;
#endif

}

void TIM8_UP_TIM13_IRQHandler(void) {
#ifdef USED_QEI6
	if (htim8.Instance -> CR1 == 129)
	{
		BIOS_QEI6.signbit += 1;
	}
	else if (htim8.Instance ->CR1 == 145)
	{
		BIOS_QEI6.signbit -= 1;
	}
	htim8.Instance -> SR = 0;
	QEIDelay(200);
#else
	HAL_TIM_IRQHandler(&htim8);
#endif
//	HAL_TIM_IRQHandler(&htim13);
	return;
}

void TIM2_IRQHandler(void) {
	HAL_TIM_IRQHandler(&htim2);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		HAL_IncTick();
		MUXUpdate(&MUX);
		SHIFTREGShift(&SR);
		counter++;
		smttime++;
	}
}

int ERflag = 0;
//Callback for I2C RXBuffer
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	if (hi2c == ps4.hi2cps4) {
		PSxConnectDMA(&ps4);
	} else if (hi2c == IMU.hi2cimu) {
		IMUConnectI2C(&IMU);
	}
	ERflag = 1;
}

void I2C1_ER_IRQHandler(void) {
	HAL_I2C_ER_IRQHandler(&hi2c1);

	if (ERflag) {
		if (ps4.hi2cps4 == &hi2c1) {
			HAL_DMA_DeInit(&hi2c1_rx_dma);
			HAL_I2C_DeInit(&hi2c1);
			I2CX_DMA_RX_Init(&hi2c1, &hi2c1_rx_dma, main_board_1,
			CLOCK_SPEED_400KHz);
			PSxInitDMA(&ps4, &hi2c1);
		} else if (IMU.hi2cimu == &hi2c1) {
			HAL_I2C_DeInit(&hi2c1);
			I2CxInit(&hi2c1, main_board_1, CLOCK_SPEED_100KHz, ENABLE);
			HAL_I2C_Master_Receive_IT(IMU.hi2cimu, 0x35 << 1,
					(uint8_t*) IMU.Buffer, 20);
		}
		ERflag = 0;
	}
}

void I2C2_ER_IRQHandler(void) {
	HAL_I2C_ER_IRQHandler(&hi2c2);

	if (ERflag) {
		if (IMU.hi2cimu == &hi2c2) {
			HAL_I2C_DeInit(&hi2c2);
			I2CxInit(&hi2c2, main_board_2, CLOCK_SPEED_100KHz, ENABLE);
			HAL_I2C_Master_Receive_IT(IMU.hi2cimu, 0x35 << 1,
					(uint8_t*) IMU.Buffer, 20);
		}
		ERflag = 0;
	}
}

/*
 * Function Name		: I2C3_ER_IRQHandler
 * Function Description : I2C3 Error interrupt handler.
 * Function Remarks		: This interrupt handle the error event of I2C3.
 * Function Arguments	: None
 * Function Return		: None
 * Function Example		: None
 */
void I2C3_ER_IRQHandler(void) {
	HAL_I2C_ER_IRQHandler(&hi2c3);

	if (ERflag) {

//		HAL_DMA_DeInit(&hi2c3_rx_dma);
		HAL_I2C_DeInit(&hi2c3);

//		I2CX_DMA_RX_Init(&hi2c3, &hi2c3_rx_dma, main_board_3, CLOCK_SPEED_100KHz);
		I2CxInit(&hi2c3, main_board_3, CLOCK_SPEED_100KHz, ENABLE);

		if (IMU.hi2cimu == &hi2c3) {
			HAL_I2C_Master_Receive_IT(IMU.hi2cimu, 0x35 << 1,
					(uint8_t*) IMU.Buffer, 20);
		}
		ERflag = 0;
	}
}
//void I2C3_ER_IRQHandler(void){
//
//	HAL_I2C_DeInit(&hi2c3);
//
//	I2CxInit (&hi2c3,main_board_1, CLOCK_SPEED_100KHz,DISABLE);
//
//	HAL_I2C_ER_IRQHandler(&hi2c3);
//
//}

void USART1_IRQHandler(void) {

	HAL_UART_IRQHandler(&huart1);
}

void USART2_IRQHandler(void) {

	HAL_UART_IRQHandler(&huart2);
}

void USART3_IRQHandler(void) {

	HAL_UART_IRQHandler(&huart3);
}

void UART4_IRQHandler(void) {

	HAL_UART_IRQHandler(&huart4);
}

void UART5_IRQHandler(void) {

	HAL_UART_IRQHandler(&huart5);
}

void USART6_IRQHandler(void) {

	HAL_UART_IRQHandler(&huart6);
}

/**
 * @brief This function handles USB On The Go FS global interrupt.
 */
void OTG_FS_IRQHandler(void) {

//	led3 = !led3;
	HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
//	if (huart == &huart5) {
//		usbPrint();
//	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	Testboard_Handler();
	HAL_UART_Receive_IT(huart,buff_receive,1);

}
uint8_t aData[8] = {0};
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(sys.can){
		uint8_t aData[8];
		if(hcan == &hcan1){
			HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN1RxMessage, aData);
			if(CAN1RxMessage.StdId==33)
			{
				if (aData[0]=='S') {
					sprintf(buff_transmit, "\nSUCCESS\n\n");
					sys.can=0;
					HAL_UART_Transmit(&huart2,(uint8_t *)buff_transmit,strlen(buff_transmit),100);
					sys.transmit=1;

				}
			}
		}
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	}
	PACKET_t source = 0;
	if (hcan == &hcan1) {
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN1RxMessage, aData);

		if (CAN1RxMessage.IDE == CAN_ID_EXT) {
//			led2 = !led2;
			vescmsg.Rxmsg = CAN1RxMessage;
			memcpy(vescmsg.Data, aData, 8);
			source = VESC_PACKET;


		} else if (CAN1RxMessage.IDE == CAN_ID_STD) {
			uint16_t command_mask = 0x7E0; // mask first 7 bits, get only the id
			uint16_t id = CAN1RxMessage.StdId & command_mask;
			id = id >> 5;
			int i = 0;
//			for (i = 0; i < number_of_odrive; i++) {
//				if (id == P_to_Odrive[i]->Instance) {
//					source = ODRIVE_PACKET;
//					Odrvmsg.RXmsg = CAN1RxMessage;
//					memcpy(Odrvmsg.Data, aData, 8);
//					decode_Odrive(P_to_Odrive[i]);
//					break;
//				}
//			}
			if (CAN1RxMessage.StdId >= 0x201 && CAN1RxMessage.StdId <= 0x208){
				RBMS_CAN_Handler(&CAN1RxMessage, aData);
//				led2 = !led2;
				source = RBMS_PACKET;
			}
		}
		CAN_PROCESS(source);
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	} else {
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN2RxMessage, aData);

		if (CAN2RxMessage.IDE == CAN_ID_EXT) {

			vescmsg.Rxmsg = CAN2RxMessage;
			memcpy(vescmsg.Data, aData, 8);
			set_rx_frames(&vescmsg);
			source = VESC_PACKET;

		} else {

			if (CAN2RxMessage.StdId >= 0x201 && CAN2RxMessage.StdId <= 0x208){
				RBMS_CAN_Handler(&CAN2RxMessage, aData);
				source = RBMS_PACKET;
			}

		}
		CAN_PROCESS(source);
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(sys.can){
		uint8_t aData[8];
		led3=0;
		if(hcan == &hcan2){
			HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &CAN2RxMessage, aData);
			if(CAN2RxMessage.StdId==32)
			{
				if (aData[0] == 'S') {
					sprintf(buff_transmit, "\nSUCCESS\n\n");
					sys.can=0;
					HAL_UART_Transmit(&huart2,(uint8_t *)buff_transmit,strlen(buff_transmit),100);
					sys.transmit=1;
				}
			}
		}
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
	}
	uint8_t aData[8] = {0};
	PACKET_t source = 0;
	if (hcan == &hcan1) {

		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &CAN1RxMessage, aData);

		if (CAN1RxMessage.IDE == CAN_ID_EXT) {
			vescmsg.Rxmsg = CAN1RxMessage;
			memcpy(vescmsg.Data, aData, 8);
			set_rx_frames(&vescmsg);
			source = VESC_PACKET;

		} else {
			if (CAN1RxMessage.StdId >= 0x201 && CAN1RxMessage.StdId <= 0x208){
				RBMS_CAN_Handler(&CAN1RxMessage, aData);
				source = RBMS_PACKET;
			}
		}
		CAN_PROCESS(source);
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);

	} else {

		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &CAN2RxMessage, aData);

		if (CAN2RxMessage.IDE == CAN_ID_EXT) {
			vescmsg.Rxmsg = CAN2RxMessage;
			memcpy(vescmsg.Data, aData, 8);
			set_rx_frames(&vescmsg);
			source = VESC_PACKET;

		} else if (CAN2RxMessage.IDE == CAN_ID_STD) {
//			sprintf(data, "can2 %d \n", CAN2RxMessage.StdId);
//			HAL_UART_Transmit(&huart2, data, strlen(data), 100);
			uint16_t command_mask = 0x7E0; // mask first 7 bits, get only the id
			uint16_t id = CAN2RxMessage.StdId & command_mask;
			id = id >> 5;
//			sprintf(data, "id is %d \n", id);
//			UARTPrintString(&huart2, data);
//			led2 = !led2;
			int i = 0;
//			for (i = 0; i < number_of_odrive; i++) {
//				if (id == P_to_Odrive[i]->Instance) {
//					source = ODRIVE_PACKET;
//					Odrvmsg.RXmsg = CAN2RxMessage;
//					memcpy(Odrvmsg.Data, aData, 8);
//					decode_Odrive(P_to_Odrive[i]);
//					break;
//				}
//			}
			if (CAN2RxMessage.StdId >= 0x201  && CAN2RxMessage.StdId <= 0x208){
				RBMS_CAN_Handler(&CAN2RxMessage, aData);
//				sprintf(data, "%x\n",(CAN2RxMessage.StdId-0x201));

//				led2 = !led2;
				source = RBMS_PACKET;
			}
		}
		CAN_PROCESS(source);
		HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
	}

}
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c) {
	sprintf(buff_transmit, "\nSUCCESS\n\n");
	HAL_UART_Transmit(&Uarts, (uint8_t*)buff_transmit, strlen(buff_transmit), 1000);
	sys.transmit=1;
	sys.i2c=0;
}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef * hspi)
{
#ifndef perfect
	sprintf(buff_transmit, "\nSUCCESS\n\n");
	HAL_UART_Transmit(&Uarts, (uint8_t*)buff_transmit, strlen(buff_transmit), 1000);
	sys.transmit=1;
	sys.i2c=0;
#endif
	HAL_SPI_Receive_IT(&hspi1,(uint8_t*) buff_transmit, 1);
}

void Error_Handler(void) {

	__disable_irq();
	while (1) {
	}
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

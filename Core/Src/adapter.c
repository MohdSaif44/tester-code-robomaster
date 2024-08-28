/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "adapter.h"
uint8_t buf2_flag = 0;

/************************************************/
/*		 	 	Functions		       		  	*/
/************************************************/
void Initialize(){
	/* MCU Configuration--------------------------------------------------------*/
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	GPIOPinsInit (LED1_PIN, GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);
	GPIOPinsInit (LED2_PIN, GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);
	GPIOPinsInit (LED3_PIN, GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);

	GPIOPinsInit (PB1_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);
	GPIOPinsInit (PB2_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);

	/*Normal IOs*/
	GPIOPinsInit (IP1_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP2_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP3_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP4_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP5_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP6_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
#ifdef mainboard
	GPIOPinsInit (IP7_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP8_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP9_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP10_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP11_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP12_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP13_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP14_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP15_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);

	GPIOPinsInit (IP16_Analog1_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP17_Analog2_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP18_Analog3_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP19_Analog4_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP20_Analog5_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP21_Analog6_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);

	MUXInit(&MUX, MUX1_INPUT_PIN, MUX1_S0_PIN, MUX1_S1_PIN, MUX1_S2_PIN);
#endif


#ifdef newboard
	GPIOPinsInit (PB3_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);

	GPIOPinsInit (IP7_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP8_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP9_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP10_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP11_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP12_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);

	GPIOPinsInit (IP16_Analog1_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP17_Analog2_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP18_Analog3_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP19_Analog4_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP20_Analog5_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP21_Analog6_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
#endif


#ifdef newpin
	GPIOPinsInit (IP7_Analog1_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
	GPIOPinsInit (IP8_Analog2_PIN, GPIO_MODE_INPUT,GPIO_SPEED_FREQ_MEDIUM, GPIO_PULLUP);
//	UARTInit(&huart1, 115200, ENABLE);
//	QEIInit(&htim3);
#endif
	SHIFTREGInit (&SR, CASCADE_1, SR_SCK_PIN, SR_RCK_PIN, SR_SI_PIN);

	//https://stackoverflow.com/questions/50243996/what-are-valid-values-of-hal-nvic-setpriority-when-using-stm32-and-freertos
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	/*************** Set Interrupt Priorities in BIOS/priorities.h ***************/
	I2CxInit (&hi2c1, main_board_1, CLOCK_SPEED_100KHz, ENABLE);
	I2CxInit (&hi2c2, main_board_2, CLOCK_SPEED_100KHz, ENABLE);
	I2CxInit (&hi2c3, main_board_3, CLOCK_SPEED_100KHz, ENABLE);

//	I2CX_DMA_RX_Init(&hi2c1, &hi2c1_rx_dma, main_board_1, CLOCK_SPEED_400KHz);
//	I2CX_DMA_RX_Init(&hi2c2, &hi2c2_rx_dma, main_board_2, CLOCK_SPEED_100KHz);

	//Servo Driver - recommended to use 100KHz I2C as 400KHz hang frequently
//	ServoDriverInit(&srv_drv,&hi2c3,0x40);

//	UARTx_DMA_Rx_Init(&huart4, &hdma_uart4_rx, 115200);
//	UARTx_DMA_Rx_Init(&huart2, &hdma_usart2_rx, 115200);//Bluebee Tuning
#ifdef newpin
	UARTInit(&huart1, 115200, ENABLE);
#endif
	UARTInit(&huart2, 115200, ENABLE);					//ros
	UARTInit(&huart3, 115200, ENABLE);
	UARTInit(&huart4, 115200, ENABLE);					//tfmini
	UARTInit(&huart5, 115200, ENABLE);

	QEIInit(&htim1);	//X
	QEIInit(&htim4);
	QEIInit(&htim8);


//	CANxInit(&hcan1,4,CAN_FILTER_FIFO0,CAN_FILTERSCALE_16BIT,RNS_TO_mainboard,(0x7FF - 0x3),0,CAN_500KHz);	//receive data from RNS board only
//	CANxInit(&hcan1,4,CAN_FILTER_FIFO0,CAN_FILTERSCALE_32BIT,0,0,10,CAN_1MHz);
//	CANxInit(&hcan2,4,CAN_FILTER_FIFO1,CAN_FILTERSCALE_32BIT,0,0,14,CAN_500KHz);
//	CANxInit(&hcan2,4,CAN_FILTER_FIFO1,CAN_FILTERSCALE_32BIT,0,0,14,CAN_1MHz);

	CANxInit(&hcan1, CAN_FILTER_FIFO0, CAN_FILTERSCALE_32BIT, 0, 0, 10, CAN_1MHz);// RNS, 3 vesc, 1 robomaster
	CANxInit(&hcan2, CAN_FILTER_FIFO1, CAN_FILTERSCALE_32BIT, 0, 0, 14, CAN_1MHz);// 5 vesc


	PWMTimeBaseInit(&htim3, 20000, 84);
	PWMChannelConfig(&htim3, TIM_CHANNEL_3, TIM3_CHANNEL3_PIN);
	PWMChannelConfig(&htim3, TIM_CHANNEL_4 , TIM3_CHANNEL4_PIN);

	PWMTimeBaseInit(&htim5, 20000, 84);
	PWMChannelConfig(&htim5, TIM_CHANNEL_1, TIM5_CHANNEL1_PIN);
	PWMChannelConfig(&htim5, TIM_CHANNEL_2, TIM5_CHANNEL2_PIN);
	PWMChannelConfig(&htim5, TIM_CHANNEL_3, TIM5_CHANNEL3_PIN);
	PWMChannelConfig(&htim5, TIM_CHANNEL_4, TIM5_CHANNEL4_PIN);

	PWMTimeBaseInit(&htim9, 20000, 168);
	PWMChannelConfig(&htim9, TIM_CHANNEL_1, TIM9_CHANNEL1_PIN);
	PWMChannelConfig(&htim9, TIM_CHANNEL_2, TIM9_CHANNEL2_PIN);

	BDCInit(&BDC1, &htim3, TIM_CHANNEL_4, SHIFTREG, &(SR.cast[1]), Bit6, Bit7);
	BDCInit(&BDC2, &htim3, TIM_CHANNEL_3, SHIFTREG, &(SR.cast[1]), Bit4, Bit5);
	BDCInit(&BDC3, &htim9, TIM_CHANNEL_1, SHIFTREG, &(SR.cast[1]), Bit2, Bit3);
	BDCInit(&BDC4, &htim9, TIM_CHANNEL_2, SHIFTREG, &(SR.cast[1]), Bit0, Bit1);
	BDCInit(&BDC5, &htim5, TIM_CHANNEL_4, SHIFTREG, &(SR.cast[0]), Bit6, Bit7);
	BDCInit(&BDC6, &htim5, TIM_CHANNEL_3, SHIFTREG, &(SR.cast[0]), Bit4, Bit5);
	BDCInit(&BDC7, &htim5, TIM_CHANNEL_2, SHIFTREG, &(SR.cast[0]), Bit2, Bit3);
	BDCInit(&BDC8, &htim5, TIM_CHANNEL_1, SHIFTREG, &(SR.cast[0]), Bit0, Bit1);

/******Laser******/
//	ADC_DMAxInit(&adc,&hadc2,&hdma_adc1,6);
//	ADC_Channel_Config(&adc,ADC_CHANNEL_10,IP16_Analog1_PIN);
//	ADC_Channel_Config(&adc,ADC_CHANNEL_11,IP17_Analog2_PIN);
//
//	LaserInit(&X_laser, 7.0, 0.03, 0.15, 0.03);
//	LaserInit(&Y_laser, 7.0, 0.03, 2.0, 0.03);

//	KalmanFilterInit(1, &(adc.ADC_value[0]), &x_kfo, 7.27, 16.03, 0.15, &kf_adc_x);
//	KalmanFilterInit(1, &(adc.ADC_value[1]), &y_kfo, 7.5, 12.01, 2.0, &kf_adc_y);

//	Moving_Average_Init (&move_aveX, &x_kfo, &x_kfoav);
//	Moving_Average_Init (&move_aveY, &y_kfo, &y_kfoav);

/******Servo******/
//	ServoxInit(&servo, &htim3, TIM3_CHANNEL4_PIN, TIM_CHANNEL_4);
//	ServoInitPulseLimit(&servo, 500, 2500);
//	ServoInitAngle(&servo, 270);

/******SPI******/
//	SPIx_DMA_TX_Init(&hspi1, &hdma_spi1_tx, SPI1_NSS_PIN, SPI_MODE_MASTER);
#ifndef newpin
#ifdef perfect
	SPIxInit(&hspi1, SPI1_NSS_PIN, SPI_MODE_MASTER, ENABLE);
#else
	SPIxInit(&hspi1, SPI1_NSS_PIN, SPI_MODE_SLAVE, ENABLE);
#endif
	GPIOPinsInit(SPI1_MISO_PIN, GPIO_MODE_OUTPUT_PP,GPIO_SPEED_FREQ_HIGH, GPIO_PULLUP);
#endif

/******GPIO*****/
	//Unused peripheral pins can be used as GPIO Input or Output

/*****Motor*****/
//	HAL_Delay(999);	//wait rns board ready
//	HAL_Delay(999);	//wait rns board ready
//	IMU_InitI2C(&IMU, &hi2c2);
//	OdriveInit(&odrive, &hcan1, ODRIVE1, ENCODER_MODE_INCREMENTAL);
//	VESCInit(VESC1, VESC2, VESC3, VESC4, &vesc);
//	FHInit(pfh[0], &hcan1, faulhaber1, 0, 3000, 2147483647, -2147483647);
}


void CAN1_RX0_IRQHandler()
{
	HAL_CAN_IRQHandler(&hcan1);
}

void CAN1_RX1_IRQHandler()
{
	HAL_CAN_IRQHandler(&hcan1);
}

void CAN2_RX0_IRQHandler()
{
	HAL_CAN_IRQHandler(&hcan2);
}

void CAN2_RX1_IRQHandler()
{
	HAL_CAN_IRQHandler(&hcan2);
}

void CAN_PROCESS(PACKET_t packet_src) {

	switch (packet_src) {
	case VESC_PACKET:
		decode_VESC();
		if (vesc.error_flag) {
			vesc.error_flag = 0;
////			strcpy(data, vescerror);
////			UARTSend
		}
		break;

	case ODRIVE_PACKET:
//		SwerveCANHandler();

		break;

//	case RNS_PACKET:
//		if (insData_receive[0] == 1) {
//			rns.RNS_data.common_instruction = insData_receive[1];
//			insData_receive[0] = 2;
//		}
//		if (insData_receive[0] == 17) {
//			if (buf2_flag == 1) {
//				rns.RNS_data.common_instruction = insData_receive[1];
//				rns.RNS_data.common_buffer[0].data = buf1_receive[0].data;
//				rns.RNS_data.common_buffer[1].data = buf1_receive[1].data;
//				rns.RNS_data.common_buffer[2].data = buf2_receive[0].data;
//				rns.RNS_data.common_buffer[3].data = buf2_receive[1].data;
//				insData_receive[0] = 3;
//			}
//		}
//
//		break;

	case RBMS_PACKET:
		break;
//	case CYBERGEAR_PACKET:

//		break;
	}
}





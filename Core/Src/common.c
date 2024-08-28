/*********************************************/
/*          Include Header                   */
/*********************************************/

#include "common.h"

void set(void) {

	Initialize();
//	TIMxInit(&htim7, 20000, 84);
//	PSxInitDMA(&ps4,&hi2c1);
	TIMxInit(&htim6, 20000, 84);	//20ms
	TIMxInit(&htim7, 50000, 840);		//500ms
//	FH_WaitInit();

	/***NAVI***/
//	RNS_config(&hcan1);
//	MODNInit(MODN_FWD_OMNI, 2.0, 0.625, 3.0, 0.03, 800);
	RBMS_Init(&rbms1, &hcan1, RBMS_1234);
	RBMS_Config(&rbms1, RBMS1, C610, 1);
	RBMS_Config(&rbms1, RBMS2, C610, 1);
	RBMS_Config(&rbms1, RBMS3, C610, 1);
	RBMS_Config(&rbms1, RBMS4, C610, 1);
	RBMS_PID_Init(&rbms1);
	RBMS_Set_Control_Mode(&rbms1, RBMS1, VELOCITY);
	RBMS_Set_Control_Mode(&rbms1, RBMS2, VELOCITY);
	RBMS_Set_Control_Mode(&rbms1, RBMS3, VELOCITY);
	RBMS_Set_Control_Mode(&rbms1, RBMS4, VELOCITY);

}

void Testboard_Handler(void) {
	switch (state) {
	case MAIN:
		switch (buff_receive[0]) {
		case '1':
			state = GPIOS;
			sys.transmit=1;
			break;
		case '2':
			state = HSPM;
			sys.transmit=1;
			break;
		case '3':
			state = ENCODER;
			sys.transmit=1;
			break;
		case '4':
			state = UART;
			sys.transmit=1;
			break;
		case '5':
			state = I2C;
			sys.transmit=1;
			break;
		case '6':
			state = CAN;
			sys.transmit=1;
			break;
		case '7':
			state = USB;
			sys.transmit=1;
			break;
		case '8':
			state = ROBOMASTER;
			sys.transmit=1;
			break;
		case 'B':
			sys.prompt = 1;
			break;
		}
		break;
	case GPIOS:
		if (buff_receive[0] == 'B') {
			state = 0;
			sys.prompt = 1;
		}
		break;
	case HSPM:
		if (buff_receive[0] == 'B') {
			state = 0;
			sys.prompt = 1;
		} else if (buff_receive[0] == 'C') {
			sys.pwm ^= 1;
			if (sys.pwm == 0) {
				WriteBDC(&BDC1, 10000);
				WriteBDC(&BDC2, 10000);
				WriteBDC(&BDC3, 10000);
				WriteBDC(&BDC4, 10000);
				WriteBDC(&BDC5, 10000);
				WriteBDC(&BDC6, 10000);
				WriteBDC(&BDC7, 10000);
				WriteBDC(&BDC8, 10000);
			} else {
				WriteBDC(&BDC1, -10000);
				WriteBDC(&BDC2, -10000);
				WriteBDC(&BDC3, -10000);
				WriteBDC(&BDC4, -10000);
				WriteBDC(&BDC5, -10000);
				WriteBDC(&BDC6, -10000);
				WriteBDC(&BDC7, -10000);
				WriteBDC(&BDC8, -10000);
			}
		}
		break;
	case ENCODER:
		if (buff_receive[0] == 'B') {
			state = 0;
			sys.prompt = 1;
		}
		break;
	case UART:
		switch (buff_receive[0]) {
#ifdef newpin
		case '1':
			UART_state = 1;
			sys.uart = 1;
			break;
#endif
		case '2':
			UART_state = 2;
			sys.uart = 1;
			break;
		case '3':
			UART_state = 3;
			sys.uart = 1;
			break;
		case '4':
			UART_state = 4;
			sys.uart = 1;
			break;
		case '5':
			UART_state = 5;
			sys.uart = 1;
			break;
		case 'B':
			state = 0;
			sys.prompt = 1;
			break;
		}
		break;
	case I2C:
		switch (buff_receive[0]) {
		case '1':
			i2c_state = 1;
			sys.i2c = 1;
			break;
		case '2':
			i2c_state = 2;
			sys.i2c = 1;
			break;
		case '3':
#ifndef newboard
			i2c_state = 3;
			sys.i2c = 1;
			break;
		case '4':
			i2c_state = 4;
			sys.i2c = 1;
			break;
		case '5':
			i2c_state = 5;
			sys.i2c = 1;
			break;
		case '6':
			i2c_state = 6;
			sys.i2c = 1;
			break;
#endif
		case 'B':
			state = 0;
			sys.prompt = 1;
			break;
		default:
			break;
		}
		break;
	case CAN:
		if (buff_receive[0] == '1') {
			CAN_TxMsg(&hcan1, 32, complete, 1);
			sys.can = 1;
		} else if (buff_receive[0] == '2') {
			CAN_TxMsg(&hcan2, 33, complete, 1);
			sys.can = 1;
		} else if (buff_receive[0] == 'B') {
			state = 0;
			sys.prompt = 1;
		}

		break;
	case CHANGEUART:
		if (buff_receive[0] == 'C') {
			state = UART;
			sys.transmit = 1;
		}
		break;

	case USB:
		if(buff_receive[0] == 'B') {
			sys.usb = 0;
			state = 0;
			sys.transmit = 1;
		}
		else if (buff_receive[0] == 'C'){
			sys.usb = 1;
		}
		break;

	case ROBOMASTER:
		if (buff_receive[0] == '1'){
			FLAG = C610;
			RBMS_Init(&rbms1, &hcan1, RBMS_1234);
			RBMS_Config(&rbms1, RBMS1, C610, 1);
			RBMS_Config(&rbms1, RBMS2, C610, 1);
			RBMS_Config(&rbms1, RBMS3, C610, 1);
			RBMS_Config(&rbms1, RBMS4, C610, 1);
			RBMS_PID_Init(&rbms1);
			RBMS_Set_Control_Mode(&rbms1, RBMS1, VELOCITY);
			RBMS_Set_Control_Mode(&rbms1, RBMS2, VELOCITY);
			RBMS_Set_Control_Mode(&rbms1, RBMS3, VELOCITY);
			RBMS_Set_Control_Mode(&rbms1, RBMS4, VELOCITY);
			RBMS_Set_Target_Velocity(&rbms1, RBMS1, 150);
			RBMS_Set_Target_Velocity(&rbms1, RBMS2, 150);
			RBMS_Set_Target_Velocity(&rbms1, RBMS3, 150);
			RBMS_Set_Target_Velocity(&rbms1, RBMS4, 150);
			sprintf(buff_transmit,"Type C to change motor type or B to exit\n");
			UART_Send();

		}
		if (buff_receive[0] == '2'){
			FLAG = C620;
			RBMS_Init(&rbms1, &hcan1, RBMS_1234);
			RBMS_Config(&rbms1, RBMS1, C620, 1);
			RBMS_Config(&rbms1, RBMS2, C620, 1);
			RBMS_Config(&rbms1, RBMS3, C620, 1);
			RBMS_Config(&rbms1, RBMS4, C620, 1);
			RBMS_PID_Init(&rbms1);
			RBMS_Set_Control_Mode(&rbms1, RBMS1, VELOCITY);
			RBMS_Set_Control_Mode(&rbms1, RBMS2, VELOCITY);
			RBMS_Set_Control_Mode(&rbms1, RBMS3, VELOCITY);
			RBMS_Set_Control_Mode(&rbms1, RBMS4, VELOCITY);
			RBMS_Set_Target_Velocity(&rbms1, RBMS1, 150);
			RBMS_Set_Target_Velocity(&rbms1, RBMS2, 150);
			RBMS_Set_Target_Velocity(&rbms1, RBMS3, 150);
			RBMS_Set_Target_Velocity(&rbms1, RBMS4, 150);
			sprintf(buff_transmit,"Type C to change motor type or B to exit\n");
			UART_Send();

		}
		if(buff_receive[0] == 'C') {
			RBMS_Set_Target_Velocity(&rbms1, RBMS1, 0);
			RBMS_Set_Target_Velocity(&rbms1, RBMS2, 0);
			RBMS_Set_Target_Velocity(&rbms1, RBMS3, 0);
			RBMS_Set_Target_Velocity(&rbms1, RBMS4, 0);
			state = 9;
			sys.transmit = 1;
		}
		if(buff_receive[0] == 'B') {
			RBMS_Set_Target_Velocity(&rbms1, RBMS1, 0);
			RBMS_Set_Target_Velocity(&rbms1, RBMS2, 0);
			RBMS_Set_Target_Velocity(&rbms1, RBMS3, 0);
			RBMS_Set_Target_Velocity(&rbms1, RBMS4, 0);
			state = 0;
			sys.transmit = 1;
		}
   }
}

void UART_Send(void){
	HAL_UART_Transmit(&Uarts, (uint8_t*)buff_transmit, strlen(buff_transmit), 1000);
	HAL_Delay(100);
}

void Prompt(void){
	if (sys.transmit == 1 || sys.prompt == 1) {
				switch (state) {
				case MAIN:
					sprintf(buff_transmit, "Welcome to board testing program.\n");
					UART_Send();
				#ifdef newpin
					sprintf(buff_transmit,"The board being tested is: Mainboard V3\n");
					UART_Send();
				#endif
				#ifdef mainboard
					sprintf(buff_transmit,"The board being tested is: Mainboard V2\n");
					UART_Send();
				#endif
				#ifdef newboard
					sprintf(buff_transmit,"The board being tested is: Mainboard V3.3\n");
					UART_Send();
					#endif
					sprintf(buff_transmit,
							"Please select what you like to test \n");
					UART_Send();
					sprintf(buff_transmit, "1) GPIO\n");
					UART_Send();
					sprintf(buff_transmit, "2) HSPM\n");
					UART_Send();
					sprintf(buff_transmit, "3) Encoder\n");
					UART_Send();
					sprintf(buff_transmit, "4) UART\n");
					UART_Send();
					sprintf(buff_transmit, "5) I2C\n");
					UART_Send();
					sprintf(buff_transmit, "6) CAN\n");
					UART_Send();
					sprintf(buff_transmit, "7) USB\n");
					UART_Send();
					sprintf(buff_transmit, "8) Robomaster\n");
					UART_Send();
					sprintf(buff_transmit,
							"Type B to exit after entering a test mode\n");
					UART_Send();
					sys.prompt = 0;
					break;
				case GPIOS:
					sprintf(buff_transmit, "PIN: ");
					UART_Send();
					if(PB1) {sprintf(buff_transmit, "");} else{ sprintf(buff_transmit, "B1 ");};
					PB2 ? strcat(buff_transmit, "") : strcat(buff_transmit, "B2 ");
#ifdef newboard
					PB3 ? strcat(buff_transmit, "") : strcat(buff_transmit, "B3 ");
#endif
					IP1 ? strcat(buff_transmit, "") : strcat(buff_transmit, "D1 ");
					IP2 ? strcat(buff_transmit, "") : strcat(buff_transmit, "D2 ");
					IP3 ? strcat(buff_transmit, "") : strcat(buff_transmit, "D3 ");
					IP4 ? strcat(buff_transmit, "") : strcat(buff_transmit, "D4 ");
					IP5 ? strcat(buff_transmit, "") : strcat(buff_transmit, "D5 ");
					IP6 ? strcat(buff_transmit, "") : strcat(buff_transmit, "D6 ");
#ifdef newpin
					IP7 ? strcat(buff_transmit, "") : strcat(buff_transmit, "A1 ");
					IP8 ? strcat(buff_transmit, "") : strcat(buff_transmit, "A2");
#else

					IP7 ? strcat(buff_transmit, "") : strcat(buff_transmit, "D7 ");
					IP8 ? strcat(buff_transmit, "") : strcat(buff_transmit, "D8 ");
					IP9 ? strcat(buff_transmit, "") : strcat(buff_transmit, "D9 ");
					IP10 ? strcat(buff_transmit, "") : strcat(buff_transmit, "D10 ");
					IP11 ? strcat(buff_transmit, "") : strcat(buff_transmit, "D11 ");
					IP12 ? strcat(buff_transmit, "") : strcat(buff_transmit, "D12 ");
#endif
#ifdef mainboard
					IP13 ? strcat(buff_transmit, "") : strcat(buff_transmit, "D13 ");
					IP14 ? strcat(buff_transmit, "") : strcat(buff_transmit, "D14 ");
					IP15 ? strcat(buff_transmit, "") : strcat(buff_transmit, "D15 ");
#endif
#ifndef newpin
					IP16 ? strcat(buff_transmit, "") : strcat(buff_transmit, "A0 ");
					IP17 ? strcat(buff_transmit, "") : strcat(buff_transmit, "A1 ");
					IP18 ? strcat(buff_transmit, "") : strcat(buff_transmit, "A2 ");
					IP19 ? strcat(buff_transmit, "") : strcat(buff_transmit, "A3 ");
					IP20 ? strcat(buff_transmit, "") : strcat(buff_transmit, "A4 ");
					IP21 ? strcat(buff_transmit, "") : strcat(buff_transmit, "A5");
#endif
#ifdef mainboard
					Mux1 ? strcat(buff_transmit, "") : strcat(buff_transmit, "M1 ");
					Mux2 ? strcat(buff_transmit, "") : strcat(buff_transmit, "M2 ");
					Mux3 ? strcat(buff_transmit, "") : strcat(buff_transmit, "M3 ");
					Mux4 ? strcat(buff_transmit, "") : strcat(buff_transmit, "M4 ");
					Mux5 ? strcat(buff_transmit, "") : strcat(buff_transmit, "M5 ");
					Mux6 ? strcat(buff_transmit, "") : strcat(buff_transmit, "M6 ");
					Mux7 ? strcat(buff_transmit, "") : strcat(buff_transmit, "M7 ");
					Mux8 ? strcat(buff_transmit, "") : strcat(buff_transmit, "M8");
#endif
					UART_Send();
					sprintf(buff_transmit,"\n");
					UART_Send();
					break;
				case HSPM:
					led3^=1;
					sprintf(buff_transmit, "HSPM mode\n");
					UART_Send();
					sprintf(buff_transmit, "Type C to change motor direction\n");
					UART_Send();
					sprintf(buff_transmit, "Type B to exit\n");
					UART_Send();
					WriteBDC(&BDC1, 10000);
					WriteBDC(&BDC2, 10000);
					WriteBDC(&BDC3, 10000);
					WriteBDC(&BDC4, 10000);
					WriteBDC(&BDC5, 10000);
					WriteBDC(&BDC6, 10000);
					WriteBDC(&BDC7, 10000);
					WriteBDC(&BDC8, 10000);
					break;
				case ENCODER:
	#ifdef newpin
					sprintf(buff_transmit, "QEI1: %d, QEI3: %d, QEI4: %d, QEI8: %d\n",QEIRead(QEI1),QEIRead(QEI3), QEIRead(QEI4), QEIRead(QEI6));
	#else
					sprintf(buff_transmit, "QEI1: %d, QEI4: %d, QEI8: %d\n",QEIRead(QEI1), QEIRead(QEI4), QEIRead(QEI6));
	#endif
					UART_Send();
					break;
				case UART:
					sprintf(buff_transmit, "Please select UART\n");
					UART_Send();
	#ifdef newpin
					sprintf(buff_transmit, "1) UART1\n");
					UART_Send();
	#endif
					sprintf(buff_transmit, "2) UART2\n");
					UART_Send();
					sprintf(buff_transmit, "3) UART3\n");
					UART_Send();
					sprintf(buff_transmit, "4) UART4\n");
					UART_Send();
					sprintf(buff_transmit, "5) UART5\n");
					UART_Send();
					sprintf(buff_transmit, "Type B to exit\n");
					UART_Send();
					break;
				case I2C:
					sprintf(buff_transmit, "Please select I2C test\n");
					UART_Send();
					sprintf(buff_transmit, "1) I2C1 to I2C2\n");
					UART_Send();
					sprintf(buff_transmit, "2) I2C2 to I2C1\n");
					UART_Send();
#ifndef newboard
					sprintf(buff_transmit, "3) I2C1 to I2C3\n");
					UART_Send();
					sprintf(buff_transmit, "4) I2C3 to I2C1\n");
					UART_Send();
					sprintf(buff_transmit, "5) I2C2 to I2C3\n");
					UART_Send();
					sprintf(buff_transmit, "6) I2C3 to I2C2\n");
					UART_Send();
#endif
					sprintf(buff_transmit, "Type B to exit\n");
					UART_Send();
					break;
				case CAN:
					sprintf(buff_transmit, "Please select CAN test\n");
					UART_Send();
					sprintf(buff_transmit, "1) CAN1 to CAN2\n");
					UART_Send();
					sprintf(buff_transmit, "2) CAN2 to CAN1\n");
					UART_Send();
					sprintf(buff_transmit, "Type B to exit\n");
					UART_Send();

					break;
				case CHANGEUART:
					sprintf(buff_transmit, "Type C to continue\n");
					UART_Send();
					break;

				case USB:
					sprintf(buff_transmit, "On your PC check Serial Studio for message\n");
					UART_Send();
					sprintf(buff_transmit, "Then type C to continue\n");
					UART_Send();
					sprintf(buff_transmit, "Type B to exit\n");
					UART_Send();
					break;

				case ROBOMASTER:
					sprintf(buff_transmit, "Please test the CAN bus\n");
					UART_Send();
					sprintf(buff_transmit, "before attempting the Robomaster test\n");
					UART_Send();
					sprintf(buff_transmit, "Select Motor Type:\n");
					UART_Send();
					sprintf(buff_transmit, "1) C610\n");
					UART_Send();
					sprintf(buff_transmit, "2) C620\n");
					UART_Send();
					sprintf(buff_transmit, "Type B to exit\n");
					UART_Send();

				}

				sys.transmit = 0;
			}
}

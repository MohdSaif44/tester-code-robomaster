/*******************************************************************************
 * Title   : common.h
 * Author  : Anas Amer
 * Version : 1.0
 * Date    : 12/10
 *******************************************************************************
 * Description: Sensor and function definitions
 *
 * Version History:
 *  1.0 - converted to hal library
 *
 * Bugs:
 *
 ******************************************************************************/

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "adapter.h"
#include "cmsis_os.h"
#include "usb_device.h"


#define IP1  	HAL_GPIO_ReadPin(IP1_PIN)
#define IP2  	HAL_GPIO_ReadPin(IP2_PIN)
#define IP3  	HAL_GPIO_ReadPin(IP3_PIN)
#define IP4		HAL_GPIO_ReadPin(IP4_PIN)
#define IP5 	HAL_GPIO_ReadPin(IP5_PIN)
#define IP6 	HAL_GPIO_ReadPin(IP6_PIN)


#ifdef newpin
#define IP7		HAL_GPIO_ReadPin(IP7_Analog1_PIN)
#define IP8		HAL_GPIO_ReadPin(IP8_Analog2_PIN)
#endif

#ifdef mainboard
#define IP7		HAL_GPIO_ReadPin(IP7_PIN)
#define IP8	 	HAL_GPIO_ReadPin(IP8_PIN)
#define IP9  	HAL_GPIO_ReadPin(IP9_PIN)
#define IP10    HAL_GPIO_ReadPin(IP10_PIN)
#define IP11  	HAL_GPIO_ReadPin(IP11_PIN)
#define IP12 	HAL_GPIO_ReadPin(IP12_PIN)
#define IP13  	HAL_GPIO_ReadPin(IP13_PIN)
#define IP14 	HAL_GPIO_ReadPin(IP14_PIN)
#define IP15	HAL_GPIO_ReadPin(IP15_PIN)

//ANALOG PIN//
#define IP16	HAL_GPIO_ReadPin(IP16_Analog1_PIN)
#define IP17	HAL_GPIO_ReadPin(IP17_Analog2_PIN)
#define	IP18 	HAL_GPIO_ReadPin(IP18_Analog3_PIN)
#define IP19	HAL_GPIO_ReadPin(IP19_Analog4_PIN)
#define IP20	HAL_GPIO_ReadPin(IP20_Analog5_PIN)
#define IP21	HAL_GPIO_ReadPin(IP21_Analog6_PIN)
#endif

#ifdef newboard
#define IP7		HAL_GPIO_ReadPin(IP7_PIN)
#define IP8	 	HAL_GPIO_ReadPin(IP8_PIN)
#define IP9  	HAL_GPIO_ReadPin(IP9_PIN)
#define IP10    HAL_GPIO_ReadPin(IP10_PIN)
#define IP11  	HAL_GPIO_ReadPin(IP11_PIN)
#define IP12 	HAL_GPIO_ReadPin(IP12_PIN)

//ANALOG PIN//
#define IP16	HAL_GPIO_ReadPin(IP16_Analog1_PIN)
#define IP17	HAL_GPIO_ReadPin(IP17_Analog2_PIN)
#define	IP18 	HAL_GPIO_ReadPin(IP18_Analog3_PIN)
#define IP19	HAL_GPIO_ReadPin(IP19_Analog4_PIN)
#define IP20	HAL_GPIO_ReadPin(IP20_Analog5_PIN)
#define IP21	HAL_GPIO_ReadPin(IP21_Analog6_PIN)
#endif

#define Mux1	MUX.mux_data.bit0
#define Mux2	MUX.mux_data.bit1
#define Mux3	MUX.mux_data.bit2
#define Mux4	MUX.mux_data.bit3
#define Mux5	MUX.mux_data.bit4
#define Mux6	MUX.mux_data.bit5
#define Mux7	MUX.mux_data.bit6
#define Mux8	MUX.mux_data.bit7

float fxPos,fyPos,fyaw,frad;


osThreadId_t MainTaskHandle;
osThreadId_t SecondTaskHandle;
osThreadId_t ThirdTaskHandle;
osThreadId_t EmergencyTaskHandle;
osThreadId_t CalculationTaskHandle;
//osThreadId_t TuneTaskHandle;
//osThreadId_t CalculationTaskHandle;
//osThreadId_t LaserNavigateTaskHandle;
//osThreadId_t SecondaryTaskHandle;
//osThreadId_t FlywheelPitchPIDTaskHandle;
//osThreadId_t FlywheelYawPIDTaskHandle;
//osThreadId_t TestTaskHandle;

osSemaphoreId_t TuneSemaphore, CalcSemaphore;

typedef union{
	uint32_t flags;
	struct{
		//Least significant 16 bits can be cleared all at once by
		//sys.flags = 0 for example during emergency
		unsigned transmit      :1;
		unsigned digital      :1;
		unsigned analog    :1;
		unsigned mux		  :1;
		unsigned pwm       :1;
		unsigned error  :1;
		unsigned uart      :1;
		unsigned i2c		  :1;
		unsigned prompt      :1;
		unsigned flag10     :2;
		unsigned can    :1;
		unsigned usb       :1;
		unsigned flag12       :1;
		unsigned flag13       :1;
		unsigned flag14       :1;

		//Most significant 16 bits are not clear

		unsigned flag16		  :1;
		unsigned flag17		  :1;
		unsigned flag18		  :1;
		unsigned flag19		  :1;
		unsigned flag20		  :1;
		unsigned flag21		  :1;
		unsigned flag22		  :1;
		unsigned flag23		  :1;
		unsigned flag24       :1;
		unsigned flag25       :1;
		unsigned flag26	      :1;
		unsigned flag27		  :1;
		unsigned flag28		  :1;
		unsigned flag29		  :1;
		unsigned flag30		  :1;
		unsigned flag31		  :1;

		//A flag can use more than 1 bit
		//Example : Combine flag30 and flag31
		//unsigned flag29     :1;
		//unsigned flag30     :2;
		//the value of sys.flag30 range from 0 to 3 then overflow to 0 again and vice versa
		//the value of flag29 is not affected when flag30 overflow
	};
}sys_t;

volatile sys_t sys;

void RNS_config(CAN_HandleTypeDef* hcanx);

void set(void);
void mode1(void);
void mode2(void);

void setPP_points(void);

void enq(void);

#ifdef SWERVE
#define NUM_INT_UPDATE		2
#define NUM_FLOAT_UPDATE	11
#else
#define NUM_INT_UPDATE		2
#define NUM_FLOAT_UPDATE	5
#endif
void move(float x, float y, float w,float rad);
void ILI9341_Init_List(void);
void ILI9341_Update_List(void);
void Testboard_Handler(void);
void UART_Send(void);
void Prompt(void);

#endif /* INC_COMMON_H_ */

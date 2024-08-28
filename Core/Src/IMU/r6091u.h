/*
 * r6091u.c
 *
 *  Created on: Aug 5, 2022
 *      Author: wai
 */

#ifndef SRC_R6091U_R6091U_H_
#define SRC_R6091U_R6091U_H_

#include "../BIOS/bios.h"
#include "../I2C/i2c.h"

typedef struct {

	I2C_HandleTypeDef *hi2cimu;
	UART_HandleTypeDef* huartx;
	volatile uint8_t Buffer[20];
	uint8_t index;
	uint8_t State;
	volatile float roll;
	volatile float roll_rate;
	volatile float pitch;
	volatile float pitch_rate;
	volatile float yaw;
	volatile float yaw_rate;
	volatile float x_acc;
	volatile float y_acc;
	volatile float z_acc;
	volatile int8_t turn_no;
	volatile uint8_t checksum;
	float prev_yaw;
	float yaw_constant;
	float real_z;
	float real_zrad;
	float offset;
}R6091U_t;

void IMU_Init(R6091U_t* IMU,UART_HandleTypeDef* huartx);
void IMU_InitI2C(R6091U_t* IMU, I2C_HandleTypeDef *hi2c);
void IMUConnectI2C(R6091U_t* IMU);
void IMU_Handler(R6091U_t* IMU);
#endif /* SRC_R6091U_R6091U_H_ */

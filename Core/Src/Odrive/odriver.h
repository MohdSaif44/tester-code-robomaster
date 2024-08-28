/*
 * odriver.h
 *
 *  Created on: Oct 6, 2022
 *      Author: heheibhoi
 */
#ifndef SRC_ODRIVE_ODRIVER_H_
#define SRC_ODRIVE_ODRIVER_H_

#include "../CAN/can.h"

#define BRAKE_PERIOD 500

char odrivefaultcode[128];

enum{
	ODRIVE1 = 0x10,
	ODRIVE2,
	ODRIVE3,
	ODRIVE4,
	ODRIVELAST,
	ODRIVEPOS1,
	ODRIVEPOS2,
	ODRIVEPOS3,
	ODRIVEPOS4,
	ODRIVEPOSLAST = 0x1F,
};

typedef enum{

	BUS_VOLTAGE_AND_CURRENT,
	TURN_COUNT_AND_VELOCITY,
	MOTOR_AND_INVERTER_TEMPERATURE,
	SHADOW_COUNT,
	IQ_VALUE,
	ODRIVE_ERROR_CODE,

}Feedback;

typedef enum{

	ENCODER_MODE_INCREMENTAL,
	ENCODER_MODE_SPI_ABS_AMS,
	ENCODER_MODE_HALL,

}Encoder_mode;

enum Commands{
	GET_ERROR = 3,
	FULL_MOTOR_CALIBRATION,
	SET_AXIS_STATE = 7,
	GET_ENCODER_ESTIMATE = 9,
	GET_SHADOW_COUNT,					//not in datasheet
	SET_CONTROLLER_MODES,
	SET_INPUT_POS,
	SET_INPUT_VEL,
	SET_INPUT_TORQUE,
	SET_LIMITS,							//set current limit and rps limit, rps byte 0-3, current 4-7
	SET_TRAP_TRAJ_VEL_LIMIT = 17,
	SET_TRAP_TRAJ_ACCEL_LIMIT,			//accel limit at lower 4 byte, decel limit at lower 4 byte
	SET_TRAP_TRAJ_INERTIA,
	GET_IQ,
	GET_TEMPERATURE,
	REBOOT_ODRIVE,
	GET_BUS_VOLTAGE_AND_CURRENT,
	CLEAR_ERROR,
	SET_ABSOLUTE_POSITION,
};						//basic commands

typedef enum {

	VOLTAGE_CONTROL,
	TORQUE_CONTROL,
	VELOCITY_CONTROL,
	POSITION_CONTROL

}ControlMode;			//control mode state

typedef enum {
	UNDEFINE_STATE,
	IDLE_STATE,
	STARTUP_SEQUENCE,
	FULL_CALIBRATION_SEQUENCE,
	MOTOR_CALIBRATION,
	ENCODER_INDEX_SEARCH = 6,
	ENCODER_OFFSET_CALIBRATION,
	CLOSED_LOOP_CONTROL,
	LOCKIN_SPIN,
	ENCODER_DIR_FIND,
	HOMING,
	ENCODER_HALL_POLARITY_CALIBRATION,
	ENCODER_HALL_PHASE_CALIBRATION,

}AxisRequestedState;	//axis requested state

typedef enum {

	PASSTHROUGH = 1,
	VEL_RAMP,		//vel ctrl
	POS_FILTER,		//pos ctrl
	TRAPEZOIDAL_TRAJECTORY = 5,	//pos ctrl
	TORQUE_RAMP,	//tor ctrl
	MIRROR,			//pos ctrl, 2 motor

}InputMode;				//input mode

typedef enum{
	INITIALIZING 				= 0x01,
	SYSTEM_LEVEL 				= 0x02,	//firmware bug / system error: memory corruption, stack overflow, frozen thread
	TIMING_ERROR 				= 0x04,
	MISSING_ESTIMATE			= 0x08,	//enc not calibrated / Abs pos ctrl used b4 axis homed / enc misbehaving or dc
	BAD_CONFIG 					= 0x10,
	DRV_FAULT					= 0x20,
	MISSING_INPUT				= 0x40,
	DC_BUS_OVER_VOLTAGE			= 0x100,
	DC_BUS_UNDER_VOLTAGE		= 0x200,
	DC_BUS_OVER_CURRENT			= 0x400,
	DC_BUS_OVER_REGEN_CURRENT	= 0x800,
	CURRENT_LIMIT_VIOLATION		= 0x1000,
	MOTOR_OVER_TEMP				= 0x2000,
	INVERTER_OVER_TEMP			= 0x4000,
	VELOCITY_LIMIT_VIOLATION	= 0x8000,
	POSITION_LIMIT_VIOLATION	= 0x10000,
	WATCHDOG_TIMER_EXPIRED		= 0x1000000,
	ESTOP_REQUESTED				= 0x2000000,
	SPINOUT_DETECTED			= 0x4000000,
	OTHER_DEVICE_FAILED			= 0x8000000,
	CALIBRATION_ERROR			= 0x40000000,
}error_code;

typedef struct {

	float velocity;
	float encoder;
	float round_per_second;
	float inverter_temperature;
	float motor_temperature;
	float bus_voltage;
	float bus_current;
	float iq_setpoint;
	float iq_measured;
	float angle;
	error_code error;

}Odrv_feedback;

typedef struct {
	CAN_RxHeaderTypeDef RXmsg;
	uint8_t Data[8];
	uint16_t id;
} odrvmsg;

typedef struct{

	CAN_HandleTypeDef* hcanx;
	float wheel_diameter;
	float target;
	volatile uint8_t used;
	volatile uint8_t busy;
	volatile uint8_t stop;
	volatile uint8_t error;
	volatile uint8_t hand_brake;
	uint16_t Instance;
	uint32_t hand_brake_start;
	InputMode input_mode;
	ControlMode control_mode;
	AxisRequestedState current_state;
	Odrv_feedback feedback;
	Encoder_mode encoder_mode;

}Odrv_t;

odrvmsg Odrvmsg;
extern const char* Odriveerror_to_string(error_code fault);

void OdriveInit(Odrv_t* odrive,CAN_HandleTypeDef* hcanx,uint16_t axis_id,Encoder_mode encoder_type);

void OdriveTurn(Odrv_t* odrive,float count_num,InputMode input_mode);
void OdriveTurnCountInertia(Odrv_t* odrive,float count_num,float inertia);
void OdriveAbsoluteTurn(Odrv_t* odrive,float count_num,InputMode input_mode);

void OdriveVelocity(Odrv_t* odrive,float target_velocity,uint8_t input_mode);

void OdriveTorque(Odrv_t* odrive,float target_torque,uint8_t input_mode);

void OdriveStop(Odrv_t* odrive);
void OdriveHandBrake( Odrv_t* odrive);
void OdriveRelease(Odrv_t* odrive);
void OdriveRestart(Odrv_t* odrive);
void OdriveClearError(Odrv_t* odrive);

void OdriveSetControlMode(Odrv_t* odrive,uint8_t control_mode);
void OdriveSetInputMode(Odrv_t* odrive,uint8_t input_mode);
void OdriveSetRPSandCurrentMax(Odrv_t* odrive,float rps_lim,float current_lim);
void OdriveSetControlInputMode(Odrv_t* odrive,ControlMode requested_control_mode,InputMode requested_input_mode);
void OdriveSetAbsolutePosition(Odrv_t* odrive);
void OdriveSetInertia(Odrv_t* odrive,float inertia);

void decode_Odrive(Odrv_t* odrive);
void OdriveEnquire(Odrv_t* odrive,uint8_t feedback);
void OdriveGetTemperatureFeedback(Odrv_t* odrive);
void OdriveGetEncoderFeedback(Odrv_t* odrive);
void OdriveGetShadowCountCPR(Odrv_t* odrive);
void OdriveGetBusVoltageAndCurrent(Odrv_t* odrive);
void OdriveRPStoVelocity(Odrv_t* odrive);
void OdriveAngle(Odrv_t* odrive, float angle);
#endif /* SRC_ODRIVE_ODRIVER_H_ */

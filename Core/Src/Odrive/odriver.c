/*
 * odriver.c
 *
 *  Created on: Oct 6, 2022
 *      Author: heheibhoi
 */

#include "odriver.h"

/*TODO before testing
 * 1)if disable handbrake, try to update current_turn before changing control mode
 *
 * 2)if using absolute encoder,init current_turn
 *
 * 3)if enable handbrake, update current turn after switching control modes
 *
 * 4)if stop, set velocity to 0, if handbrake, update pos and  set to position control
 *
 * 5)reduce data in structures
 *
 *
 * */

/*
 * CAN id:
 * Upper 6 bits - Node ID - max 0x3F
 * Lower 5 bits - Command ID - max 0x1F
 */

/**********Private Function*************/

void OdriveUpdatePos(Odrv_t* odrive);
void OdriveFeedBackCorrection(Odrv_t* odrive);
void OdriveSetAxisRequestedState(Odrv_t* odrive,uint8_t AxisRequested_state);
void OdriveSendCAN(Odrv_t* odrive,uint8_t command,uint8_t* buffer);
void OdriveSendRTRCAN(Odrv_t* odrive,uint8_t command);

/**************************************/

/*
 * Function Name		: OdriveInit
 * Function Description : This function is called to init odrive structure
 * Function Arguments	: odrive		structure pointer to Odrv_t
 * 						  hcanx			can used for communication with odrive
 * 						  axis_id		can id for odrive, restricted to 6 bits (0x00 - 0x3F)
 * Function Return		: None
 * Function Example		: OdriveInit(&Odrv1, &hcan1, ODRIVE1, VEL_RAMP,VELOCITY_CONTROL);
 */
void OdriveInit(Odrv_t* odrive, CAN_HandleTypeDef* hcanx, uint16_t axis_id,Encoder_mode encoder_type)
{
	odrive->Instance = axis_id;
	odrive->hcanx = hcanx;
	odrive->control_mode = POSITION_CONTROL;
	odrive->input_mode = PASSTHROUGH;
	odrive->stop = true;

	odrive->used = 1;
	if(encoder_type == ENCODER_MODE_SPI_ABS_AMS)
		OdriveUpdatePos(odrive);
}

/*
 * Function Name		: OdriveSetControlMode
 * Function Description : This function is called to change control mode of odrive
 * Function Arguments	: odrive		structure pointer to Odrv_t
 * 						  requested_control_mode
 * Function Return		: None
 * Function Example		: OdriveSetControlMode(&odrive, POSITION_CONTROL);
 */
void OdriveSetControlMode(Odrv_t* odrive, ControlMode requested_control_mode)
{
	uint8_t TxBuffer[8] = {0};
	//
	//	if(odrive->control_mode==requested_control_mode)
	//		return;

	TxBuffer[0] = requested_control_mode;
	odrive->control_mode = requested_control_mode;

	OdriveSendCAN(odrive, SET_CONTROLLER_MODES, TxBuffer);
}

/*
 * Function Name		: OdriveSetInputMode
 * Function Description : This function is called to change input mode of odrive
 * Function Arguments	: odrive		structure pointer to Odrv_t
 * 						  requested_input_mode
 * Function Return		: None
 * Function Example		: OdriveSetInputMode(&odrive, PASSTHROUGH);
 */
void OdriveSetInputMode(Odrv_t* odrive,InputMode requested_input_mode)
{
	uint8_t TxBuffer[8] = {0};
	//
	//	if(odrive->input_mode==requested_input_mode)
	//		return;

	TxBuffer[4] = requested_input_mode;

	odrive->input_mode = requested_input_mode;

	OdriveSendCAN(odrive, SET_CONTROLLER_MODES, TxBuffer);
}

/*
 * Function Name		: OdriveSetControlInputMode
 * Function Description : This function is called to change input mode and control of odrive
 * Function Arguments	: odrive		structure pointer to Odrv_t
 *                        requested_control_mode
 * 						  requested_input_mode
 * Function Return		: None
 * Function Example		: OdriveSetControlInputMode(&odrive, VELOCITY_CONTROL, PASSTHROUGH);
 */

void OdriveSetControlInputMode(Odrv_t* odrive,ControlMode requested_control_mode,InputMode requested_input_mode)
{
	uint8_t TxBuffer[8] = {0};
	//
	//	if(odrive->control_mode==requested_control_mode&&odrive->input_mode==requested_input_mode)
	//		return;

	TxBuffer[0] = requested_control_mode;
	TxBuffer[4] = requested_input_mode;

	odrive->control_mode = requested_control_mode;
	odrive->input_mode = requested_input_mode;

	OdriveSendCAN(odrive, SET_CONTROLLER_MODES, TxBuffer);
}

/*
 * Function Name		: OdriveTurn
 * Function Description : This function is called to turn odrive motor n number of turns from current position, float number is also acceptable
 * Function Arguments	: odrive		structure pointer to Odrv_t
 *                        count_num		number of turns to count
 * 						  input_mode	requested_input_mode
 * Function Return		: None
 * Function Example		: OdriveTurn(&odrive, 1.3, TRAPEZOIDAL_TRAJECTORY);
 */

void OdriveTurn(Odrv_t* odrive,float count_num,InputMode input_mode)//turn number of turns
{
	odrive->stop = odrive->hand_brake = 0;

	uint8_t TxBuffer[8] = {0};

	if(input_mode != PASSTHROUGH || input_mode != POS_FILTER || input_mode != TRAPEZOIDAL_TRAJECTORY)
		input_mode = TRAPEZOIDAL_TRAJECTORY;

	OdriveEnquire(odrive, TURN_COUNT_AND_VELOCITY);

	while(odrive->busy);

	float target = odrive->feedback.encoder + count_num;
	memcpy(&TxBuffer[0], &target, 4);
	OdriveSetControlInputMode(odrive, POSITION_CONTROL, input_mode);

	if(odrive->current_state != CLOSED_LOOP_CONTROL)
		OdriveSetAxisRequestedState(odrive, CLOSED_LOOP_CONTROL);

	OdriveSendCAN(odrive, SET_INPUT_POS, TxBuffer);
	odrive->stop = odrive->hand_brake = 0;
}

/*
 * Function Name		: OdriveFeedBackCorrection
 * Function Description : This function is called to avoid odrive overspeed error that occurs when user change from velocity control mode to position control mode
 * Function Arguments	: odrive		structure pointer to Odrv_t
 * Function Return		: None
 * Function Example		: OdriveFeedBackCorrection(&odrive);
 */

void OdriveFeedBackCorrection(Odrv_t* odrive)//turn number of turns
{
	uint8_t TxBuffer[8] = {0};

	//	OdriveStop(odrive);
	OdriveEnquire(odrive, TURN_COUNT_AND_VELOCITY);

	while(odrive->busy);

	memcpy(&TxBuffer[0], &odrive->feedback.encoder, 4);
	OdriveSetControlInputMode(odrive, POSITION_CONTROL, PASSTHROUGH);	//only passthrough works, trap traj and pos filter will induce overspeed
//	HAL_Delay(10);
	OdriveSendCAN(odrive, SET_INPUT_POS, TxBuffer);
}


/*
 * Function Name		: OdriveUpdatePos
 * Function Description : This function is called to avoid odrive overspeed error that occurs when user change from velocity control mode to position control mode
 * Function Arguments	: odrive		structure pointer to Odrv_t
 * Function Return		: None
 * Function Example		: OdriveUpdatePos(&odrive);
 */
void OdriveUpdatePos(Odrv_t* odrive)
{
	uint8_t TxBuffer[8] = {0};

	OdriveEnquire(odrive,TURN_COUNT_AND_VELOCITY);

	while(odrive->busy);

	memcpy(&TxBuffer[0], &odrive->feedback.encoder, 4);

	OdriveSendCAN(odrive,SET_INPUT_POS,TxBuffer);
}


/*
 * Function Name		: OdriveTurnCountInertia
 * Function Description : This function is called to turn with high initial load
 * Function Arguments	: odrive		structure pointer to Odrv_t
 *                        count_num		turn count number
 *                        inertia
 *
 * Function Return		: None
 * Function Example		: OdriveTurnCountInertia(&odrive,1.1,0.5);
 */
void OdriveTurnCountInertia(Odrv_t* odrive,float count_num,float inertia)
{
	uint8_t TxBuffer[8] = {0};

	OdriveSetInertia(odrive, inertia);

	memcpy(&TxBuffer[0], &count_num, 4);

	OdriveSendCAN(odrive, SET_INPUT_POS, TxBuffer);

	OdriveSetControlInputMode(odrive, POSITION_CONTROL, TRAPEZOIDAL_TRAJECTORY);

	if(odrive->current_state != CLOSED_LOOP_CONTROL)
		OdriveSetAxisRequestedState(odrive, CLOSED_LOOP_CONTROL);

	odrive->stop = odrive->hand_brake = 0;
}


void OdriveHandBrake(Odrv_t* odrive)
{
	OdriveFeedBackCorrection(odrive);//go back to position control

	if(odrive->hand_brake)
	{
		if(HAL_GetTick() - odrive->hand_brake_start > BRAKE_PERIOD)//long period of position control consumes tons of power
		{
			OdriveStop(odrive);
			odrive->hand_brake = 0;
		}
		return;
	}

	odrive->hand_brake = 1;
	odrive->hand_brake_start = HAL_GetTick();
}


/*
 * Function Name		: OdriveVelocity
 * Function Description : This function is called to turn odrive motor in RPS
 * Function Arguments	: odrive			structure pointer to Odrv_t
 *                        target_velocity	target rps
 * 						  input_mode		requested_input_mode
 * Function Return		: None
 * Function Example		: OdriveVelocity(&odrive, 1.3, VEL_RAMP);
 */
void OdriveVelocity(Odrv_t* odrive,float target_velocity,InputMode input_mode)
{
	uint8_t TxBuffer[8] = {0};

	if(!target_velocity && odrive->stop == true)
		return;

	memcpy(&TxBuffer[0], &target_velocity, 4);

	OdriveSetControlInputMode(odrive, VELOCITY_CONTROL, input_mode);

	if(odrive->current_state != CLOSED_LOOP_CONTROL)
		OdriveSetAxisRequestedState(odrive, CLOSED_LOOP_CONTROL);

	OdriveSendCAN(odrive, SET_INPUT_VEL, TxBuffer);

	if(!target_velocity)
	{
		odrive->stop = 1;
		return;
	}

	odrive->stop = odrive->hand_brake = 0;
}

/*
 * Function Name		: OdriveStop
 * Function Description : This function is called to stop odrive with handbrake or without
 * Function Arguments	: odrive		structure pointer to Odrv_t
 * Function Return		: None
 * Function Example		: OdriveStop(&odrive);
 */
void OdriveStop(Odrv_t* odrive)
{
	if(odrive->stop == true)
		return;

	OdriveVelocity(odrive, 0.0, VEL_RAMP);

	odrive->stop = 1;
}

/*
 * Function Name		: OdriveRelease
 * Function Description : This function is called to release motor
 * Function Arguments	: odrive		structure pointer to Odrv_t
 * Function Return		: None
 * Function Example		: OdriveRelease(&odrive);
 */
void OdriveRelease(Odrv_t* odrive)
{
	OdriveSetAxisRequestedState(odrive, IDLE_STATE);
}

/*
 * Function Name		: OdriveTorque
 * Function Description : This function is called to turn odrive motor with torque control
 * Function Arguments	: odrive		structure pointer to Odrv_t
 *                        target_torque
 *                        input mode
 * Function Return		: None
 * Function Example		: OdriveTorque(&odrive, 1.3, TORQUE_RAMP);
 */
void OdriveTorque(Odrv_t* odrive, float target_torque, InputMode input_mode)//torque control
{
	odrive->stop = odrive->hand_brake = 0;

	uint8_t TxBuffer[8] = {0};

	if(input_mode != PASSTHROUGH || input_mode != TORQUE_RAMP)
		input_mode = TORQUE_RAMP;

	memcpy(&TxBuffer[0], &target_torque, sizeof(float));

	OdriveSetControlInputMode(odrive, TORQUE_CONTROL, input_mode);

	if(odrive->current_state != CLOSED_LOOP_CONTROL)
		OdriveSetAxisRequestedState(odrive, CLOSED_LOOP_CONTROL);

	OdriveSendCAN(odrive, SET_INPUT_TORQUE, TxBuffer);
}

/*
 * Function Name		: OdriveRestart
 * Function Description : This function is called to restart odrive
 * Function Arguments	: odrive		structure pointer to Odrv_t
 * Function Return		: None
 * Function Example		: OdriveRestart(&odrive);
 */
void OdriveRestart(Odrv_t* odrive)
{
	uint8_t TxBuffer[8] = {0};

	OdriveSendCAN(odrive, REBOOT_ODRIVE, TxBuffer);
}

/*
 * Function Name		: OdriveClearError
 * Function Description : This function is called to clear errors of odrive
 * Function Arguments	: odrive		structure pointer to Odrv_t
 * Function Return		: None
 * Function Example		:OdriveClearError(&odrive);
 */
void OdriveClearError(Odrv_t* odrive)
{
	uint8_t TxBuffer[8] = {0};

	OdriveSendCAN(odrive, CLEAR_ERROR, TxBuffer);
}
/*
 * Function Name		: OdriveSetAbsolutePosition
 * Function Description : This function is called to set absolute position of odrive (not useful)
 * Function Arguments	: odrive		structure pointer to Odrv_t
 * Function Return		: None
 * Function Example		: OdriveSetAbsolutePosition(&odrive);
 */
void OdriveSetAbsolutePosition(Odrv_t* odrive)
{
	uint8_t TxBuffer[8] = {0};

	OdriveSendCAN(odrive, SET_ABSOLUTE_POSITION, TxBuffer);
}


/*
 * Function Name		: OdriveEnquire
 * Function Description : This function is called to get odrive feedback
 * Function Arguments	: odrive		structure pointer to Odrv_t
 *                        feedback		the feedback required
 * Function Return		: None
 * Function Example		:OdriveEnquire(&odrive,TURN_COUNT_AND_VELOCITY);
 */
void OdriveEnquire(Odrv_t* odrive,Feedback feedback)
{
	odrive->busy = 1;
	if(feedback == BUS_VOLTAGE_AND_CURRENT)
		OdriveGetBusVoltageAndCurrent(odrive);
	else if(feedback == TURN_COUNT_AND_VELOCITY)
		OdriveGetEncoderFeedback(odrive);
	else if(feedback == MOTOR_AND_INVERTER_TEMPERATURE)
		OdriveGetTemperatureFeedback(odrive);
	else if(feedback == IQ_VALUE)
		OdriveSendRTRCAN(odrive, GET_IQ);
	else if(feedback == ODRIVE_ERROR_CODE)
		OdriveSendRTRCAN(odrive, GET_ERROR);
}


/*
 * Function Name		: decode_Odrive
 * Function Description : This function is called to decode odrive message sent back from o drive
 * Function Arguments	: odrive		structure pointer to Odrv_t
 * Function Return		: None
 * Function Example		: decode_Odrive(&odrive);
 */
void decode_Odrive(Odrv_t* odrive)
{
	float data = 0;
	uint16_t id = Odrvmsg.RXmsg.StdId;
	uint16_t mask = 0x01F;
	uint16_t command = id & mask;

	switch(command){
	case GET_ENCODER_ESTIMATE:
		memcpy(&data, &Odrvmsg.Data[0], sizeof(float));
		odrive->feedback.encoder = data;
		memcpy(&data, &Odrvmsg.Data[4], sizeof(float));
		odrive->feedback.round_per_second = data;
		odrive->feedback.angle = odrive->feedback.encoder * 360.0;
		break;

	case GET_TEMPERATURE:
		memcpy(&data, &Odrvmsg.Data[0], sizeof(float));
		odrive->feedback.inverter_temperature = data;
		memcpy(&data, &Odrvmsg.Data[4], sizeof(float));
		odrive->feedback.motor_temperature = data;
		break;

	case GET_BUS_VOLTAGE_AND_CURRENT:
		memcpy(&data, &Odrvmsg.Data[0], sizeof(float));
		odrive->feedback.bus_voltage = data;
		memcpy(&data, &Odrvmsg.Data[4], sizeof(float));
		odrive->feedback.bus_current = data;
		break;

	case GET_IQ:
		memcpy(&data, &Odrvmsg.Data[0], sizeof(float));
		odrive->feedback.iq_setpoint = data;
		memcpy(&data, &Odrvmsg.Data[4], sizeof(float));
		odrive->feedback.iq_measured = data;
		break;

	case GET_ERROR:
		memcpy(&data, &Odrvmsg.Data[0], 8);
		odrive->feedback.error = data;
		odrive->error = 1;
		sprintf(odrivefaultcode, "%d ", (int)odrive->Instance);
		strcat(odrivefaultcode, Odriveerror_to_string(odrive->feedback.error));
		strcat(odrivefaultcode, "\n");
		break;
	}
	odrive->busy = 0;

}

void OdriveSetAxisRequestedState(Odrv_t* odrive,AxisRequestedState AxisRequested_state)
{
	odrive->current_state = AxisRequested_state;

	uint8_t TxBuffer[8] = {0};

	memcpy(&TxBuffer[0], (uint32_t *)&AxisRequested_state, 4);

	OdriveSendCAN(odrive, SET_AXIS_STATE, TxBuffer);
}

/*
 * Function Name		: OdriveSetInertia
 * Function Description : This function is called to set inertia to odrive if a heavy load is attached
 * Function Arguments	: odrive		structure pointer to Odrv_t
 *                         inertia value, initial torque value in Nm needed to move motor with a load attached
 * Function Return		: None
 * Function Example		:OdrveSetInertia(&odrive,1.1);
 */
void OdriveSetInertia(Odrv_t* odrive,float inertia)
{
	uint8_t TxBuffer[8] = {0};

	memcpy(&TxBuffer[0], &inertia, 4);

	OdriveSendCAN(odrive, SET_TRAP_TRAJ_INERTIA, TxBuffer);
}
/*
 * Function Name		: OdriveAbsoluteTurn
 * Function Description : This function is called to turn odrive motor n number of turns, float number is also acceptable
 * Function Arguments	: odrive		structure pointer to Odrv_t
 *                        count_num		turn count umber
 * 						  input_mode	requested_input_mode
 * Function Return		: None
 * Function Example		: OdriveAbsoluteTurn(&odrive, 1.3, TRAPEZOIDAL_TRAJECTORY);
 */

void OdriveAbsoluteTurn(Odrv_t* odrive,float count_num,InputMode input_mode)
{
	uint8_t TxBuffer[8] = {0};

	if(input_mode != PASSTHROUGH || input_mode != POS_FILTER || input_mode != TRAPEZOIDAL_TRAJECTORY)
		input_mode = TRAPEZOIDAL_TRAJECTORY;

	memcpy(&TxBuffer[0], &count_num, 4);

	OdriveSetControlInputMode(odrive, POSITION_CONTROL, input_mode);	//make sure control mode is position control

	if(odrive->current_state != CLOSED_LOOP_CONTROL)
		OdriveSetAxisRequestedState(odrive, CLOSED_LOOP_CONTROL);

	OdriveSendCAN(odrive, SET_INPUT_POS, TxBuffer);

	odrive->stop = odrive->hand_brake = 0;
}


void OdriveSetRPSandCurrentMax(Odrv_t* odrive,float rps_lim,float current_lim)
{
	uint8_t TxBuffer[8] = {0};

	memcpy(&TxBuffer[0], &rps_lim, 4);
	memcpy(&TxBuffer[4], &current_lim, 4);

	OdriveSendCAN(odrive,SET_LIMITS,TxBuffer);
}

void OdriveSendCAN(Odrv_t* odrive,uint8_t command,uint8_t* buffer){
	CAN_TxMsg(odrive->hcanx, odrive->Instance << 5 | ((uint16_t)command), buffer, 8);
}

void OdriveSendRTRCAN(Odrv_t* odrive,uint8_t command){
	CAN_TxRTR(odrive->hcanx, odrive->Instance << 5 | ((uint16_t)command));
}

void OdriveGetEncoderFeedback(Odrv_t* odrive)
{
	OdriveSendRTRCAN(odrive, GET_ENCODER_ESTIMATE);
}

void OdriveGetTemperatureFeedback(Odrv_t* odrive)
{
	OdriveSendRTRCAN(odrive, GET_TEMPERATURE);
}

void OdriveGetBusVoltageAndCurrent(Odrv_t* odrive)
{
	OdriveSendRTRCAN(odrive, GET_BUS_VOLTAGE_AND_CURRENT);
}

void OdriveRPStoVelocity(Odrv_t* odrive)
{
	float wheel_radius = odrive->wheel_diameter / 2;
	odrive->feedback.velocity = wheel_radius * M_PI * odrive->feedback.encoder;
}

const char* Odriveerror_to_string(error_code fault) {
	switch (fault) {
	case INITIALIZING: return "INITIALIZING";
	case SYSTEM_LEVEL: return "SYSTEM_LEVEL";	//firmware bug / system error: memory corruption, stack overflow, frozen thread
	case TIMING_ERROR: return "TIMING_ERROR";
	case MISSING_ESTIMATE: return "MISSING_ESTIMATE";
	case BAD_CONFIG: return "BAD_CONFIG";
	case DRV_FAULT: return "DRV_FAULT";
	case MISSING_INPUT: return "MISSING_INPUT";
	case DC_BUS_OVER_VOLTAGE: return "DC_BUS_OVER_VOLTAGE";
	case DC_BUS_UNDER_VOLTAGE: return "DC_BUS_UNDER_VOLTAGE";
	case DC_BUS_OVER_CURRENT: return "DC_BUS_OVER_CURRENT";
	case DC_BUS_OVER_REGEN_CURRENT: return "DC_BUS_OVER_REGEN_CURRENT";
	case CURRENT_LIMIT_VIOLATION: return "CURRENT_LIMIT_VIOLATION";
	case MOTOR_OVER_TEMP: return "MOTOR_OVER_TEMP";
	case INVERTER_OVER_TEMP: return "INVERTER_OVER_TEMP";
    case VELOCITY_LIMIT_VIOLATION: return "VELOCITY_LIMIT_VIOLATION";
    case POSITION_LIMIT_VIOLATION: return "POSITION_LIMIT_VIOLATION";
    case WATCHDOG_TIMER_EXPIRED: return "WATCHDOG_TIMER_EXPIRED";
    case ESTOP_REQUESTED: return "ESTOP_REQUESTED";
    case SPINOUT_DETECTED: return "SPINOUT_DETECTED";
    case OTHER_DEVICE_FAILED: return "OTHER_DEVICE_FAILED";
    case CALIBRATION_ERROR: return "CALIBRATION_ERROR";
	}

	return "Unknown fault";
}

void OdriveAngle(Odrv_t* odrive, float angle){
	float pos = angle / 360.0;
	OdriveAbsoluteTurn(odrive, pos, TRAPEZOIDAL_TRAJECTORY);
}

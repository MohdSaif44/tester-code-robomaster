/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "RNS_interface.h"
//#include "include.h"

/*********************************************/
/*          Variable                         */
/*********************************************/

uint8_t insData_send[2];

/*********************************************/
/*           Subroutine Function             */
/*********************************************/

/*
 * Function Name		: RNSInit
 * Function Description : This function is called to initialize the Robot Navigation System Module.
 * Function Remarks		: NONE
 * Function Arguments	: -if user define USED_CAN
 * 						   		CANx 		Select CAN peripheral (CAN1 or CAN2)
 * 						 		rns 		pointer to a RNS data structure with RNS_interface _t type
 * 						  -if user define USED_I2C
 * 						  		id			I2C address of RNS
 * 						  		I2Cx		Select I2C peripheral (I2C1, I2C2 or I2C3)
 * 						  		rns 		pointer to a RNS data structure with RNS_interface _t type
 * Function Return		: None
 * Function Example		: RNSInit(CAN1, &RNS);
 * 						  RNSInit(Robot_navi_system, I2C1,  &RNS);
 */

void RNSInit(CAN_HandleTypeDef* hcanx, RNS_interface_t* rns)
{
	rns->busy=0;

	rns->rns_hcanx = hcanx;
	insData_send[0] = 1;
	insData_send[1] = RNS_PENDING;
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS,insData_send,2);

	rns->RNS_data.common_instruction = RNS_PENDING;

	insData_send[0] = 1;
	insData_send[1] = RNS_RESET_POS;
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS,insData_send,2);

	rns->RNS_data.common_instruction = RNS_WAITING;
	int wait=0;
	while(rns->RNS_data.common_instruction == RNS_WAITING){
		if(wait >= 200000){
			insData_send[0] = 1;
			insData_send[1] = RNS_RESET_POS;
			CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS,insData_send,2);
			GPIOC_OUT->bit15 = !GPIOC_OUT->bit15;
			wait = 0;
		}else{
			wait ++;
		}
	}
	rns->busy=1;
}

/*
 * Function Name		: RNSStop
 * Function Description : Command the RNS board to stop and reset the position count.
 * Function Remarks		: NONE
 * Function Arguments	: rns 		pointer to a RNS data structure with RNS_interface _t type
 * Function Return		: None
 * Function Example		: RNSStop(&RNS);
 */

void RNSStop(RNS_interface_t* rns)
{
	if(rns->noinit == 1)
		return;
	rns->ins.instruction = RNS_STOP;
	rns->ins.ins_buffer[0].data = 0.0;
	rns->ins.ins_buffer[1].data = 0.0;
	rns->ins.ins_buffer[2].data = 0.0;
	rns->ins.ins_buffer[3].data = 0.0;

	//RNSSendIns(rns);
	rns->RNS_data.common_instruction = RNS_WAITING;

	rns->busy=0;

	insData_send[0] = 17;
	insData_send[1] = rns->ins.instruction;
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS,insData_send,2);
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf1,(uint8_t*)&(rns->ins.ins_buffer[0]),8);
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf2,(uint8_t*)&(rns->ins.ins_buffer[2]),8);

	while(rns->RNS_data.common_instruction == RNS_WAITING);

	rns->busy=1;
}

/*
 * Function Name		: RNSVelocity
 * Function Description : Command the RNS to move with specified velocity without any position control.
 * Function Remarks		: NONE
 * Function Arguments	: fFLeftVelR	speed of front left motor in meter per second
 * 						  fFRightVelR	speed of front right motor in meter per second
 * 						  fBLeftVelR 	speed of back left motor in meter per second
 * 						  fBRightVelR	speed of back right motor in meter per second
 * 						  rns 			pointer to a RNS data structure with RNS_interface _t type
 * Function Return		: None
 * Function Example		: RNSVelocity(1.0, 1.5 , 1.5 , 1.0, &RNS);
 */

void RNSVelocity(float fFLeftVelR, float fFRightVelR, float fBLeftVelR, float fBRightVelR, RNS_interface_t* rns)
{
	rns->ins.instruction = RNS_VELOCITY;
	rns->ins.ins_buffer[0].data = fFLeftVelR;
	rns->ins.ins_buffer[1].data = fFRightVelR;
	rns->ins.ins_buffer[2].data = fBLeftVelR;
	rns->ins.ins_buffer[3].data = fBRightVelR;
	RNSSendIns(rns);
}

/*
 * Function Name		: RNSPDC
 * Function Description : Command the RNS to move with give pulse width modulation duty cycle.
 * Function Remarks		: NONE
 * Function Arguments	: fFLeftPDC		pulse of front left motor
 * 						  fFRightPDC	pulse of front right motor
 * 						  fBLeftPDC 	pulse of back left motor
 * 						  fBRightPDC	pulse of back right motor
 * 						  rns 			pointer to a RNS data structure with RNS_interface _t type
 * Function Return		: None
 * Function Example		: RNSVelocity(10000, 10000, 20000, 15000, &RNS);
 */
void RNSPDC(float fFLeftPDC, float fFRightPDC, float fBLeftPDC, float fBRightPDC, RNS_interface_t* rns)
{
	rns->ins.instruction = RNS_PDC;
	rns->ins.ins_buffer[0].data = fFLeftPDC;
	rns->ins.ins_buffer[1].data = fFRightPDC;
	rns->ins.ins_buffer[2].data = fBLeftPDC;
	rns->ins.ins_buffer[3].data = fBRightPDC;
	RNSSendIns(rns);
}

/*
 * Function Name		: RNSSwerveVel
 * Function Description : Command the Swerve to move with specified velocity without any position control.
 * Function Remarks		: ONLY APPLICABLE TO SWERVE_DRIVE
 * Function Arguments	: fFLeftVelR	speed of front left BLDC in meter per second
 * 						  fFRightVelR	speed of front right BLDC in meter per second
 * 						  fBLeftVelR 	speed of back left BLDC in meter per second
 * 						  fBRightVelR	speed of back right BLDC in meter per second
 * 						  rns 			pointer to a RNS data structure with RNS_interface _t type
 * Function Return		: None
 * Function Example		: RNSSwerveVel(1.0, 1.5 , 1.5 , 1.0, &RNS);
 */
void RNSSwerveVel(float fFLeftVelR, float fFRightVelR, float fBLeftVelR, float fBRightVelR, RNS_interface_t* rns)
{
	rns->ins.instruction = RNS_SWERVE_SET_VEL;
	rns->ins.ins_buffer[0].data = fFLeftVelR;
	rns->ins.ins_buffer[1].data = fFRightVelR;
	rns->ins.ins_buffer[2].data = fBLeftVelR;
	rns->ins.ins_buffer[3].data = fBRightVelR;
	RNSSendIns(rns);
}

/*
 * Function Name		: RNSSwerveRPM
 * Function Description : Command the Swerve to move with specified RPM without any position control.
 * Function Remarks		: ONLY APPLICABLE TO SWERVE_DRIVE
 * Function Arguments	: fFLeftRPM		speed of front left BLDC in rotation per minute
 * 						  fFRightRPM	speed of front right BLDC in rotation per minute
 * 						  fBLeftRPM 	speed of back left BLDC in rotation per minute
 * 						  fBRightRPM	speed of back right BLDC in rotation per minute
 * 						  rns 			pointer to a RNS data structure with RNS_interface _t type
 * Function Return		: None
 * Function Example		: RNSSwerveRPM(10000, 15000, 15200, 10000, &RNS);
 */
void RNSSwerveRPM(float fFLeftRPM, float fFRightRPM, float fBLeftRPM, float fBRightRPM, RNS_interface_t* rns)
{
	rns->ins.instruction = RNS_SWERVE_SET_RPM;
	rns->ins.ins_buffer[0].data = fFLeftRPM;
	rns->ins.ins_buffer[1].data = fFRightRPM;
	rns->ins.ins_buffer[2].data = fBLeftRPM;
	rns->ins.ins_buffer[3].data = fBRightRPM;
	RNSSendIns(rns);
}

/*
 * Function Name		: RNSSwervePDC
 * Function Description : Command the Swerve to move with specified velocity without any position control.
 * Function Remarks		: ONLY APPLICABLE TO SWERVE_DRIVE
 * Function Arguments	: fFLeftPDC		duty cycle front left BLDC
 * 						  fFRightPDC	duty cycle front right BLDC
 * 						  fBLeftPDC 	duty cycle back left BLDC
 * 						  fBRightPDC	duty cycle back right BLDC
 * 						  rns 			pointer to a RNS data structure with RNS_interface _t type
 * Function Return		: None
 * Function Example		: RNSSwervePDC(0.1, 0.5 , 0.15 , 1.0, &RNS);
 */
void RNSSwervePDC(float fFLeftPDC, float fFRightPDC, float fBLeftPDC, float fBRightPDC, RNS_interface_t* rns)
{
	rns->ins.instruction = RNS_SWERVE_SET_PDC;
	rns->ins.ins_buffer[0].data = fFLeftPDC;
	rns->ins.ins_buffer[1].data = fFRightPDC;
	rns->ins.ins_buffer[2].data = fBLeftPDC;
	rns->ins.ins_buffer[3].data = fBRightPDC;
	RNSSendIns(rns);
}

/*
 * Function Name		: RNSSwerveCurr
 * Function Description : Command the Swerve to move with specified velocity without any position control.
 * Function Remarks		: ONLY APPLICABLE TO SWERVE_DRIVE
 * Function Arguments	: fFLeftCurr	current feed to front left motor in Ampere
 * 						  fFRightCurr	current feed to front right motor in Ampere
 * 						  fBLeftCurr 	current feed to back left motor in Ampere
 * 						  fBRightCurr	current feed to back right motor in Ampere
 * 						  rns 			pointer to a RNS data structure with RNS_interface _t type
 * Function Return		: None
 * Function Example		: RNSSwerveCurr(1.0, 1.5 , 1.5 , 1.0, &RNS);
 */
void RNSSwerveCurr(float fFLeftCurr, float fFRightCurr, float fBLeftCurr, float fBRightCurr, RNS_interface_t* rns)
{
	rns->ins.instruction = RNS_SWERVE_SET_Curr;
	rns->ins.ins_buffer[0].data = fFLeftCurr;
	rns->ins.ins_buffer[1].data = fFRightCurr;
	rns->ins.ins_buffer[2].data = fBLeftCurr;
	rns->ins.ins_buffer[3].data = fBRightCurr;
	RNSSendIns(rns);
}

/*
 * Function Name		: RNSDir
 * Function Description : Command the RNS to move to specified angle position without any velocity control.
 * Function Remarks		: ONLY APPLICABLE TO SWERVE_DRIVE
 * 						: range (-360 <-> 360)
 * Function Arguments	: fFLeftPosR	direction of front left motor in angle
 * 						  fFRightPosR	direction of front right motor in angle
 * 						  fBLeftPosR 	direction of back left motor in angle
 * 						  fBRightPosR	direction of back right motor in angle
 * 						  rns 			pointer to a RNS data structure with RNS_interface _t type
 * Function Return		: None
 * Function Example		: RNSDir(30, 30 , 30 , 30, &RNS);
 */

void RNSDir(float fFLeftPosR, float fFRightPosR, float fBLeftPosR, float fBRightPosR, RNS_interface_t* rns)
{
	rns->ins.instruction = RNS_SWERVE_SET_WHEEL_ANGLE;
	rns->ins.ins_buffer[0].data = fFLeftPosR;
	rns->ins.ins_buffer[1].data = fFRightPosR;
	rns->ins.ins_buffer[2].data = fBLeftPosR;
	rns->ins.ins_buffer[3].data = fBRightPosR;
	RNSSendIns(rns);
}

/*
 * Function Name		: RNSDirZero
 * Function Description : Command the RNS to move to 0 deg without any velocity control.
 * Function Remarks		: ONLY APPLICABLE TO SWERVE_DRIVE
 * Function Arguments	: rns 			pointer to a RNS data structure with RNS_interface _t type
 * Function Return		: None
 * Function Example		: RNSDirZero(&RNS);
 */

void RNSDirZero(RNS_interface_t* rns)
{
	rns->ins.instruction = RNS_SWERVE_RESET_WHEEL_ANGLE;
	rns->ins.ins_buffer[0].data = 0.0;
	rns->ins.ins_buffer[1].data = 0.0;
	rns->ins.ins_buffer[2].data = 0.0;
	rns->ins.ins_buffer[3].data = 0.0;
	RNSSendIns(rns);
	while(rns->RNS_data.common_instruction == RNS_BUSY);
}

/*
 * Function Name		: RNSSwerveCommand
 * Function Description : Command the Swerve to move manually without any position control.
 * Function Remarks		: ONLY APPLICABLE TO SWERVE_DRIVE
 * Function Arguments	: angle			target angle of wheel
 * 						  magnitude		speed of BLDC, based on the moving command of VESC
 * 						  turn			turnning of robot
 * 						  rns 			pointer to a RNS data structure with RNS_interface _t type
 * Function Return		: None
 * Function Example		: RNSSwerveCommand(45, 0.3, 0.0, 0.2, &RNS);
 */
void RNSSwerveCommand(float angle, float magnitude, float turn, float speed, RNS_interface_t* rns)
{
	rns->ins.instruction = RNS_SWERVE_COMMAND;
	rns->ins.ins_buffer[0].data = angle;
	rns->ins.ins_buffer[1].data = magnitude;
	rns->ins.ins_buffer[2].data = turn;
	rns->ins.ins_buffer[3].data = speed;
	RNSSendIns(rns);
}

/*
 * Function Name		: RNSSwerveReset
 * Function Description : Command the RNS board to reset position and stop BLDC of swerve base.
 * Function Remarks		: ONLY APPLICABLE TO SWERVE_DRIVE
 * Function Arguments	: rns 		pointer to a RNS data structure with RNS_interface _t type
 * Function Return		: None
 * Function Example		: RNSSwerveReset(&RNS);
 */
void RNSSwerveReset(RNS_interface_t* rns)
{
	if(rns->noinit == 1)
		return;
	rns->ins.instruction = RNS_SWERVE_Reset;

	//RNSSendIns(rns);
	rns->RNS_data.common_instruction = RNS_WAITING;

	rns->busy=0;

	insData_send[0] = 1;
	insData_send[1] = rns->ins.instruction;
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS,insData_send,2);

	while(rns->RNS_data.common_instruction == RNS_WAITING);

	rns->busy=1;
}

/*
 * Function Name		: RNSSwerveStop
 * Function Description : Command the RNS board to stop BLDC of swerve base and stay.
 * Function Remarks		: ONLY APPLICABLE TO SWERVE_DRIVE
 * Function Arguments	: rns 		pointer to a RNS data structure with RNS_interface _t type
 * Function Return		: None
 * Function Example		: RNSSwerveStop(&RNS);
 */
void RNSSwerveStop(RNS_interface_t* rns)
{
	if(rns->noinit == 1)
		return;
	rns->ins.instruction = RNS_SWERVE_STOP;
	rns->ins.ins_buffer[0].data = 0.0;
	rns->ins.ins_buffer[1].data = 0.0;
	rns->ins.ins_buffer[2].data = 0.0;
	rns->ins.ins_buffer[3].data = 0.0;

	//RNSSendIns(rns);
	rns->RNS_data.common_instruction = RNS_WAITING;

	rns->busy=0;

	insData_send[0] = 17;
	insData_send[1] = rns->ins.instruction;
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS,insData_send,2);
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf1,(uint8_t*)&(rns->ins.ins_buffer[0]),8);
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf2,(uint8_t*)&(rns->ins.ins_buffer[2]),8);

	while(rns->RNS_data.common_instruction == RNS_WAITING);

	rns->busy=1;
}

/*
 * Function Name		: RNSRelease
 * Function Description : Command the RNS board to stop BLDC and cut off power to BLDC.
 * Function Remarks		: ONLY APPLICABLE TO SWERVE_DRIVE
 * Function Arguments	: rns 		pointer to a RNS data structure with RNS_interface _t type
 * Function Return		: None
 * Function Example		: RNSRelease(&RNS);
 */
void RNSRelease(RNS_interface_t* rns)
{
	if(rns->noinit == 1)
		return;
	rns->ins.instruction = RNS_SWERVE_Release;
	rns->ins.ins_buffer[0].data = 0.0;
	rns->ins.ins_buffer[1].data = 0.0;
	rns->ins.ins_buffer[2].data = 0.0;
	rns->ins.ins_buffer[3].data = 0.0;

	//RNSSendIns(rns);
	rns->RNS_data.common_instruction = RNS_WAITING;

	rns->busy=0;

	insData_send[0] = 17;
	insData_send[1] = rns->ins.instruction;
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS,insData_send,2);
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf1,(uint8_t*)&(rns->ins.ins_buffer[0]),8);
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf2,(uint8_t*)&(rns->ins.ins_buffer[2]),8);

	while(rns->RNS_data.common_instruction == RNS_WAITING);

	rns->busy=1;
}

/*
 * Function Name		: RNSHBrake
 * Function Description : Command the RNS board to stop BLDC and cut off power to BLDC.
 * Function Remarks		: ONLY APPLICABLE TO SWERVE_DRIVE
 * Function Arguments	: Hbrake	HandBrake current of BLDC in Ampere
 * 						  rns 		pointer to a RNS data structure with RNS_interface _t type
 * Function Return		: None
 * Function Example		: RNSHBrake(6.6, &RNS);
 */
void RNSHBrake(float Hbrake, RNS_interface_t* rns)
{
	if(rns->noinit == 1)
		return;
	rns->ins.instruction = RNS_SWERVE_HBRAKE;
	rns->ins.ins_buffer[0].data = Hbrake;
	rns->ins.ins_buffer[1].data = Hbrake;

	rns->RNS_data.common_instruction = RNS_WAITING;

	rns->busy=0;

	insData_send[0] = 17;
	insData_send[1] = rns->ins.instruction;
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS,insData_send,2);
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf1,(uint8_t*)&(rns->ins.ins_buffer[0]),8);

	while(rns->RNS_data.common_instruction == RNS_WAITING);

	rns->busy=1;
}

/*
 * Function Name		: RNSCurrBrake
 * Function Description : Command the RNS board to stop BLDC and cut off power to BLDC.
 * Function Remarks		: ONLY APPLICABLE TO SWERVE_DRIVE
 * Function Arguments	: Currbrake		Brake current of BLDC in Ampere
 * 						  rns 			pointer to a RNS data structure with RNS_interface _t type
 * Function Return		: None
 * Function Example		: RNSCurrBrake(6.6, &RNS);
 */
void RNSCurrBrake(float Currbrake, RNS_interface_t* rns)
{
	if(rns->noinit == 1)
		return;
	rns->ins.instruction = RNS_SWERVE_CurrBRAKE;
	rns->ins.ins_buffer[0].data = Currbrake;
	rns->ins.ins_buffer[1].data = Currbrake;

	rns->RNS_data.common_instruction = RNS_WAITING;

	rns->busy=0;

	insData_send[0] = 17;
	insData_send[1] = rns->ins.instruction;
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS,insData_send,2);
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf1,(uint8_t*)&(rns->ins.ins_buffer[0]),8);

	while(rns->RNS_data.common_instruction == RNS_WAITING);

	rns->busy=1;
}

/*
 * Function Name		: RNSLFDist
 * Function Description : Command the RNS to line follow with given distance and direction.
 * Function Remarks		: NONE
 * Function Arguments	: MoveDir	    enum type with the members of x_ax, y_ax
 * 						  Dir			enum type with the members of
 * 						     			DIR_FRONT, DIR_BACK,DIR_LEFT,DIR_RIGHT
 * 						  LF_dist 		Distance for line follow
 * 						  rns 			pointer to a RNS data structure with RNS_interface _t type
 * Function Return		: None
 * Function Example		: RNSLFDist(x_ax, DIR_LEFT, 100.0 ,&RNS);
 */
void RNSLFDist( dir_t Dir, float LF_vel, float LF_dist,RNS_interface_t* rns){

	rns->ins.instruction = RNS_LF_DIST;
	rns->ins.ins_buffer[0].data = (float)Dir;
	rns->ins.ins_buffer[1].data = LF_vel;
	rns->ins.ins_buffer[2].data = LF_dist;
	rns->ins.ins_buffer[3].data = 0;

	RNSSendIns(rns);
}

/*
 * Function Name		: RNSLFJunc
 * Function Description : Command the RNS to line follow with given direction and junction.
 * Function Remarks		: NONE
 * Function Arguments	: Dir			enum type with the members of
 * 						     			DIR_FRONT, DIR_BACK,DIR_LEFT,DIR_RIGHT
 * 						  LF_junc 		number of junction to be pass through with line follow
 * 						  rns 			pointer to a RNS data structure with RNS_interface _t type
 * Function Return		: None
 * Function Example		: RNSLFJunc(DIR_LEFT,1.5,2.5, 2.0 ,&RNS);
 */
void RNSLFJunc(dir_t Dir, float LF_vel, float LF_dist, float LF_junc,RNS_interface_t* rns){

	rns->ins.instruction = RNS_LF_JUNC;
	rns->ins.ins_buffer[0].data = (float)Dir;
	rns->ins.ins_buffer[1].data = LF_vel;
	rns->ins.ins_buffer[2].data = LF_dist;
	rns->ins.ins_buffer[3].data = LF_junc;

	RNSSendIns(rns);
}

/*
 * Function Name		: RNSIMURotate
 * Function Description : Command the RNS to rotate with given angle of max:+-180 degree.
 * Function Remarks		: NONE
 * Function Arguments	: AngleDeg		Rotate angle. Max:+-180degree, +:clkwise, -:anticlkwise
 * 						  rns 			pointer to a RNS data structure with RNS_interface _t type
 * Function Return		: None
 * Function Example		: RNSIMURotate(70,&RNS);
 */
void RNSIMURotate(int AngleDeg,RNS_interface_t* rns){
	rns->ins.instruction = RNS_ROTATE;
	rns->ins.ins_buffer[0].data = (float)AngleDeg;
	rns->ins.ins_buffer[1].data = 0;
	rns->ins.ins_buffer[2].data = 0;
	rns->ins.ins_buffer[3].data = 0;

	RNSSendIns(rns);
}

/*
 * Function Name		: RNSOdnGoto
 * Function Description : This function is called to send all the points  for the path to rns for calculation
 * Function Remarks		: This function can only be called after RNSOdnStart is called
 * Function Arguments	: allpoints[][5]	array for all the points in the path
 * 							[][0]= minimum speed
 * 							[][1]= x-coordinate
 * 							[][2]= y-coordinate
 * 							[][3]= z-coordinate
 * 							[][4]= xy pid output
 * 							[][5]= Point Lock
 * 							[][6]= Curve Control Radius
 * 						  no_point			Number of points to be sent
 * 						  rns 			pointer to a RNS data structure with RNS_interface _t type
 * Function Return		: None
 * Function Example		: float point[1][7] = {{3.0, 0.001, 1.0, -180.0, 0.0, 0.0, 1.5}};					//for coordinates use this type of naming
 * 								RNSPPstart(point,1,&rns);
 */

void RNSPPstart(float allpoints[][7],int no_point,RNS_interface_t* rns){
	RNSSet(rns, RNS_PPSend_num_Point,(float)no_point);
	int k;
	for(k=0;k<no_point;k++)
		RNSSet(rns, RNS_PPSendPoint, allpoints[k][0],allpoints[k][1],allpoints[k][2],allpoints[k][3],allpoints[k][4],allpoints[k][5],allpoints[k][6]);
	rns->ins.instruction = RNS_PPStart;
	rns->ins.ins_buffer[0].data = 0;
	rns->ins.ins_buffer[1].data = 0;
	rns->ins.ins_buffer[2].data = 0;
	rns->ins.ins_buffer[3].data = 0;

	rns->PathPlan = 1;
	RNSSendIns(rns);
}

void RNSPPstart_PS(float** allpoints,int no_point,RNS_interface_t* rns){
	RNSSet(rns, RNS_PPSend_num_Point,(float)no_point);
	int k;
	for(k=0;k<no_point;k++)
		RNSSet(rns, RNS_PPSendPoint, allpoints[k][0],allpoints[k][1],allpoints[k][2],allpoints[k][3],allpoints[k][4],allpoints[k][5],allpoints[k][6]);
	rns->ins.instruction = RNS_PPStart;
	rns->ins.ins_buffer[0].data = 0;
	rns->ins.ins_buffer[1].data = 0;
	rns->ins.ins_buffer[2].data = 0;
	rns->ins.ins_buffer[3].data = 0;

	RNSSendIns(rns);
}

/*
 * Function Name		: RNSSendVelIns
 * Function Description : Checks the status of the RNS and sends the instruction to RNS after the previous instruction is sent.
 * Function Remarks		: Not intended to be used by user
 * Function Arguments	: rns 			pointer to a RNS data structure with RNS_interface _t type
 * Function Return		: None
 * Function Example		: None
 */

void RNSSendIns(RNS_interface_t* rns)
{
	if(rns->noinit == 1)
		return;
	rns->busy=0;
	while(rns->RNS_data.common_instruction == RNS_BUSY);
	rns->RNS_data.common_instruction = RNS_WAITING;

	insData_send[0] = 17;
	insData_send[1] = rns->ins.instruction;
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS, insData_send,2);
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf1,(uint8_t*)&(rns->ins.ins_buffer[0].data),8);
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf2,(uint8_t*)&(rns->ins.ins_buffer[2].data),8);

	while(rns->RNS_data.common_instruction == RNS_WAITING){
		//		if(GPIOB_IN->bit7==0){
		//			GPIOC_OUT->bit14=0;
		//		} else{
		//			GPIOC_OUT->bit14=1;
		//		}
	}

	rns->busy=1;
}

/*
 * Function Name		: RNSSet
 * Function Description : Configure the RNS in terms of system function and gains.
 * Function Remarks		: Can be called during an instruction is running or during pending. Every value must cast to float type.
 * Function Arguments	: rns 			pointer to a RNS data structure with RNS_interface _t type
 * 						  parameter 	Enumeration of parameters
 * 						  ...			Values to set for the RNS, depending on parameters
 * Function Return		: None
 * Function Example		: RNSSet(&RNS, RNS_F_KCD_PTD, 0.9956, 0.01/2000);
 */

void RNSSet(RNS_interface_t* rns, unsigned char parameter, ...)
{
	if(rns->noinit == 1)
		return;
	va_list value;
	rns->param.parameter = parameter;
	va_start(value, parameter);
	while(rns->RNS_data.common_instruction == RNS_WAITING);
	if (parameter > RNS_PARAM_1){
		rns->param.param_buffer[0].data = va_arg(value, double);
		rns->param.param_buffer[1].data = 0;
		rns->param.param_buffer[2].data = 0;
		rns->param.param_buffer[3].data = 0;
		rns->param.param_buffer[4].data = 0;
		rns->param.param_buffer[5].data = 0;
		rns->param.param_buffer[6].data = 0;
		rns->param.param_buffer[7].data = 0;
	}
	if (parameter > RNS_PARAM_2){
		rns->param.param_buffer[1].data = va_arg(value, double);
		rns->param.param_buffer[2].data = 0;
		rns->param.param_buffer[3].data = 0;
		rns->param.param_buffer[4].data = 0;
		rns->param.param_buffer[5].data = 0;
		rns->param.param_buffer[6].data = 0;
		rns->param.param_buffer[7].data = 0;
	}
	if (parameter > RNS_PARAM_3){
		rns->param.param_buffer[2].data = va_arg(value, double);
		rns->param.param_buffer[3].data = 0;
		rns->param.param_buffer[4].data = 0;
		rns->param.param_buffer[5].data = 0;
		rns->param.param_buffer[6].data = 0;
		rns->param.param_buffer[7].data = 0;
	}
	if (parameter > RNS_PARAM_4){
		rns->param.param_buffer[3].data = va_arg(value, double);
		rns->param.param_buffer[4].data = 0;
		rns->param.param_buffer[5].data = 0;
		rns->param.param_buffer[6].data = 0;
		rns->param.param_buffer[7].data = 0;
	}
	if (parameter > RNS_PARAM_5){
		rns->param.param_buffer[4].data = va_arg(value,double);
		rns->param.param_buffer[5].data = 0;
		rns->param.param_buffer[6].data = 0;
		rns->param.param_buffer[7].data = 0;
	}
	if (parameter > RNS_PARAM_6){
		rns->param.param_buffer[5].data = va_arg(value, double);
		rns->param.param_buffer[6].data = 0;
		rns->param.param_buffer[7].data = 0;
	}
	if (parameter > RNS_PARAM_7){
		rns->param.param_buffer[6].data = va_arg(value, double);
		rns->param.param_buffer[7].data = 0;
	}
	if(parameter > RNS_PARAM_8){
		rns->param.param_buffer[7].data = va_arg(value, double);
	}

	va_end(value);

	rns->busy=0;

	insData_send[0] = 17;
	insData_send[1] = rns->param.parameter;

	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS,insData_send,2);
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf1,(uint8_t*)&(rns->param.param_buffer[0]),8);
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf2,(uint8_t*)&(rns->param.param_buffer[2]),8);
	if(parameter > RNS_PARAM_5)
		CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf3,(uint8_t*)&(rns->param.param_buffer[4]),8);
	if(parameter > RNS_PARAM_7)
		CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf4,(uint8_t*)&(rns->param.param_buffer[6]),8);

	rns->RNS_data.common_instruction = RNS_WAITING;
	while(rns->RNS_data.common_instruction == RNS_WAITING);
	//UARTPrintString(&huart5,"1\r\n");
	rns->busy=1;
}

/*
 * Function Name		: RNSEnquire
 * Function Description : This function is classified under enquiry type instruction.
 * Function Remarks		: Should only be called during an instruction is running (the RNS is in motion).
 * 						  Failing to do so will return gibberish data or previously returned value.
 * Function Arguments	: parameter 	Enumeration to parameter to enquire
 * 						  rns 			pointer to a RNS data structure with RNS_interface _t type
 * Function Return		: None
 * Function Example		: RNSSet(&RNS, RNS_F_KCD_PTD, 0.9956, 0.01/2000);
 */

uint8_t RNSEnquire(unsigned char parameter, RNS_interface_t* rns)
{
	if(rns->noinit == 1)
		return 0;
	rns->ins.instruction = parameter;
	rns->ins.ins_buffer[0].data = 0.0;
	rns->ins.ins_buffer[1].data = 0.0;
	rns->ins.ins_buffer[2].data = 0.0;
	rns->ins.ins_buffer[3].data = 0.0;

	//	RNSSendIns(rns);

	rns->busy=0;

	rns->RNS_data.common_instruction = RNS_WAITING;

	insData_send[0] = 17;
	insData_send[1] = rns->ins.instruction;
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS,insData_send,2);
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf1,(uint8_t*)&(rns->ins.ins_buffer[0].data),8);
	CAN_TxMsg(rns->rns_hcanx,mainboard_TO_RNS_buf2,(uint8_t*)&(rns->ins.ins_buffer[2].data),8);

	while(rns->RNS_data.common_instruction == RNS_WAITING){
		//		if(GPIOB_IN->bit7==0){
		//					GPIOC_OUT->bit14=0;
		//				} else{
		//					GPIOC_OUT->bit14=1;
		//				}
	}

	rns->busy=1;

	rns->enq.enquiry = parameter;
	rns->enq.enq_buffer[0].data = rns->RNS_data.common_buffer[0].data;
	rns->enq.enq_buffer[1].data = rns->RNS_data.common_buffer[1].data;
	rns->enq.enq_buffer[2].data = rns->RNS_data.common_buffer[2].data;
	rns->enq.enq_buffer[3].data = rns->RNS_data.common_buffer[3].data;

	return 1;
}

void RNSSync (RNS_interface_t* rns){

	if(rns->noinit == 1)
		return;

	if(rns->busy == 1){
		insData_send[0] = 5;
		insData_send[1] = RNS_RECONNECT;
		CAN_TxMsg(rns->rns_hcanx, RNS_Check_Connection, insData_send, 2);

		int wait=0;
		while(rns->RNS_data.common_instruction == RNS_WAITING && rns->RNS_data.common_instruction != RNS_BUSY){
			if(wait >= 200000){
				insData_send[0] = 5;
				insData_send[1] = RNS_RECONNECT;
				CAN_TxMsg(rns->rns_hcanx, RNS_Check_Connection, insData_send, 2);
				GPIOC_OUT->bit15 = !GPIOC_OUT->bit15;
				wait = 0;
			}else{
				wait ++;
			}
		}
	}
}
/*********************************************/

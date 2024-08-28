#ifndef MODN_H
#define MODN_H

/*********************************************/
/*          Include Header                   */
/*********************************************/
#include "../PID/PID.h"
#include "../PSx_Interface/PSx_Interface.h"
#include "../RNS_interface/RNS_interface.h"
#include "../VESC_CAN/vesc_interface.h"
#include "../IMU/r6091u.h"
/*********************************************/


/*********************************************/
/*          Define                           */
/*********************************************/

extern R6091U_t IMU;
extern VESC_t vesc;

/*********************************************/



/*********************************************/
/*          Enumarator                       */
/*********************************************/

typedef enum{
	MODN_FWD_OMNI,
	MODN_TRI_OMNI,
	MODN_PI_OMNI,
	MODN_MECANUM,
	MODN_FWD_OMNI_BLDC,
	MODN_TRI_OMNI_BLDC,
}RobotBaseType_t;

typedef enum{
	OPERATOR_TURNED_0_DEGREE,
	OPERATOR_TURNED_90_DEGREES_CLOCKWISE,
	OPERATOR_TURNED_180_DEGREES,
	OPERATOR_TURNED_90_DEGREES_ANTICLOCKWISE
}OrientationMODN_t;

enum{
	NO_TURN,
	START_TURN
}TurnState;

/*********************************************/

/*********************************************/
/*          Variable                         */
/*********************************************/
struct {
	float x_vel;
	float y_vel;
	float w_vel;
	float vel1;
	float vel2;
	float vel3;
	float vel4;
	float speed;
	float brakecurr;
	float turnspeed;
	float e;
	RobotBaseType_t base;
	uint8_t turnState;
	float radTol;
	float imuGain;
	float imuFeedback;
	float radTarget;
	int orientation;
	int t;
	int delay;
}MODN;

/*********************************************/
float realZ,realZrad,stopcommand;

/*********************************************/
/*           Function Prototype              */
/*********************************************/
void MODNInit(RobotBaseType_t base, float speed, float turnSpeed, float angleTol, float imuGain, int delay_time);
void MODNSwerveInit(VESC_t* vesc, char BLDCmethod, char ArgMotor, float width, float length, float threshold, float tolerance, float brake_current);
void LegacyMODN(PSxBT_t *psx, RNS_interface_t* rns);
void realMODN(PSxBT_t *psx, RNS_interface_t* rns);
void imuRealMODN(PSxBT_t *psx, RNS_interface_t* rns);
void movableRealMODN(PSxBT_t *psx, RNS_interface_t* rns);
void autoorientMODN(PSxBT_t *psx, RNS_interface_t* rns);
void setOrientationMODN(OrientationMODN_t orientation);
void setSpeedMODN(float speed);
void setImuGainMODN(float imuGain);
void setDelayMODN(int ticks);
/*********************************************/
#endif

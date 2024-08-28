/*********************************************/
/*          Include Header                   */
/*********************************************/

#include "MODN.h"

/*********************************************/
/*        	   Variables		             */
/*********************************************/



/*********************************************/
/*              Private Function             */
/*********************************************/

void ApplyMODN(PSxBT_t *psx, RNS_interface_t* rns);

/*********************************************/
/*           Subroutine Function             */
/*********************************************/

/* Function Name		: MODNInit
 * Function Description : To initialise manual navigation parameters.
 * Function Remarks		:
 * Function Arguments	:
 *						  base                          Can be any of enum RobotBaseType_t
 * 						  speed	 						speed in m/s
 * 						  turnSpeed					    turnSpeed in relative to speed
 * 						  angleTol						maximum angle tolerance in imuRealMODN
 * 						  imuGain						range from 0.0 to 1.0. Angle correction
 * 						  								gain added to four wheels based on the speed
 * Function Return		: NONE
 * Function Example		: void MODNInit(MODN_FWD_OMNI, 3.0, 0.5, 2.0, 0.1);
 */
void MODNInit(RobotBaseType_t base, float speed, float turnSpeed, float angleTol, float imuGain, int delay_time)
{
	MODN.brakecurr = 9.9;
	MODN.base = base;
	MODN.speed = speed;
	MODN.turnspeed = turnSpeed;
	MODN.radTol = angleTol/180.0*3.14159265359;
	MODN.imuGain = speed*imuGain/MODN.radTol;
	MODN.radTarget = 0.0;
	MODN.orientation = 0;
	MODN.t = 0;
	MODN.delay = delay_time;
}

/* Function Name		: LegacyMODN
 * Function Description : To calculate velocity of four wheels based on ps4 left joyStick and ps4.joyR_2 ps4.joyL_2
 * Function Remarks		: Follow robot x and y. May cause confusion after robot rotate
 * Function Arguments	:
 *						  psx							struct of PlayStation controller
 * 						  rns	 						pointer to RNS_interface_t struct
 * Function Return		: NONE
 * Function Example		: LegacyMODN(ps4, &rns);
 */
void LegacyMODN(PSxBT_t *psx, RNS_interface_t* rns)
{
	MODN.x_vel = (psx->joyL_x) * MODN.speed;
	MODN.y_vel = (psx->joyL_y) * MODN.speed;
	MODN.w_vel = ((psx->joyR_2) - (psx->joyL_2))* MODN.speed;
	if(MODN.x_vel==0.0 && MODN.y_vel==0.0 && MODN.w_vel==0.0){
		RNSStop(rns);
		if(stopcommand < 66){
			stopcommand++;
			VESCHandBrake(MODN.brakecurr, &vesc);
		}else{
			VESCReleaseMotor(&vesc);
			VESCStop(&vesc);
		}
	}else{
		if(MODN.base == MODN_FWD_OMNI) {
			MODN.vel1 = MODN.x_vel * COS_45_DEG  + MODN.y_vel * SIN_45_DEG + MODN.w_vel * MODN.turnspeed;
			MODN.vel2 = MODN.x_vel * -COS_45_DEG + MODN.y_vel * SIN_45_DEG - MODN.w_vel * MODN.turnspeed;
			MODN.vel3 = MODN.x_vel * -COS_45_DEG + MODN.y_vel * SIN_45_DEG + MODN.w_vel * MODN.turnspeed;
			MODN.vel4 = MODN.x_vel * COS_45_DEG  + MODN.y_vel * SIN_45_DEG - MODN.w_vel * MODN.turnspeed;
			RNSVelocity(MODN.vel1, MODN.vel2, MODN.vel3, MODN.vel4, rns);
		}else if(MODN.base == MODN_MECANUM){
			MODN.vel1 = MODN.y_vel*(1.0) + MODN.x_vel*(1.0)  + MODN.w_vel/*(MODN.turnspeed + MODN.e)*/;
			MODN.vel2 = MODN.y_vel*(1.0) + MODN.x_vel*(-1.0) - MODN.w_vel/*(MODN.turnspeed + MODN.e)*/;
			MODN.vel3 = MODN.y_vel*(1.0) + MODN.x_vel*(-1.0) + MODN.w_vel/*(MODN.turnspeed + MODN.e)*/;
			MODN.vel4 = MODN.y_vel*(1.0) + MODN.x_vel*(1.0)  - MODN.w_vel/*(MODN.turnspeed + MODN.e)*/;
			RNSVelocity(MODN.vel1, MODN.vel2, MODN.vel3, MODN.vel4, rns);
		}else if (MODN.base == MODN_TRI_OMNI){
			MODN.vel3 = MODN.x_vel * (1.0)   + MODN.w_vel * MODN.turnspeed;
			MODN.vel1 = MODN.y_vel * (0.866) + MODN.x_vel * -COS_30_DEG + MODN.w_vel * MODN.turnspeed;
			MODN.vel2 = MODN.y_vel * (0.866) + MODN.x_vel * COS_30_DEG  - MODN.w_vel * MODN.turnspeed;
			RNSVelocity(MODN.vel1, MODN.vel2, MODN.vel3, 0.0, rns);
		}else if(MODN.base == MODN_FWD_OMNI_BLDC){
			MODN.vel1 = MODN.x_vel * COS_45_DEG  + MODN.y_vel * SIN_45_DEG + MODN.w_vel * MODN.turnspeed;
			MODN.vel2 = MODN.x_vel * -COS_45_DEG + MODN.y_vel * SIN_45_DEG - MODN.w_vel * MODN.turnspeed;
			MODN.vel3 = MODN.x_vel * -COS_45_DEG + MODN.y_vel * SIN_45_DEG + MODN.w_vel * MODN.turnspeed;
			MODN.vel4 = MODN.x_vel * COS_45_DEG  + MODN.y_vel * SIN_45_DEG - MODN.w_vel * MODN.turnspeed;
			VESCVelocity(MODN.vel1, MODN.vel2, MODN.vel3, MODN.vel4, &vesc);
		}else if(MODN.base == MODN_TRI_OMNI_BLDC){
			MODN.vel3 = MODN.x_vel * (1.0)   + MODN.w_vel * MODN.turnspeed;
			MODN.vel1 = MODN.y_vel * (0.866) + MODN.x_vel * -COS_30_DEG + MODN.w_vel * MODN.turnspeed;
			MODN.vel2 = MODN.y_vel * (0.866) + MODN.x_vel * COS_30_DEG  - MODN.w_vel * MODN.turnspeed;
			VESCVelocity(MODN.vel1, MODN.vel2, MODN.vel3, 0.0, &vesc);
		}
		stopcommand = 0;
	}
}

/* Function Name		: realMODN
 * Function Description : To calculate velocity of four wheels based on ps4 left joyStick and ps4.joyR_2 ps4.joyL_2
 * 						  after finding angle of wheels relative to World Real X and World Real Y
 * Function Remarks		: World Real X and Y is set at the moment IMU power up
 * 						  Operators have to face the same direction. May cause confusion if operators went to other place
 * Function Arguments	:
 *						  psx							struct of PlayStation controller
 * 						  rns	 						pointer to RNS_interface_t struct
 * Function Return		: NONE
 * Function Example		: realMODN(ps4, &rns);
 */

void realMODN(PSxBT_t *psx, RNS_interface_t* rns)
{
	MODN.x_vel = (psx->joyL_x) * MODN.speed;
	MODN.y_vel =  (psx->joyL_y) * MODN.speed;
	MODN.w_vel = ((psx->joyR_2) - (psx->joyL_2))* MODN.speed;
	RNSEnquire(RNS_COORDINATE_X_Y_Z_Zrad, rns);
	realZrad = rns->enq.enq_buffer[3].data;
	if(MODN.base == MODN_FWD_OMNI || MODN.base == MODN_FWD_OMNI_BLDC) {
		MODN.vel1 = MODN.x_vel*cosf(0.7854-realZrad) + MODN.y_vel*sinf(0.7854-realZrad) + MODN.w_vel*MODN.turnspeed;
		MODN.vel2 = MODN.x_vel*cosf(2.3562-realZrad) + MODN.y_vel*sinf(2.3562-realZrad) - MODN.w_vel*MODN.turnspeed;
		MODN.vel3 = MODN.x_vel*cosf(2.3562-realZrad) + MODN.y_vel*sinf(2.3562-realZrad) + MODN.w_vel*MODN.turnspeed;
		MODN.vel4 = MODN.x_vel*cosf(0.7854-realZrad) + MODN.y_vel*sinf(0.7854-realZrad) - MODN.w_vel*MODN.turnspeed;
	}
	//		else if(MODN.base == MODN_MECANUM){
	//		MODN.vel1 = MODN.y_vel*(1.0) + MODN.x_vel*(1.0)  + *(MODN.w_vel)/*(MODN.turnspeed + MODN.e)*/;
	//		MODN.vel2 = MODN.y_vel*(1.0) + MODN.x_vel*(-1.0) - *(MODN.w_vel)/*(MODN.turnspeed + MODN.e)*/;
	//		MODN.vel3 = MODN.y_vel*(1.0) + MODN.x_vel*(-1.0) + *(MODN.w_vel)/*(MODN.turnspeed + MODN.e)*/;
	//		MODN.vel4 = MODN.y_vel*(1.0) + MODN.x_vel*(1.0)  - *(MODN.w_vel)/*(MODN.turnspeed + MODN.e)*/;
	//	}
	else if (MODN.base == MODN_TRI_OMNI || MODN.base == MODN_TRI_OMNI_BLDC){
		/*			 C
		 * 		   -___+
		 *
		 * 		+		  +
		 * 		A\		 /B
		 * 		  -     -
		 */
		MODN.vel3 = MODN.y_vel*sinf(-realZrad) 		 + MODN.x_vel*cosf(-realZrad)  	    + MODN.w_vel*MODN.turnspeed;
		MODN.vel1 = MODN.y_vel*sinf(2.0944-realZrad) + MODN.x_vel*cosf(2.0944-realZrad) + MODN.w_vel*MODN.turnspeed;
		MODN.vel2 = MODN.y_vel*sinf(1.0472-realZrad) + MODN.x_vel*cosf(1.0472-realZrad) - MODN.w_vel*MODN.turnspeed;
	}
	ApplyMODN(psx, rns);
}

/* Function Name		: imuRealMODN
 * Function Description : Same usage as realMODN, added with angle correction
 * Function Remarks		: Tune angleTol and imuGain for smoother navigation
 * 						  Alternative way for straight navigatoin
 * Function Arguments	:
 *						  psx							struct of PlayStation controller
 * 						  rns	 						pointer to RNS_interface_t struct
 * Function Return		: NONE
 * Function Example		: realMODN(ps4, &rns);
 */
void imuRealMODN(PSxBT_t *psx, RNS_interface_t* rns){
	MODN.x_vel =  (psx->joyL_x) * MODN.speed;
	MODN.y_vel =  (psx->joyL_y) * MODN.speed;
	MODN.w_vel = ((psx->joyR_2) - (psx->joyL_2))* MODN.speed;
	RNSEnquire(RNS_COORDINATE_X_Y_Z_Zrad, rns);
	realZrad = rns->enq.enq_buffer[3].data;

	switch(MODN.turnState){
	case NO_TURN:
		if(MODN.w_vel!=0){
			MODN.turnState = START_TURN;
			MODN.imuFeedback = 0;
		}else{
			if(HAL_GetTick()-MODN.t>=MODN.delay){
				float errorRad = realZrad-MODN.radTarget;
				if(fabsf(errorRad)>=MODN.radTol){
					MODN.imuFeedback = (realZrad - MODN.radTarget)*MODN.imuGain;
				}else{
					MODN.imuFeedback = 0.0;
				}
			}else{
				MODN.radTarget = realZrad;
				MODN.imuFeedback = 0.0;
			}
		}
		break;
	case START_TURN:
		if(MODN.w_vel== 0){
			MODN.radTarget = realZrad;
			MODN.t = HAL_GetTick();
			MODN.turnState = NO_TURN;
		}else{
			MODN.imuFeedback = 0.0;
		}
		break;
	}

	if(MODN.base == MODN_FWD_OMNI || MODN.base == MODN_FWD_OMNI_BLDC) {
		MODN.vel1 = MODN.x_vel * cosf(0.7854-realZrad) +  MODN.y_vel * sinf(0.7854-realZrad) + MODN.w_vel * MODN.turnspeed - MODN.imuFeedback;
		MODN.vel2 = MODN.x_vel * cosf(2.3562-realZrad) +  MODN.y_vel * sinf(2.3562-realZrad) - MODN.w_vel * MODN.turnspeed + MODN.imuFeedback;
		MODN.vel3 = MODN.x_vel * cosf(2.3562-realZrad) +  MODN.y_vel * sinf(2.3562-realZrad) + MODN.w_vel * MODN.turnspeed - MODN.imuFeedback;
		MODN.vel4 = MODN.x_vel * cosf(0.7854-realZrad) +  MODN.y_vel * sinf(0.7854-realZrad) - MODN.w_vel * MODN.turnspeed + MODN.imuFeedback;
	}else if (MODN.base == MODN_TRI_OMNI || MODN.base == MODN_TRI_OMNI_BLDC){
		/*			 C
		 * 		   -___+
		 *
		 * 		+		  +
		 * 		A\		 /B
		 * 		  -     -
		 */
		MODN.vel3 = MODN.y_vel*sinf(-realZrad) 		 + MODN.x_vel*cosf(-realZrad)  	    + MODN.w_vel*MODN.turnspeed - MODN.imuFeedback;;
		MODN.vel1 = MODN.y_vel*sinf(2.0944-realZrad) + MODN.x_vel*cosf(2.0944-realZrad) + MODN.w_vel*MODN.turnspeed - MODN.imuFeedback;;
		MODN.vel2 = MODN.y_vel*sinf(1.0472-realZrad) + MODN.x_vel*cosf(1.0472-realZrad) - MODN.w_vel*MODN.turnspeed + MODN.imuFeedback;;
	}
	ApplyMODN(psx, rns);
}

/* Function Name		: movableRealMODN
 * Function Description : Same as realMODN. When operator is needed to move. Operator can set 4 types of orientation
 * 						  as in OrientationMODN_t
 * Function Remarks		: World Real X and Y is set at the moment IMU power up
 * 						  Need some ps4 buttons to set orientation, unless robot know your position by itself
 * Function Arguments	:
 *						  psx							struct of PlayStation controller
 * 						  rns	 						pointer to RNS_interface_t struct
 * Function Return		: NONE
 * Function Example		: realMODN(ps4, &rns);
 */
void movableRealMODN(PSxBT_t *psx, RNS_interface_t* rns)
{
	switch(MODN.orientation){
	case OPERATOR_TURNED_0_DEGREE:
		MODN.x_vel = psx->joyL_x * MODN.speed;
		MODN.y_vel = psx->joyL_y * MODN.speed;
		break;
	case OPERATOR_TURNED_90_DEGREES_CLOCKWISE:
		MODN.x_vel = -psx->joyL_y * MODN.speed;
		MODN.y_vel =  psx->joyL_x * MODN.speed;
		break;
	case OPERATOR_TURNED_180_DEGREES:
		MODN.x_vel = -psx->joyL_x * MODN.speed;
		MODN.y_vel = -psx->joyL_y * MODN.speed;
		break;
	case OPERATOR_TURNED_90_DEGREES_ANTICLOCKWISE:
		MODN.x_vel =  psx->joyL_y * MODN.speed;
		MODN.y_vel = -psx->joyL_x * MODN.speed;
		break;
	}
	MODN.w_vel = ((psx->joyR_2) - (psx->joyL_2))* MODN.speed;
	RNSEnquire(RNS_COORDINATE_X_Y_Z_Zrad, rns);
	realZrad = rns->enq.enq_buffer[3].data;
	if(MODN.base == MODN_FWD_OMNI || MODN.base == MODN_FWD_OMNI_BLDC) {
		MODN.vel1 = MODN.x_vel*cosf(0.7854-realZrad) + MODN.y_vel*sinf(0.7854-realZrad) + MODN.w_vel*MODN.turnspeed;
		MODN.vel2 = MODN.x_vel*cosf(2.3562-realZrad) + MODN.y_vel*sinf(2.3562-realZrad) - MODN.w_vel*MODN.turnspeed;
		MODN.vel3 = MODN.x_vel*cosf(2.3562-realZrad) + MODN.y_vel*sinf(2.3562-realZrad) + MODN.w_vel*MODN.turnspeed;
		MODN.vel4 = MODN.x_vel*cosf(0.7854-realZrad) + MODN.y_vel*sinf(0.7854-realZrad) - MODN.w_vel*MODN.turnspeed;
	}
	//		else if(MODN.base == MODN_MECANUM){
	//		MODN.vel1 = MODN.y_vel*(1.0) + MODN.x_vel*(1.0)  + *(MODN.w_vel)/*(MODN.turnspeed + MODN.e)*/;
	//		MODN.vel2 = MODN.y_vel*(1.0) + MODN.x_vel*(-1.0) - *(MODN.w_vel)/*(MODN.turnspeed + MODN.e)*/;
	//		MODN.vel3 = MODN.y_vel*(1.0) + MODN.x_vel*(-1.0) + *(MODN.w_vel)/*(MODN.turnspeed + MODN.e)*/;
	//		MODN.vel4 = MODN.y_vel*(1.0) + MODN.x_vel*(1.0)  - *(MODN.w_vel)/*(MODN.turnspeed + MODN.e)*/;
	//	}
	else if (MODN.base == MODN_TRI_OMNI || MODN.base == MODN_TRI_OMNI_BLDC){
		/*			 C
		 * 		   -___+
		 *
		 * 		+		  +
		 * 		A\		 /B
		 * 		  -     -
		 */
		MODN.vel3 = MODN.y_vel*sinf(-realZrad) 		 + MODN.x_vel*cosf(-realZrad)  	    + MODN.w_vel*MODN.turnspeed;
		MODN.vel1 = MODN.y_vel*sinf(2.0944-realZrad) + MODN.x_vel*cosf(2.0944-realZrad) + MODN.w_vel*MODN.turnspeed;
		MODN.vel2 = MODN.y_vel*sinf(1.0472-realZrad) + MODN.x_vel*cosf(1.0472-realZrad) - MODN.w_vel*MODN.turnspeed;
	}
	ApplyMODN(psx, rns);
}

void autoorientMODN(PSxBT_t *psx, RNS_interface_t* rns){
	RNSEnquire(RNS_COORDINATE_X_Y_Z_Zrad, rns);
	realZ = rns->enq.enq_buffer[2].data;
	realZrad = rns->enq.enq_buffer[3].data;
	float realheading;
	if(realZ > 0)
		realheading = ((int)(realZ/90))%4;
	else
		realheading = ((int)(fabsf(realZ + 360)/90))%4;
	if(realheading == 0)		MODN.orientation = OPERATOR_TURNED_0_DEGREE;
	else if(realheading == 1)	MODN.orientation = OPERATOR_TURNED_90_DEGREES_ANTICLOCKWISE;
	else if(realheading == 2)	MODN.orientation = OPERATOR_TURNED_180_DEGREES;
	else						MODN.orientation = OPERATOR_TURNED_90_DEGREES_CLOCKWISE;

	switch(MODN.orientation){
	case OPERATOR_TURNED_0_DEGREE:
		MODN.x_vel = psx->joyL_x * MODN.speed;
		MODN.y_vel = psx->joyL_y * MODN.speed;
		break;
	case OPERATOR_TURNED_90_DEGREES_CLOCKWISE:
		MODN.x_vel = -psx->joyL_y * MODN.speed;
		MODN.y_vel =  psx->joyL_x * MODN.speed;
		break;
	case OPERATOR_TURNED_180_DEGREES:
		MODN.x_vel = -psx->joyL_x * MODN.speed;
		MODN.y_vel = -psx->joyL_y * MODN.speed;
		break;
	case OPERATOR_TURNED_90_DEGREES_ANTICLOCKWISE:
		MODN.x_vel =  psx->joyL_y * MODN.speed;
		MODN.y_vel = -psx->joyL_x * MODN.speed;
		break;
	}
	MODN.w_vel = ((psx->joyR_2) - (psx->joyL_2))* MODN.speed;
	if(MODN.base == MODN_FWD_OMNI || MODN.base == MODN_FWD_OMNI_BLDC) {
		MODN.vel1 = MODN.x_vel*cosf(0.7854-realZrad) + MODN.y_vel*sinf(0.7854-realZrad) + MODN.w_vel*MODN.turnspeed;
		MODN.vel2 = MODN.x_vel*cosf(2.3562-realZrad) + MODN.y_vel*sinf(2.3562-realZrad) - MODN.w_vel*MODN.turnspeed;
		MODN.vel3 = MODN.x_vel*cosf(2.3562-realZrad) + MODN.y_vel*sinf(2.3562-realZrad) + MODN.w_vel*MODN.turnspeed;
		MODN.vel4 = MODN.x_vel*cosf(0.7854-realZrad) + MODN.y_vel*sinf(0.7854-realZrad) - MODN.w_vel*MODN.turnspeed;
	}
	//		else if(MODN.base == MODN_MECANUM){
	//		MODN.vel1 = MODN.y_vel*(1.0) + MODN.x_vel*(1.0)  + *(MODN.w_vel)/*(MODN.turnspeed + MODN.e)*/;
	//		MODN.vel2 = MODN.y_vel*(1.0) + MODN.x_vel*(-1.0) - *(MODN.w_vel)/*(MODN.turnspeed + MODN.e)*/;
	//		MODN.vel3 = MODN.y_vel*(1.0) + MODN.x_vel*(-1.0) + *(MODN.w_vel)/*(MODN.turnspeed + MODN.e)*/;
	//		MODN.vel4 = MODN.y_vel*(1.0) + MODN.x_vel*(1.0)  - *(MODN.w_vel)/*(MODN.turnspeed + MODN.e)*/;
	//	}
	else if (MODN.base == MODN_TRI_OMNI || MODN.base == MODN_TRI_OMNI_BLDC){
		/*			 C
		 * 		   -___+
		 *
		 * 		+		  +
		 * 		A\		 /B
		 * 		  -     -
		 */
		MODN.vel3 = MODN.y_vel*sinf(-realZrad) 		 + MODN.x_vel*cosf(-realZrad)  	    + MODN.w_vel*MODN.turnspeed;
		MODN.vel1 = MODN.y_vel*sinf(2.0944-realZrad) + MODN.x_vel*cosf(2.0944-realZrad) + MODN.w_vel*MODN.turnspeed;
		MODN.vel2 = MODN.y_vel*sinf(1.0472-realZrad) + MODN.x_vel*cosf(1.0472-realZrad) - MODN.w_vel*MODN.turnspeed;
	}
	ApplyMODN(psx, rns);
}

void ApplyMODN(PSxBT_t *psx, RNS_interface_t* rns){
	if(fabs(MODN.x_vel) + fabs(MODN.y_vel) + fabs(MODN.w_vel) <= 0.01){
		if(MODN.base == MODN_FWD_OMNI_BLDC || MODN.base == MODN_TRI_OMNI_BLDC){
			RNSSwerveStop(rns);
			if(stopcommand < 66){
				stopcommand++;
				VESCHandBrake(MODN.brakecurr, &vesc);
			}else{
				VESCReleaseMotor(&vesc);
				VESCStop(&vesc);
			}
		}else{
			RNSStop(rns);
		}

	}else{
		if(MODN.base == MODN_FWD_OMNI) {
			RNSVelocity(MODN.vel1, MODN.vel2, MODN.vel3, MODN.vel4, rns);
		}else if(MODN.base == MODN_MECANUM){
			RNSVelocity(MODN.vel1, MODN.vel2, MODN.vel3, MODN.vel4, rns);
		}else if (MODN.base == MODN_TRI_OMNI){
			RNSVelocity(MODN.vel1, MODN.vel2, MODN.vel3, 0.0, rns);
		}else if(MODN.base == MODN_FWD_OMNI_BLDC){
			RNSSwerveVel(MODN.vel1, MODN.vel2, MODN.vel3, MODN.vel4, rns);
			VESCVelocity(MODN.vel1, MODN.vel2, MODN.vel3, MODN.vel4, &vesc);
		}else if(MODN.base == MODN_TRI_OMNI_BLDC){
			RNSSwerveVel(MODN.vel1, MODN.vel2, MODN.vel3, 0.0, rns);
			VESCVelocity(MODN.vel1, MODN.vel2, MODN.vel3, 0.0, &vesc);
		}
		stopcommand = 0;
	}
}

void setOrientationMODN(OrientationMODN_t orientation)
{
	MODN.orientation = orientation;
}

void setSpeedMODN(float speed){  //fucked up, do not use MODN.imuGain eqn!

	MODN.speed = speed;
//	MODN.imuGain = speed*MODN.imuGain/MODN.radTol;
}

void setImuGainMODN(float imuGain){
	MODN.imuGain = MODN.speed*imuGain/MODN.radTol;
}

void setDelayMODN(int ticks){
	MODN.delay=ticks;
}

/*********************************************/

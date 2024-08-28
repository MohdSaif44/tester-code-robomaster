
/*******************************************************************************
 * Title   : Laser (
 * Author  : KLok
 * Version : 1.00
 * Date    : Sept 2020
 *******************************************************************************
 * Description:
 * - Combined with ADC and kalman_filter(KF) to fully used it
 * -
 *
 * Version History:
 * 1.00 by Klok
 * - Basic function of laser calibration
 *
 * 1.1 by Anas
 * - Added Check for distance and way to manually tune
 *
 * Bugs:
 *
 ******************************************************************************/

#ifndef SRC_LASER_LASER_H_
#define SRC_LASER_LASER_H_

#include "../BIOS/bios.h"
#include "../ADC/adc.h"
#include "../KF/KF.h"

/**************************************************
 * 		Structure							  	  *
 *************************************************/
typedef struct{
	uint64_t rawCu;
	float raw;
	uint32_t cnt;
	float min_dist;
	float ratio;
	float rawDist;
	float dist;
	float min_err;
	uint8_t swap;
	KALMANFILTER_t kf;
}Laser_t;

extern ADC_t adc;
/**************************************************
 * 		Function Prototype			  			  *
 *************************************************/

void LaserInit(Laser_t *laser, float max_dist, float min_dist, float kalman_noise, float min_error);
void Laser(Laser_t *laser);
void LaserUpdate(Laser_t *laser, uint8_t channel_no);

#endif /* SRC_LASER_LASER_H_ */

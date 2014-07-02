/*
 * purpins_motors.h
 *
 *  Created on: Jul 2, 2014
 *      Author: bgouveia
 */

#ifndef PURPINS_MOTORS_H_
#define PURPINS_MOTORS_H_

#include <stdint.h>


class PID;

#define PWM_FREQUENCY (20000)
#define QEILOOPFREQUENCY (40)
#define QEIRATE (1.0/QEILOOPFREQUENCY)

#define KC   (2.6)
#define TI    (0.0)
#define TD    (0.0)



typedef struct _motor{
	PID * pid;
	int32_t vel;
	int32_t goal_vel;
	int32_t pwm_value;
}PP_MOTOR;





#endif /* PURPINS_MOTORS_H_ */

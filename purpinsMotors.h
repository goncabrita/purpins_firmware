/*
 * purpins_motors.h
 *
 *  Created on: Jul 2, 2014
 *      Author: bgouveia
 */

#ifndef PURPINS_MOTORS_H_
#define PURPINS_MOTORS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>


class PID;

/**
 * Motorcontrol Frequency constants
 */

#define PWM_FREQUENCY (20000)
#define QEILOOPFREQUENCY (40)
#define QEIRATE (1.0/QEILOOPFREQUENCY)

/**
 * PID constants
 */
#define KC   (2.6)
#define TI    (0.0)
#define TD    (0.0)

typedef struct _motor{
	PID * pid;
	int32_t vel; //measured speed
	int32_t goal_vel; //requested speed
	int32_t pwm_value; //Corresponding speed
}PP_MOTOR;


void ppMotorsControlInit();
void ppMotorsSetSpeed(float leftSpeed,float rightSpeed);

void ppMotorsLeftQEIHandler();
void ppMotorsRightQEIHandler();

#ifdef __cplusplus
}
#endif

#endif /* PURPINS_MOTORS_H_ */

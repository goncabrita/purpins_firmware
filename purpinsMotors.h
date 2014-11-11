/*
 * purpins_motors.h
 *
 *  Created on: Jul 2, 2014
 *      Author: bgouveia
 */

#ifndef PURPINS_MOTORS_H_
#define PURPINS_MOTORS_H_

/** @file */

/**
 * @defgroup MOTOR_GROUP Motor control constants
 *
 * @{
 */

#include <stdint.h>

class PID;

//MotorControl constants

#define PWM_FREQUENCY (20000)           /**< motor pwm frequency in Hz */
#define QEILOOPFREQUENCY (40)           /**< QEI loop frequency in Hz */
#define QEIRATE (1.0/QEILOOPFREQUENCY)  /**< QEI loop interval in seconds */


//PID constants

#define KC    (2.6)  /**< proportional tuning constant*/
#define TI    (0.0)  /**<integral tuning constant*/
#define TD    (0.0)  /**<derivative tuning constant*/

/** @} */

/**
 * pp_motor structure used for convenience with all the motor variables
 */
typedef struct _motor{
	PID * pid;
	int32_t vel; //measured speed
	int32_t goal_vel; //requested speed
	int32_t pwm_value; //Corresponding speed
	int32_t pulses; //ticks counted by the QEI
}pp_motor;


/**
 * Class with all the functions to control and get
 * the information about the 2 Motors
 *
 * \author Bruno Duarte Gouveia
 * \date Sep 16, 2014
 */
class purpinsMotors {

public:
    /**
     * Constructor.
     */
	purpinsMotors();

    /**
     * Set the goal speeds for the two motors
     *
     * @param leftSpeed Left goal motor speed in m/s
     * @param rightSpeed Right goal motor speed in m/s
     */
	void setSpeed(float leftSpeed,float rightSpeed);

	/**
     * Get the current Motor Speeds in m/s
     *
     *  @param leftSpeed Left motor speed in m/s
     *  @param rightSpeed Right motor speed in m/s
     */
	void getSpeed(float & leftSpeed,float & rightSpeed);

	/**
	 * Set the PWM for the two motors
	 *
	 * @param leftPWM Left goal motor PWM
	 * @param rightPWM Right goal motor PWM
	 */
	void setPWM(int leftPWM, int rightPWM);

	/**
     * Get the QEI ticks for the two motors
     *
     * @param left Left encoder ticks
     * @param right Right encoder ticks
     */
	void getQEITicks(int32_t & left, int32_t & right);

private:
	pp_motor leftMotor;
	pp_motor rightMotor;

	/**
     * Configure the QEI
     *
     */
	void configureQEI();

	/**
     * Configure the motor PWM
     *
     */
	void configurePWM();

};



extern "C"{
void motorsLeftQEIHandler();
void motorsRightQEIHandler();
}

#endif /* PURPINS_MOTORS_H_ */

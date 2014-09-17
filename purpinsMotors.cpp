/*
 * purpins_motors.c
 *
 *  Created on: Jul 2, 2014
 *      Author: bgouveia
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/qei.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "utils/uartstdio.h"

#include "purpinsMotors.h"

#include "pid.h"


static pp_motor * leftptr;
static pp_motor * rightptr;
static unsigned long pwmPeriod;

purpinsMotors::purpinsMotors() {

	leftptr=&leftMotor;
	rightptr=&rightMotor;

	PID leftmotor_pid(KC,TI,TD,QEIRATE);
	PID rightmotor_pid(KC,TI,TD,QEIRATE);

	leftMotor.pid=&leftmotor_pid;
	rightMotor.pid=&rightmotor_pid;


	configureQEI();
}

void purpinsMotors::setSpeed(float leftSpeed, float rightSpeed) {
}

void purpinsMotors::getSpeed(float& leftSpeed, float& rightSpeed) {

}

void purpinsMotors::getQEITicks(int32_t& left, int32_t& right) {
}

void purpinsMotors::configureQEI() {

	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	//
	// Set GPIO C5 and C6 as QEI pins.
	//
	MAP_GPIOPinConfigure(GPIO_PC5_PHA1);
	MAP_GPIOPinConfigure(GPIO_PC6_PHB1);
	MAP_GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6);

	//
	// Set GPIO D6 and D7 as QEI pins.
	//
	MAP_GPIOPinConfigure(GPIO_PD6_PHA0);
	MAP_GPIOPinConfigure(GPIO_PD7_PHB0);
	MAP_GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);

	MAP_QEIConfigure(QEI0_BASE,
			(QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP
					| QEI_CONFIG_NO_RESET), 1200);
	MAP_QEIConfigure(QEI1_BASE,
			(QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP
					| QEI_CONFIG_NO_RESET), 1200);

	MAP_QEIEnable(QEI0_BASE);
	MAP_QEIEnable(QEI1_BASE);

	MAP_QEIVelocityConfigure(QEI0_BASE, QEI_VELDIV_1,
			MAP_SysCtlClockGet() / QEILOOPFREQUENCY);
	MAP_QEIVelocityConfigure(QEI1_BASE, QEI_VELDIV_1,
			MAP_SysCtlClockGet() / QEILOOPFREQUENCY);

	QEIIntRegister(QEI0_BASE, motorsRightQEIHandler);
	QEIIntRegister(QEI1_BASE, motorsLeftQEIHandler);

	MAP_IntEnable(INT_QEI0);
	MAP_IntEnable(INT_QEI1);

	MAP_QEIIntEnable(QEI0_BASE, QEI_INTTIMER);
	MAP_QEIIntEnable(QEI1_BASE, QEI_INTTIMER);
	MAP_QEIVelocityEnable(QEI0_BASE);
	MAP_QEIVelocityEnable(QEI1_BASE);
}



void purpinsMotors::configurePWM(){

	//TODO: configure the right pwm modules and pins

	//Configure PWM Clock
	MAP_SysCtlPWMClockSet(SYSCTL_PWMDIV_2);

	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

	//PD6 & PD7 for direction
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE,GPIO_INT_PIN_6 |GPIO_INT_PIN_7);
	MAP_GPIOPinWrite(GPIO_PORTD_BASE,GPIO_INT_PIN_6 |GPIO_INT_PIN_7,0);

	pwmPeriod = MAP_SysCtlClockGet() / 2 / PWM_FREQUENCY; //PWM frequency


	MAP_GPIOPinConfigure(GPIO_PF1_M1PWM5);
	MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);

	MAP_PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

	//Set the Period (expressed in clock ticks)
	MAP_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, pwmPeriod);

	//Set PWM duty-0%
	MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5 , 0);

	// Enable the PWM generator
	MAP_PWMGenEnable(PWM1_BASE, PWM_GEN_2);

	// Turn on the Output pins
	MAP_PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, true);
}



void motorsLeftQEIHandler() {

	MAP_QEIIntClear(QEI1_BASE,QEI_INTTIMER);


	leftptr->vel=MAP_QEIVelocityGet(QEI1_BASE)*MAP_QEIDirectionGet(QEI1_BASE);

	leftptr->pid->setSetPoint(leftptr->goal_vel);
	leftptr->pid->setProcessValue(leftptr->vel);
	leftptr->pwm_value = leftptr->pid->compute()*(pwmPeriod/255);

	//pwm_value = leftmotor_pid.run(goal_vel,vel)*(ulPeriod/255);

	//TODO: change to corresponding motor pwm pins
	if (leftptr->pwm_value == 0){
		MAP_GPIOPinWrite(GPIO_PORTD_BASE,GPIO_INT_PIN_6 |GPIO_INT_PIN_7,0);
	}else if (leftptr->pwm_value<0){
		MAP_GPIOPinWrite(GPIO_PORTD_BASE,GPIO_INT_PIN_6 |GPIO_INT_PIN_7,GPIO_INT_PIN_6);
		MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5 , -leftptr->pwm_value);
	}else {
		MAP_GPIOPinWrite(GPIO_PORTD_BASE,GPIO_INT_PIN_6 |GPIO_INT_PIN_7,GPIO_INT_PIN_7);
		MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5 , leftptr->pwm_value);
	}

}

void motorsRightQEIHandler() {

	MAP_QEIIntClear(QEI0_BASE,QEI_INTTIMER);


	rightptr->vel=MAP_QEIVelocityGet(QEI1_BASE)*MAP_QEIDirectionGet(QEI1_BASE);

	rightptr->pid->setSetPoint(rightptr->goal_vel);
	rightptr->pid->setProcessValue(rightptr->vel);
	rightptr->pwm_value = rightptr->pid->compute()*(pwmPeriod/255);

	//pwm_value = leftmotor_pid.run(goal_vel,vel)*(ulPeriod/255);


	//TODO: change to corresponding motor pwm pins
	if (rightptr->pwm_value == 0){
		MAP_GPIOPinWrite(GPIO_PORTD_BASE,GPIO_INT_PIN_6 |GPIO_INT_PIN_7,0);
	}else if (rightptr->pwm_value<0){
		MAP_GPIOPinWrite(GPIO_PORTD_BASE,GPIO_INT_PIN_6 |GPIO_INT_PIN_7,GPIO_INT_PIN_6);
		MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5 , -rightptr->pwm_value);
	}else {
		MAP_GPIOPinWrite(GPIO_PORTD_BASE,GPIO_INT_PIN_6 |GPIO_INT_PIN_7,GPIO_INT_PIN_7);
		MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5 , rightptr->pwm_value);
	}

}

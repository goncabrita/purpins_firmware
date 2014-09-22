/*
 * main.cpp
 *
 *  Created on: Apr 10, 2014
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

#include "pid.h"

#include <string>


#define PWM_FREQUENCY 20000
#define QEILOOPFREQUENCY 40
#define QEIRATE 1.0/QEILOOPFREQUENCY

#define Kc   -2.6
#define Ti    0.0
#define Td    0.0

unsigned long ulPeriod;

int32_t vel=0;
int32_t goal_vel=0;
int32_t pwm_value=0;
PID leftmotor_pid(Kc,Ti,Td,QEIRATE);

void QEIVelIntHandler();
void configurePWM(void);
void configureUART();
void configureQEI();


int main(){

	unsigned long ulClockMS=0;

	//
	// Enable lazy stacking for interrupt handlers.  This allows floating-point
	// instructions to be used within interrupt handlers, but at the expense of
	// extra stack usage.
	//
	MAP_FPUEnable();
	MAP_FPULazyStackingEnable();

	//
	// Set the clocking to run directly from the crystal.
	//
	MAP_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
			SYSCTL_XTAL_16MHZ);

	MAP_IntMasterDisable();

	configureQEI();
	configurePWM();

	/*leftmotor_pid.setMaxOutput(255);
	leftmotor_pid.setMinOutput(-255);
	leftmotor_pid.setGains(12.0,5.0,0.0);
	leftmotor_pid.setSampleTime(1.0/QEILOOPFREQUENCY);*/

	leftmotor_pid.setInputLimits(-100,100);
	leftmotor_pid.setOutputLimits(-255,255);
    leftmotor_pid.setBias(0.0);
    leftmotor_pid.setMode(AUTO_MODE);


	configureUART();

	MAP_IntMasterEnable();


	// Get the current processor clock frequency.
	ulClockMS = MAP_SysCtlClockGet() / (3 * 1000);
	int char_counter=0;
	char inputbuffer[10];

	while(1){

		inputbuffer[char_counter]=UARTgetc();
		char_counter++;

		if (char_counter==3){
			inputbuffer[3]=0;
			goal_vel=atoi(inputbuffer);
			char_counter=0;
		}


		//uint32_t pos = MAP_QEIPositionGet(QEI1_BASE);
		/*uint32_t vel = MAP_QEIVelocityGet(QEI1_BASE);
		//UARTprintf("pos : %u \n",pos);QEIDirectionGet(QEI1_BASE)*/

		/*UARTprintf("vel : %d pwm %d\n",vel,pwm_value);
		MAP_SysCtlDelay(ulClockMS*50);*/

	}
	return 0;
}


//PWM = PF1/M1PWM5 & PC4/M0PWM6
void configurePWM(void)
{



	//Configure PWM Clock to match system
	MAP_SysCtlPWMClockSet(SYSCTL_PWMDIV_2);

	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

	//PD6 & PD7 for direction
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE,GPIO_INT_PIN_6 |GPIO_INT_PIN_7);
	MAP_GPIOPinWrite(GPIO_PORTD_BASE,GPIO_INT_PIN_6 |GPIO_INT_PIN_7,0);

	ulPeriod = MAP_SysCtlClockGet() / 2 / PWM_FREQUENCY; //PWM frequency


	MAP_GPIOPinConfigure(GPIO_PF1_M1PWM5);
	MAP_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);

	MAP_PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

	//Set the Period (expressed in clock ticks)
	MAP_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, ulPeriod);

	//Set PWM duty-0%
	MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5 , 0);

	// Enable the PWM generator
	MAP_PWMGenEnable(PWM1_BASE, PWM_GEN_2);

	// Turn on the Output pins
	MAP_PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT, true);
}

void configureUART(){
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	//
	// Set GPIO A0 and A1 as UART pins.
	//
	MAP_GPIOPinConfigure(GPIO_PA0_U0RX);
	MAP_GPIOPinConfigure(GPIO_PA1_U0TX);
	MAP_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTStdioConfig(0,115200,MAP_SysCtlClockGet());
}


void configureQEI(){

	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

	//
	// Set GPIO C5 and C6 as QEI pins.
	//
	MAP_GPIOPinConfigure(GPIO_PC5_PHA1);
	MAP_GPIOPinConfigure(GPIO_PC6_PHB1);
	MAP_GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 |  GPIO_PIN_6);

	MAP_QEIConfigure(QEI1_BASE,(QEI_CONFIG_CAPTURE_A_B | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP | QEI_CONFIG_NO_RESET),1200);
	MAP_QEIEnable(QEI1_BASE);

	MAP_QEIVelocityConfigure(QEI1_BASE, QEI_VELDIV_1, MAP_SysCtlClockGet()/QEILOOPFREQUENCY);

	QEIIntRegister(QEI1_BASE,QEIVelIntHandler);
	MAP_IntEnable(INT_QEI1);
	MAP_QEIIntEnable(QEI1_BASE,QEI_INTTIMER);

	MAP_QEIVelocityEnable(QEI1_BASE);
}

void QEIVelIntHandler(){
	MAP_QEIIntClear(QEI1_BASE,QEI_INTTIMER);

	vel = MAP_QEIVelocityGet(QEI1_BASE)*MAP_QEIDirectionGet(QEI1_BASE);

	leftmotor_pid.setSetPoint(goal_vel);
	leftmotor_pid.setProcessValue(vel);
	pwm_value = leftmotor_pid.compute()*(ulPeriod/255);

	//pwm_value = leftmotor_pid.run(goal_vel,vel)*(ulPeriod/255);

	if (pwm_value == 0){
		MAP_GPIOPinWrite(GPIO_PORTD_BASE,GPIO_INT_PIN_6 |GPIO_INT_PIN_7,0);
	}else if (pwm_value<0){
		MAP_GPIOPinWrite(GPIO_PORTD_BASE,GPIO_INT_PIN_6 |GPIO_INT_PIN_7,GPIO_INT_PIN_6);
		MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5 , -pwm_value);
	}else {
		MAP_GPIOPinWrite(GPIO_PORTD_BASE,GPIO_INT_PIN_6 |GPIO_INT_PIN_7,GPIO_INT_PIN_7);
		MAP_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5 , pwm_value);
	}
	UARTprintf("%d ; %d ; %d\n",goal_vel,vel,pwm_value);
}



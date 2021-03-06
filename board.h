//*****************************************************************************
//
// board.h - Board definitions for EK-TM4C123GXL Tiva C Series LaunchPad.
//
// Copyright (c) 2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.1.0.12573 of the Tiva Firmware Development Package.
//
//*****************************************************************************
#ifndef __BOARD_H__
#define __BOARD_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// The desired system tick frequency.Defines for setting up the system tick.
//
//*****************************************************************************
#define SYSTICK_PER_SECOND            10

//*****************************************************************************
//
// The desired system clock rate.
//
//*****************************************************************************
#define SYSCLOCK_RATE_HZ                50000000

//*****************************************************************************
//
// The parameter to pass to SysCtlDelay() to cause a delay of approximately
// 50 microseconds.
//
//*****************************************************************************
#define DELAY_50_MICROSECONDS           ((SYSCLOCK_RATE_HZ / 20000) / 3)

//*****************************************************************************
//
// SPI-related definitions.
//
//*****************************************************************************

//
// SPI General Defines
//
#define SPI_WINDOW_SIZE                 DMA_WINDOW_SIZE
#define FWT_DELAY                       4000
#define DMA_WINDOW_SIZE                 1024

//
// CC3000 Board specific Macros
//
#define ASSERT_CS()          (MAP_GPIOPinWrite(SPI_CS_PORT, SPI_CS_PIN, 0))
#define DEASSERT_CS()        (MAP_GPIOPinWrite(SPI_CS_PORT, SPI_CS_PIN, 0xFF))

//
// IRQ settings PC7
//
#define SYSCTL_PERIPH_IRQ_PORT          SYSCTL_PERIPH_GPIOC
#define SPI_GPIO_IRQ_BASE               GPIO_PORTC_BASE
#define SPI_IRQ_PIN                     GPIO_PIN_7
#define INT_GPIO_SPI                    INT_GPIOC
#define INT_SPI                         INT_SSI3

//
// SW EN settings PE3
//
#define SYSCTL_PERIPH_SW_EN_PORT        SYSCTL_PERIPH_GPIOE
#define SPI_GPIO_SW_EN_BASE             GPIO_PORTE_BASE
#define SPI_EN_PIN                      GPIO_PIN_3

//
// CS settings PD1
//
#define SPI_CS_PORT                     GPIO_PORTD_BASE
#define SPI_CS_PIN                      GPIO_PIN_1
#define SYSCTL_PERIPH_SPI_PORT          SYSCTL_PERIPH_GPIOD

//
// SPI Hardware Abstraction layer
//
#define SPI_BASE                        SSI3_BASE
#define SPI_CLK_PIN                     GPIO_PIN_0
#define SPI_RX_PIN                      GPIO_PIN_2
#define SPI_TX_PIN                      GPIO_PIN_3

#define SYSCTL_PERIPH_SPI               SYSCTL_PERIPH_SSI3
#define SYSCTL_PERIPH_SPI_BASE          SYSCTL_PERIPH_GPIOD

#define SPI_PORT                        GPIO_PORTD_BASE
#define SPI_CLK_MUX_SEL                 GPIO_PD0_SSI3CLK
#define SPI_RX_MUX_SEL                  GPIO_PD2_SSI3RX
#define SPI_TX_MUX_SEL                  GPIO_PD3_SSI3TX

#define SPI_UDMA_RX_CHANNEL             UDMA_CH14_SSI3RX
#define SPI_UDMA_TX_CHANNEL             UDMA_CH15_SSI3TX

//*****************************************************************************
//
// UART definitions.
//
//*****************************************************************************
#define CC3000_UART_BASE                UART0_BASE
#define CC3000_UART_SYSCTL_PERIPH_GPIO  SYSCTL_PERIPH_GPIOA
#define CC3000_UART_SYSCTL_PERIPH_UART  SYSCTL_PERIPH_UART0
#define CC3000_UART_GPIO_RX             GPIO_PA0_U0RX
#define CC3000_UART_GPIO_TX             GPIO_PA1_U0TX
#define CC3000_UART_GPIO_PORT_BASE      GPIO_PORTA_BASE
#define CC3000_UART_GPIO_PINS           (GPIO_PIN_0 | GPIO_PIN_1)
#define CC3000_UART_INT                 INT_UART0

//*****************************************************************************
//
// LED connection definitions.
//
//*****************************************************************************
#define CC3000_LED_RED_SYSCTL_PERIPH    SYSCTL_PERIPH_GPIOF
#define CC3000_LED_RED_PORT             GPIO_PORTF_BASE
#define CC3000_LED_RED_PIN              GPIO_PIN_1

#define CC3000_LED_BLUE_SYSCTL_PERIPH   SYSCTL_PERIPH_GPIOF
#define CC3000_LED_BLUE_PORT            GPIO_PORTF_BASE
#define CC3000_LED_BLUE_PIN             GPIO_PIN_2

#define CC3000_LED_GREEN_SYSCTL_PERIPH  SYSCTL_PERIPH_GPIOF
#define CC3000_LED_GREEN_PORT           GPIO_PORTF_BASE
#define CC3000_LED_GREEN_PIN            GPIO_PIN_3

//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************
extern void pio_init(void);
extern long ReadWlanInterruptPin(void);
extern void WlanInterruptEnable(void);
extern void WlanInterruptDisable(void);
extern void WriteWlanPin( unsigned char val );

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif //__BOARD_H__

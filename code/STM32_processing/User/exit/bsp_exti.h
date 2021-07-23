#ifndef __EXTI_H
#define	__EXTI_H

#include "stm32f4xx.h"

//Òý½Å¶¨Òå
/*******************************************************/
#define INT_GPIO_PORT                GPIOA
#define INT_GPIO_CLK                 RCC_AHB1Periph_GPIOA
#define INT_GPIO_PIN                 GPIO_Pin_3
#define INT_EXTI_PORTSOURCE          EXTI_PortSourceGPIOA
#define INT_EXTI_PINSOURCE           EXTI_PinSource3
#define INT_EXTI_LINE                EXTI_Line3
#define INT_EXTI_IRQ                 EXTI3_IRQn

#define IRQHandler                   EXTI3_IRQHandler
/*******************************************************/


void EXTI_Config(void);

#endif /* __EXTI_H */

/*
 * stm32f05xx_gpio_driver.h
 *
 *  Created on: Jun 2, 2022
 *      Author: batuhansesli
 */

#ifndef STM32F05XX_GPIO_DRIVER_H_
#define STM32F05XX_GPIO_DRIVER_H_

#include "stm32f05xx.h"


/*
 * This is configuration structure for GPIO
 */

typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_AltFuncMode;
}GPIO_PinConfig_t;


/*!
 *  This is handle structure for GPIP
 */
typedef struct
{
	GPIO_RegDef_t *pGPIOx; 					/* Holds base address of GPIO pin*/
	GPIO_PinConfig_t GPIO_PinConfig;		/* Holds configuration structure*/
}GPIO_Handle_t;



#endif /* STM32F05XX_GPIO_DRIVER_H_ */

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


/*******************
 *  FUNCTION PROTOTYPES FOR GPIO DRIVER
 *******************/


/* GPIO init and DeInit */
void GPIO_Init(GPIO_Handle_t *pGPIOx);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/* GPIO Peripheral Clock Setup*/
void GPIO_PeripClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/* Read and write Data */   
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Data);
void GPIO_WriteToInputPort(GPIO_RegDef_t *pGPIOx, uint16_t Data);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/* IRQ Handling */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandle(uint8_t PinNumber);











#endif /* STM32F05XX_GPIO_DRIVER_H_ */

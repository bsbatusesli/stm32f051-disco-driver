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
	uint8_t GPIO_PinNumber;					/* possible values: @ */
	uint8_t GPIO_PinMode;					/* possible values: @GPIO_PIN_MODES */
	uint8_t GPIO_PinSpeed;					/* possible values: @GPIO_SPEED_VALUES */
	uint8_t GPIO_PinPuPdControl;			/* possible values: @GPIO_PUPD */
	uint8_t GPIO_PinOPType;					/* possible values: @GPIO_OUTPUT_TYPES */
	uint8_t GPIO_AltFuncMode;				/* possible values: @ */
}GPIO_PinConfig_t;


/*!
 *  This is handle structure for GPIO
 */
typedef struct
{
	GPIO_RegDef_t *pGPIOx; 					/* Holds base address of GPIO pin*/
	GPIO_PinConfig_t GPIO_PinConfig;	/* Holds configuration structure*/
}GPIO_Handle_t;


/***********************************************************************************************
 *  								FUNCTION PROTOTYPES FOR GPIO DRIVER
 ***********************************************************************************************/


/* GPIO init and DeInit */
void GPIO_Init(GPIO_Handle_t *pGPIO_Handle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/* GPIO Peripheral Clock Setup*/
void GPIO_PeripClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/* Read and write Data */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Data);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Data);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/* IRQ Handling */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi);
void GPIO_IRQHandle(uint8_t PinNumber);


/***********************************************************************************************
 *  								Possible Register Macros
 ***********************************************************************************************/

// @GPIO_PIN_MODES
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_IT_FT			4
#define GPIO_IT_RT			5
#define GPIO_IT_RFT			6

// @GPIO_OUTPUT_TYPES
#define GPIO_OT_PP			0
#define GPIO_OT_OD			1

// @GPIO_SPEED_VALUES
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM 	1
#define GPIO_SPEED_HIGH		3

// @GPIO_PUPD
#define GPIO_NO_PUPD		0
#define GPIO_PU				1
#define GPIP_PD				2

// @GPIO_PIN_NO
#define GPIO_PIN_0			0
#define GPIO_PIN_1			1
#define GPIO_PIN_2			2
#define GPIO_PIN_3			3
#define GPIO_PIN_4			4
#define GPIO_PIN_5			5
#define GPIO_PIN_6			6
#define GPIO_PIN_7			7
#define GPIO_PIN_8			8
#define GPIO_PIN_9			9
#define GPIO_PIN_10			10
#define GPIO_PIN_11			11
#define GPIO_PIN_12			12
#define GPIO_PIN_13			13
#define GPIO_PIN_14			14
#define GPIO_PIN_15			15



#endif /* STM32F05XX_GPIO_DRIVER_H_ */

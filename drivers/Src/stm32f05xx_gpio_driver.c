/*
 * stm32f05xx_gpio_driver.c
 *
 *  Created on: Jun 2, 2022
 *      Author: batuhansesli
 */


#include "stm32f05xx_gpio_driver.h"




/* ---------------------------------------------------------------------------
 * @name: 			GPIO_Init
 * @desc:			Initialize the GPIO of given port base address
 *
 * @in[pGPIOx]:		pointer which holds base address of GPIO
 * @returns: 		none
 * --------------------------------------------------------------------------*/
void GPIO_Init(GPIO_Handle_t *pGPIO_Handle)
{

	// enable the peripheral clock
	GPIO_PeripClockControl(pGPIO_Handle->pGPIOx, ENABLE);

	uint32_t temp = 0;
	//1. Config mode of gpio
	if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIO_Handle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIO_Handle->pGPIOx->MODER |= temp;
		temp = 0;
	}else
	{

		if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			//1. configure the FTSR
			EXTI->FTSR |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);


		}else if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			//1.configure the RTSR
			EXTI->FTSR &= ~(1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//1 configure both FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t arr_index = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t reg_index = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber % 4;
		SYSCFG_PCLK_EN();
		uint8_t port_code = GPIO_BASEADDR_TO_PORT_CODE(pGPIO_Handle->pGPIOx);
		SYSCFG->EXTICR[arr_index] &= ~(0x7 << 4*reg_index);
		SYSCFG->EXTICR[arr_index] |= (port_code << 4*reg_index);



		//3. Enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);

	}
	//2. Set speed of port

	temp = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->OSPEEDR &= ~( 0x3 << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIO_Handle->pGPIOx->OSPEEDR |= temp;

	temp = 0;

	//3. Config oullup/pulldown setting
	temp = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinPuPdControl << (2*pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIO_Handle->pGPIOx->PUPDR &= ~( 0x3 << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIO_Handle->pGPIOx->PUPDR |= temp;
	temp = 0;

	//4. Config optype
	temp = (pGPIO_Handle->GPIO_PinConfig.GPIO_PinOPType << pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIO_Handle->pGPIOx->OTYPER &= ~( 0x3 << (2 * pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIO_Handle->pGPIOx->OTYPER |= temp;
	temp = 0;
	//5. Config alternate function mode
	if(pGPIO_Handle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t arr_index, reg_index;
		arr_index = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber / 8; // if 1 -> Alternate high reg, 0-> low reg
		reg_index = pGPIO_Handle->GPIO_PinConfig.GPIO_PinNumber % 8; // gives register index

		pGPIO_Handle->pGPIOx->AFR[arr_index] = (pGPIO_Handle->GPIO_PinConfig.GPIO_AltFuncMode << (4 * reg_index));
	}

}

/* ---------------------------------------------------------------------------
 * @name: 			GPIO_Init
 * @desc:			Deinitialize the GPIO of given port base address
 *
 * @in[pGPIOx]:		pointer which holds base address of GPIO
 * @returns: 		none
 * --------------------------------------------------------------------------*/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
			if(pGPIOx == GPIOA)
				GPIOA_REG_RESET();

			else if(pGPIOx == GPIOB)
				GPIOB_REG_RESET();

			else if(pGPIOx == GPIOC)
				GPIOC_REG_RESET();

			else if(pGPIOx ==  GPIOD)
				GPIOD_REG_RESET();

			else if(pGPIOx ==  GPIOE)
				GPIOE_REG_RESET();

			else if(pGPIOx == GPIOF)
				GPIOF_REG_RESET();

}


/* GPIO Peripheral Clock Setup
 * ---------------------------------------------------------------------------
 * @name: 			GPIO_PeripClockControl
 * @desc:			Enable/Disable Peripheral clock given port
 *
 * @in[pGPIOx]:		pointer which holds base address of GPIO
 * @in[EnorDi]:     Enable/Disable
 * @returns: 		none
 * --------------------------------------------------------------------------*/
void GPIO_PeripClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
			GPIOA_PCLK_EN();

		else if(pGPIOx == GPIOB)
			GPIOB_PCLK_EN();

		else if(pGPIOx == GPIOC)
			GPIOC_PCLK_EN();

		else if(pGPIOx ==  GPIOD)
			GPIOD_PCLK_EN();

		else if(pGPIOx ==  GPIOE)
			GPIOE_PCLK_EN();

		else if(pGPIOx == GPIOF)
			GPIOF_PCLK_EN();
	}
	else if(EnorDi == DISABLE)
	{
		if(pGPIOx == GPIOA)
			GPIOA_PCLK_DI();

		else if(pGPIOx == GPIOB)
			GPIOB_PCLK_DI();

		else if(pGPIOx == GPIOC)
			GPIOC_PCLK_DI();

		else if(pGPIOx ==  GPIOD)
			GPIOD_PCLK_DI();

		else if(pGPIOx ==  GPIOE)
			GPIOE_PCLK_DI();

		else if(pGPIOx == GPIOF)
			GPIOF_PCLK_DI();

	}
}

/* Read and write Data   
 * ---------------------------------------------------------------------------
 * @name: 			GPIO_ReadFromInputPin
 * @desc:			Reads input value from given pin number
 *
 * @in[pGPIOx]:		pointer which holds base address of GPIO
 * @in[PinNumber]:	desired pin number to read
 * @returns: 		read value
 * --------------------------------------------------------------------------*/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (pGPIOx->IDR >> PinNumber) & 0x1; // mask other values
	return value;
}

/* ---------------------------------------------------------------------------
 * @name: 			GPIO_ReadFromInputPort
 * @desc:			Reads input value from given input port
 *
 * @in[pGPIOx]:		pointer which holds base address of GPIO
 * @returns: 		Current values in pins of given port
 * --------------------------------------------------------------------------*/
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = pGPIOx->IDR;
	return value;
}

/* ---------------------------------------------------------------------------
 * @name: 			GPIO_WriteToOutputPin
 * @desc:			Write the data to the given pin number
 *
 * @in[pGPIOx]:		pointer which holds base address of GPIO
 * @in[PinNumber]:	desired pin number to read
 * @in[data]:		8bit data which desired to written
 * @returns: 		none
 * --------------------------------------------------------------------------*/
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Data)
{
	if(Data % 2 == 1)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}

}

/* ---------------------------------------------------------------------------
 * @name: 			GPIO_WriteToOutputPin
 * @desc:			Write the value to all pins in given GPIO port
 *
 * @in[pGPIOx]:		pointer which holds base address of GPIO
 * @in[PinNumber]:	desired pin number to read
 * @in[data]:		16bit data which desired to written bit position indicates respected pin
 * @returns: 		none
 * --------------------------------------------------------------------------*/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Data)
{
	pGPIOx->ODR = Data;
}


/* ---------------------------------------------------------------------------
 * @name: 			GPIO_ToggleOutputPin
 * @desc:			Toggles pin which is given in pinNumber
 *
 * @in[pGPIOx]:		pointer which holds base address of GPIO
 * @in[PinNumber]:	desired pin number to toggle
 * @returns: 		none
 * --------------------------------------------------------------------------*/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

	pGPIOx->ODR ^= (1 << PinNumber);
}

/* IRQ Handling */
/* ---------------------------------------------------------------------------
 * @name: 			GPIO_IRQConfig
 * @desc:			Configration function to Interrupt service routine
 *
 * @in[IRQNumber]: 		Position in vector table
 * @in[IRQPriority]:
 * @in[EnorDi]:
 * @returns: 		none
 * --------------------------------------------------------------------------*/
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{


	if( EnorDi == ENABLE)
	{
		*NVIC_ISER |= (1 << IRQNumber);
	}else
	{
		*NVIC_ICER |= (1 << IRQNumber);
	}
}


/* IRQ Handling */
/* ---------------------------------------------------------------------------
 * @name: 			GPIO_IRQConfig
 * @desc:			Configration function to Interrupt service routine
 *
 * @in[IRQNumber]: 		Position in vector table
 * @in[IRQPriority]:
 * @in[EnorDi]:
 * @returns: 		none
 * --------------------------------------------------------------------------*/
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

	uint8_t ipr_index = IRQNumber / 4;
	uint8_t ipr_offset = IRQNumber % 4;

	uint8_t shift_amount = (8 * ipr_offset) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASEADDR + ipr_index) |= (IRQPriority << shift_amount); // Just adding ipr_index because nvic pointer is 32 bit. Adding 1 is enough for step up 32 bit
}

/* ---------------------------------------------------------------------------
 * @name: 			GPIO_IRQHandle
 * @desc:
 *
 * @in[PinNumber]:
 * @returns: 		none
 * --------------------------------------------------------------------------*/
void GPIO_IRQHandle(uint8_t PinNumber)
{
	// if corresponding PR register is 1
	if(EXTI->PR & (1 << PinNumber))
	{
		// clear pr register by writing 1.
		EXTI->PR |= (1 << PinNumber);
	}
}



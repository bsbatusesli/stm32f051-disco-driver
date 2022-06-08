/*
 * stm32f05xx.h
 *
 *  Created on: Jun 1, 2022
 *      Author: batuhansesli
 */

#ifndef STM32F05XX_H_
#define STM32F05XX_H_

#include <stdint.h>

#define __vo volatile

/*
 * SRAM and FLASH adresses
  */

#define FLASH_BASEADDR				0x08000000U
#define SRAM1_BASEADDR				0x20000000U
#define ROM 						0x1FFFEC00U
#define SRAM 						SRAM1_BASEADDR

/*
 * AHB AND APB Peripherals
 */

#define PERIPH_BASEADDR 			0x40010000U
#define APB_BASEADDR 				0x40010000U
#define AHB1_BASEADDR 				0x40020000U
#define AHB2_BASEADDR 				0x48000000U

/*
 *  AHB2 PERIPHERALS
 */

#define GPIOA_BASEADDR 				(AHB2_BASEADDR + 0x0000)
#define GPIOB_BASEADDR 				(AHB2_BASEADDR + 0x0400)
#define GPIOC_BASEADDR 				(AHB2_BASEADDR + 0x0800)
#define GPIOD_BASEADDR 				(AHB2_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR 				(AHB2_BASEADDR + 0x1000)
#define GPIOF_BASEADDR 				(AHB2_BASEADDR + 0x1400)

/*
 * AHB1 PERIPHERALS
 */

#define DMA1_BASEADDR				(AHB1_BASEADDR + 0x0000)
#define DMA2_BASEADDR				(AHB1_BASEADDR + 0x0400)
#define RCC_BASEADDR				(AHB1_BASEADDR + 0x1000)
#define FLASHINTERFACE_BASEADDR		(AHB1_BASEADDR + 0x2000)
#define CRC_BASEADDR				(AHB1_BASEADDR + 0x3000)
#define TSC_BASEADDR				(AHB1_BASEADDR + 0x4000)

/*
 * APB PERIPHERALS
 */
#define EXTI_BASEADDR				(APB_BASEADDR + 0x4000)
#define ADC_BASEADDR				(APB_BASEADDR + 0x2400)
#define DBGMCU_BASEADDR				(APB_BASEADDR + 0x5800)
#define RTC_BASEADDR				(APB_BASEADDR + 0x2800)

#define IC21_BASEADDR				(APB_BASEADDR + 0x5400)
#define IC22_BASEADDR				(APB_BASEADDR + 0x5800)

#define USART1_BASEADDR				(APB_BASEADDR + 0x3800)
#define USART2_BASEADDR				(APB_BASEADDR + 0x4400)
#define USART3_BASEADDR				(APB_BASEADDR + 0x4800)
#define USART4_BASEADDR				(APB_BASEADDR + 0x4C00)
#define USART5_BASEADDR				(APB_BASEADDR + 0x5000)
#define USART6_BASEADDR				(APB_BASEADDR + 0x1400)
#define USART7_BASEADDR				(APB_BASEADDR + 0x1800)
#define USART8_BASEADDR				(APB_BASEADDR + 0x1C00)

#define SPI1_I2S1_BASEADDR			(APB_BASEADDR + 0x3000)
#define SPI2_BASEADDR				(APB_BASEADDR + 0x3800)

#define TIM2_BASEADDR				(APB_BASEADDR + 0x0000)
#define TIM3_BASEADDR				(APB_BASEADDR + 0x0400)
#define TIM6_BASEADDR				(APB_BASEADDR + 0x1000)
#define TIM7_BASEADDR				(APB_BASEADDR + 0x1400)
#define TIM14_BASEADDR				(APB_BASEADDR + 0x2000)
#define TIM15_BASEADDR				(APB_BASEADDR + 0x4000)
#define TIM16_BASEADDR				(APB_BASEADDR + 0x4400)
#define TIM17_BASEADDR				(APB_BASEADDR + 0x4800)

#define USB_BASEADDR				(APB_BASEADDR + 0x5C00)
#define USB_CAN_BASEADDR			(APB_BASEADDR + 0x6000)
#define CAN_BASEADDR				(APB_BASEADDR + 0x6400)

#define WWDG_BASEADDR				(APB_BASEADDR + 0x2C00)
#define IWDC_BASEADDR				(APB_BASEADDR + 0x3000)


/*
 *  PERIPHERAL DEFINATION
 */

#define GPIOA						((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB						((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC						((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD						((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE						((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF						((GPIO_RegDef_t*)GPIOF_BASEADDR)

#define RCC							((RCC_RegDef_t*)RCC_BASEADDR)

/*
 * Register peripheral structure for GPIO
 */

typedef struct
{
	__vo uint32_t MODER; 		/*GPIO port mode register 							Offset = 0x00 */
	__vo uint32_t OTYPER;		/*GPIO port output type register 					Offset = 0x04 */
	__vo uint32_t OSPEEDR;		/*GPIO port output speed register 					Offset = 0x08 */
	__vo uint32_t PUPDR;		/*GPIO port pull-up/pull-down register 				Offset = 0x0C */
	__vo uint32_t IDR;			/*GPIO port input data register 					Offset = 0x10 */
	__vo uint32_t ODR;			/*GPIO port output data register 					Offset = 0x14 */
	__vo uint32_t BSRR;			/*GPIO port bit set/reset register 					Offset = 0x18 */
	__vo uint32_t LCKR;			/*GPIO port configuration lock register				Offset = 0x1C */
	__vo uint32_t AFR[2];		/*GPIO port alternate function array 0:Low 1:High   Offset = 0x20 */
	__vo uint32_t BRR;			/*GPIO port bit reset register 						Offset = 0x28 */

}GPIO_RegDef_t;

/*
 * Register peripheral structure for RCC
 */

typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t APB2RSTR;
	__vo uint32_t APB1RSTR;
	__vo uint32_t AHBENR;
	__vo uint32_t APB2ENR;
	__vo uint32_t APB1ENR;
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t AHBRSTR;
	__vo uint32_t CFGR2;
	__vo uint32_t CFGR3;
	__vo uint32_t CR2;


}RCC_RegDef_t;

/*
 *  Clock Management Macros for GPIOx
 */

#define GPIOA_PCLK_EN() 		(RCC -> AHBENR |= (1 << 17))
#define GPIOB_PCLK_EN() 		(RCC -> AHBENR |= (1 << 18))
#define GPIOC_PCLK_EN() 		(RCC -> AHBENR |= (1 << 19))
#define GPIOD_PCLK_EN() 		(RCC -> AHBENR |= (1 << 20))
#define GPIOE_PCLK_EN() 		(RCC -> AHBENR |= (1 << 21))
#define GPIOF_PCLK_EN() 		(RCC -> AHBENR |= (1 << 22))

#define GPIOA_PCLK_DI() 		(RCC -> AHBENR &= ~(1 << 17))
#define GPIOB_PCLK_DI() 		(RCC -> AHBENR &= ~(1 << 18))
#define GPIOC_PCLK_DI() 		(RCC -> AHBENR &= ~(1 << 19))
#define GPIOD_PCLK_DI() 		(RCC -> AHBENR &= ~(1 << 20))
#define GPIOE_PCLK_DI() 		(RCC -> AHBENR &= ~(1 << 21))
#define GPIOF_PCLK_DI() 		(RCC -> AHBENR &= ~(1 << 22))

/*
 *  Clock Management Macros for I2Cx
 */
#define I2C1_PCLK_EN() 		(RCC -> APB1ENR |= (1 << 21))
#define I2C1_PCLK_DI() 		(RCC -> APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_EN() 		(RCC -> APB1ENR |= (1 << 22))
#define I2C2_PCLK_DI() 		(RCC -> APB1ENR &= ~(1 << 22))

/*
 *  Clock Management Macros for SPIx
 */

#define SPI1_PCLK_EN() 		(RCC -> APB2ENR |= (1 << 12))
#define SPI1_PCLK_DI() 		(RCC -> APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_EN() 		(RCC -> APB1ENR |= (1 << 14))
#define SPI2_PCLK_DI() 		(RCC -> APB1ENR &= ~(1 << 14))


/*
 *  Clock Management Macros for SYSCFG
 */
#define SYSCFG_PCLK_EN() 	(RCC->APB2ENR |= (1 << 0))
#define SYSCFG_PCLK_DI() 	(RCC->APB2ENR &= ~(1 << 0))

/*
 * Peripheral Register Reset Macros
 */
#define GPIOA_REG_RESET() 		do{(RCC ->AHBRSTR |= (1 << 17));(RCC ->AHBRSTR &= ~(1 << 17));} while(0)
#define GPIOB_REG_RESET() 		do{(RCC ->AHBRSTR |= (1 << 18));(RCC ->AHBRSTR &= ~(1 << 18));} while(0)
#define GPIOC_REG_RESET() 		do{(RCC ->AHBRSTR |= (1 << 19));(RCC ->AHBRSTR &= ~(1 << 19));} while(0)
#define GPIOD_REG_RESET() 		do{(RCC ->AHBRSTR |= (1 << 20));(RCC ->AHBRSTR &= ~(1 << 20));} while(0)
#define GPIOE_REG_RESET() 		do{(RCC ->AHBRSTR |= (1 << 21));(RCC ->AHBRSTR &= ~(1 << 21));} while(0)
#define GPIOF_REG_RESET() 		do{(RCC ->AHBRSTR |= (1 << 22));(RCC ->AHBRSTR &= ~(1 << 22));} while(0)

/*
 * General macros
 */


#define ENABLE 				1
#define DISABLE 			0
#define RESET 				DISABLE
#define SET					ENABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET


// Drivers
#include "stm32f05xx_gpio_driver.h"


#endif /* STM32F05XX_H_ */

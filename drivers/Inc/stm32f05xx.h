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

/**************** MCU Specific Macros for M0 Cortex **********
 *
 */

#define NVIC_ISER  					((__vo uint32_t*)0xE000E100)
#define NVIC_ICER  					((__vo uint32_t*)0xE000E180)
#define NVIC_ISPR  					((__vo uint32_t*)0xE000E200)
#define NVIC_ICPR  					((__vo uint32_t*)0xE000E280)


#define NVIC_PR_BASEADDR 			((__vo uint32_t*)0xE000E400)

/*
 * NVIC PR Register number of bits implemented
 */
#define NO_PR_BITS_IMPLEMENTED		2




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
#define SYSCFG_BASEADDR				(APB_BASEADDR)
#define EXTI_BASEADDR				(APB_BASEADDR + 0x0400)
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

#define EXTI						((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG						((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1						((SPI_RegDef_t*)SPI1_I2S1_BASEADDR)
#define SPI2						((SPI_RegDef_t*)SPI2_BASEADDR)

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
 * Register peripheral structure for EXTI
 */

typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;

}EXTI_RegDef_t;



/*
 * Register peripheral structure for SYSCFG
 */

typedef struct
{
	__vo uint32_t CFGR1;
	__vo uint32_t rsv;
	__vo uint32_t EXTICR[4];
	__vo uint32_t CFGR2;

}SYSCFG_RegDef_t;

/*
 *  Register peripheral structure for SPI
 */
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2DCFGR;
	__vo uint32_t I2SPR;
}SPI_RegDef_t;

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

#define GPIO_BASEADDR_TO_PORT_CODE(baseAddr) 	((baseAddr == GPIOA) ? 0:\
												(baseAddr == GPIOB) ? 1:\
												(baseAddr == GPIOC) ? 2:\
												(baseAddr == GPIOD) ? 3:\
												(baseAddr == GPIOE) ? 4:\
												(baseAddr == GPIOF) ? 5:0)


#define SPI1_REG_RESET() 		do{(RCC->APB2RSTR |= (1<<12));(RCC->APB2RSTR &= ~(1<<12));} while(0)
#define SPI2_REG_RESET() 		do{(RCC->APB1RSTR |= (1<<14));(RCC->APB1RSTR &= ~(1<<14));} while(0)

/*
 * IRQ Number Macros
 */
#define IRQ_NO_EXTI0_1			5
#define IRQ_NO_EXTI2_3			6
#define IRQ_NO_EXTI4_15			7


/*
 * General macros
 */


#define ENABLE 				1
#define DISABLE 			0
#define RESET 				DISABLE
#define SET					ENABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define FLAG_SET			SET
#define FLAG_RESET			RESET


/***************
 *  SPI Peripherals Bit Positions
 ****************/
#define SPIx_CR1_CPHA		0
#define SPIx_CR1_CPOL		1
#define SPIx_CR1_MSTR		2
#define SPIx_CR1_BR			3
#define SPIx_CR1_SPE		6
#define SPIx_CR1_LSBFIRST	7
#define SPIx_CR1_SSI		8
#define SPIx_CR1_SSM		9
#define SPIx_CR1_RXONLY		10
#define SPIx_CR1_CRCL		11
#define SPIx_CR1_CRCNEXT	12
#define SPIx_CR1_CRCEN		13
#define SPIx_CR1_BIDIOE		14
#define SPIx_CR1_BIDIMODE	15

#define SPIx_CR2_RXDMAEN	0
#define SPIx_CR2_TXDMAEN	1
#define SPIx_CR2_SSOE		2
#define SPIx_CR2_NSSP		3
#define SPIx_CR2_FRF		4
#define SPIx_CR2_ERRIE		5
#define SPIx_CR2_RXNEIE		6
#define SPIx_CR2_TXEIE		7
#define SPIx_CR2_DS			8
#define SPIx_CR2_FRXTH		12
#define SPIx_CR2_LDMA_RX	13
#define SPIx_CR2_LDMA_TX	14

#define SPIx_SR_RXNE		0
#define SPIx_SR_TXE			1
#define SPIx_SR_CHSIDE		2
#define SPIx_SR_UDR			3
#define SPIx_SR_CRCERR		4
#define SPIx_SR_MODF		5
#define SPIx_SR_OVR			6
#define SPIx_SR_BSY			7
#define SPIx_SR_FRE			8
#define SPIx_SR_FRLVL		9
#define SPIx_SR_FTLVL		11
#define SPIx_CR2_LDMA_TX	14




// Drivers
#include "stm32f05xx_gpio_driver.h"
#include "stm32f05xx_spi_driver.h"


#endif /* STM32F05XX_H_ */

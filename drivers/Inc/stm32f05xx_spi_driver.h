/*
 * stm32f05xx_spi_driver.h
 *
 *  Created on: Jun 24, 2022
 *      Author: batuhansesli
 */

#ifndef INC_STM32F05XX_SPI_DRIVER_H_
#define INC_STM32F05XX_SPI_DRIVER_H_

#include "stm32f05xx.h"

/*
 * This is configuration structure for GPIO
 */

typedef struct
{
	uint8_t SPI_DeviceMode;					/* possible values: @ */
	uint8_t SPI_BusConfig;					/* possible values: @ */
	uint8_t SPI_DFF;					/* possible values: @ */
	uint8_t SPI_CPHA;			/* possible values: @ */
	uint8_t SPI_CPOL;					/* possible values: @ */
	uint8_t SPI_SSM;				/* possible values: @ */
	uint8_t SPI_Speed;
}SPI_Config_t;


/*!
 *  This is handle structure for GPIO
 */
typedef struct
{
	SPI_RegDef_t *pSPIx; 					/* Holds base address of SPI*/
	SPI_Config_t SPI_Config;				/* Holds configuration structure*/
}SPI_Handle_t;



/***********************************************************************************************
 *  								FUNCTION PROTOTYPES FOR SPI DRIVER
 ***********************************************************************************************/

/* GPIO init and DeInit */
void SPI_Init(SPI_Handle_t *pSPI_Handle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/* GPIO Peripheral Clock Setup*/
void SPI_PeripClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/* Data Recieve & Send */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

/* Interrupt Config and Handling */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandle(SPI_Handle_t *pHandle);

#endif /* INC_STM32F05XX_SPI_DRIVER_H_ */

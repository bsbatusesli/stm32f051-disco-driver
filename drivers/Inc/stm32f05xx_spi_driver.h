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
 * This is configuration structure for SPI
 */

typedef struct
{
	uint8_t SPI_DeviceMode;					/* possible values: @SPI_DeviceMode */
	uint8_t SPI_BusConfig;					/* possible values: @SPI_BusConfig */
	uint8_t SPI_DFF;						/* possible values: @SPI_DFF */
	uint8_t SPI_CPHA;						/* possible values: @SPI_CPHA */
	uint8_t SPI_CPOL;						/* possible values: @SPI_CPOL */
	uint8_t SPI_SSM;						/* possible values: @SPI_SSM */
	uint8_t SPI_SCLK;						/* possible values: @SPI_SCLK */
}SPI_Config_t;


/*!
 *  This is handle structure for SPI
 */
typedef struct
{
	SPI_RegDef_t *pSPIx; 					/* Holds base address of SPI*/
	SPI_Config_t SPI_Config;				/* Holds configuration structure*/
}SPI_Handle_t;

/***********************************************************************************************
 *  								Possible Register Macros
 ***********************************************************************************************/

/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER 	1
#define SPI_DEVICE_MODE_SLAVE 	0

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD 				0
#define SPI_BUS_CONFIG_HD 				1
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY 	2

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BIT 					7
#define SPI_DFF_16BIT					15

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_FIRST					0
#define SPI_CPHA_SECOND					1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_LOW					0
#define SPI_CPOL_HIGH					1

/*
 * @SPI_SSM
 */
#define SPI_SSM_DI						0
#define SPI_SSM_EN						1

/*
 * @SPI_SCLK
 */
#define SPI_SCLK_DIV2					0
#define SPI_SCLK_DIV4					1
#define SPI_SCLK_DIV8					2
#define SPI_SCLK_DIV16					3
#define SPI_SCLK_DIV32					4
#define SPI_SCLK_DIV64					5
#define SPI_SCLK_DIV128					6
#define SPI_SCLK_DIV256					7


/***********************************************************************************************
 *  								FUNCTION PROTOTYPES FOR SPI DRIVER
 ***********************************************************************************************/

/* SPI init and DeInit */
void SPI_Init(SPI_Handle_t *pSPI_Handle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/* SPI Peripheral Clock Setup*/
void SPI_PeripClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/* Data Recieve & Send */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

/* Interrupt Config and Handling */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandle(SPI_Handle_t *pHandle);

/* Other Peripheral APIs */
void SPI_Perip_Control(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSI_Control(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOE_Control(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName);

#endif /* INC_STM32F05XX_SPI_DRIVER_H_ */

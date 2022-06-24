/*
 * stm32f05xx_spi_driver.h
 *
 *  Created on: Jun 24, 2022
 *      Author: batuhansesli
 */

#ifndef INC_STM32F05XX_SPI_DRIVER_H_
#define INC_STM32F05XX_SPI_DRIVER_H_

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

#endif /* INC_STM32F05XX_SPI_DRIVER_H_ */

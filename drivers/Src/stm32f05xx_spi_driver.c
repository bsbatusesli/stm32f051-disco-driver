/*
 * stm32f05xx_spi_driver.c
 *
 *  Created on: Jun 24, 2022
 *      Author: batuhansesli
 */

#include "stm32f05xx_spi_driver.h"

/* GPIO Peripheral Clock Setup*/
void SPI_PeripClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
			SPI1_PCLK_EN();

		else if(pSPIx == SPI2)
			SPI2_PCLK_EN();
	}else
	{
		if(pSPIx == SPI1)
			SPI1_PCLK_DI();

		else if(pSPIx == SPI2)
			SPI2_PCLK_DI();
	}
}

/* GPIO init and DeInit */
void SPI_Init(SPI_Handle_t *pSPI_Handle)
{

}
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
		SPI1_REG_RESET();

	else if(pSPIx == SPI2)
		SPI2_REG_RESET();
}




/* Data Recieve & Send */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{

}
void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{

}



/* Interrupt Config and Handling */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{

}
void SPI_IRQHandle(SPI_Handle_t *pSPI_Handle)
{

}

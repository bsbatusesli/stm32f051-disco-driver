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
	SPI_PeripClockControl(pSPI_Handle->pSPIx, ENABLE);

	uint16_t temp_reg = 0;

	// setting device mod
	temp_reg |= pSPI_Handle->SPI_Config.SPI_DeviceMode << 2;

	// setting bus config
	if(pSPI_Handle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// clear bidi
		temp_reg &= ~(1 << 15);

	}else if(pSPI_Handle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//set bidi
		temp_reg |= (1 << 15);

	}else if(pSPI_Handle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//set bidi
		temp_reg |= (1 << 15);
		//RXONLY bit set
		temp_reg |= (1 << 10);
	}

	// setting the data frame size
	pSPI_Handle->pSPIx->CR2 = (pSPI_Handle->pSPIx->CR2 & 0xF0FF) | (pSPI_Handle->SPI_Config.SPI_DFF << 8);

	// setting idle state config (cpol)
	temp_reg = (temp_reg & 0xFFFD) | (pSPI_Handle->SPI_Config.SPI_CPOL << 1);
	// setting the data capture time (cpha)
	temp_reg = (temp_reg & 0xFFFE) | (pSPI_Handle->SPI_Config.SPI_CPHA);
	// setting Software select mode
	temp_reg = (temp_reg & 0xFEFF) | (pSPI_Handle->SPI_Config.SPI_SSM << 9);
	// setting clock speed
	temp_reg = (temp_reg & 0xFFC7) | (pSPI_Handle->SPI_Config.SPI_SCLK << 3);

	pSPI_Handle->pSPIx->CR1 = temp_reg;

}
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
		SPI1_REG_RESET();

	else if(pSPIx == SPI2)
		SPI2_REG_RESET();
}

void SPI_Perip_Control(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPIx_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPIx_CR1_SPE);
	}
}

void SPI_SSI_Control(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPIx_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPIx_CR1_SSI);
	}
}

void SPI_SSOE_Control(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPIx_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &= ~(1 << SPIx_CR2_SSOE);
	}
}




/* Data Recieve & Send */
/* ---------------------------------------------------------------------------
 * @name: 			SPI_SendData
 * @desc:			SPI sends Data
 *
 * @in[pSPIx]:		pointer which holds base address of SPI
 * @in[pTxBuffer]:	pointer which points the data
 * @in[len]:		total byte of the data
 *
 * @returns: 		none
 *
 * !Note!: 			this is blocking call.
 * --------------------------------------------------------------------------*/
//Buggy
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
	while(len > 0)
	{
		// wait until TX Buffer empty
		while(! (pSPIx->SR & (1 << SPIx_SR_TXE)) );

		//check data size
		if( ((pSPIx->CR2) & (0xF << SPIx_CR2_DS)) == (0xF << SPIx_CR2_DS)) // Data size = 16 bit
		{
			//load data to DF
			pSPIx->DR = *(uint16_t*)pTxBuffer;
			//decrement len
			len--;
			len--;
			//increment pTxBuffer
			(uint16_t*)pTxBuffer++;

		}else if (((pSPIx->CR2) & (0x7 << SPIx_CR2_DS)) == (0x7 << SPIx_CR2_DS)) // data size = 8 bit
		{
			//load data to DF
			pSPIx->DR = *pTxBuffer;
			//decrement len
			len--;
			//increment pTxBuffer
			pTxBuffer++;

		}

	}

}
void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{

}



/* Interrupt Config and Handling */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if( EnorDi == ENABLE)
	{
		*NVIC_ISER |= (1 << IRQNumber);
	}else
	{
		*NVIC_ICER |= (1 << IRQNumber);
	}
}
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t ipr_index = IRQNumber / 4;
	uint8_t ipr_offset = IRQNumber % 4;

	uint8_t shift_amount = (8 * ipr_offset) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_PR_BASEADDR + ipr_index) |= (IRQPriority << shift_amount); // Just adding ipr_index because nvic pointer is 32 bit. Adding 1 is enough for step up 32 bit

}
void SPI_IRQHandle(SPI_Handle_t *pSPI_Handle)
{

}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName)
{
	if((pSPIx->SR >> FlagName) % 2 == 1)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint8_t status = pSPIHandle->TxState;

	if(status != SPI_BUSY_IN_TX)
	{
		// Save the TX buffer and len info
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = len;
		//Mark the SPI status Busy so no other code can takeover
		pSPIHandle->TxState=SPI_BUSY_IN_TX;
		//Enable TXIEI control bit to get TXE flag
		pSPIHandle->pSPIx->CR2 |= (1 << SPIx_CR2_TXEIE);
		//Data transmisssion handled by ISR code
	}
	return status;
}
uint8_t SPI_RecieveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len)
{
	uint8_t status = pSPIHandle->RxState;

	if(status != SPI_BUSY_IN_RX)
	{
		// Save the TX buffer and len info
		pSPIHandle->pRxBuffer = pTxBuffer;
		pSPIHandle->RxLen = len;
		//Mark the SPI status Busy so no other code can takeover
		pSPIHandle->RxState=SPI_BUSY_IN_TX;
		//Enable TXIEI control bit to get TXE flag
		pSPIHandle->pSPIx->CR2 |= (1 << SPIx_CR2_RXNEIE);
		//Data transmisssion handled by ISR code
	}
	return status;
}

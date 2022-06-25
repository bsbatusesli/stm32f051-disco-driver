/*
 * 004_spi_tx.c
 *
 *  Created on: Jun 25, 2022
 *      Author: batuhansesli
 */

#include "stm32f05xx.h"
#include <string.h>

void SPI1_GPIO_Inits(void)
{

	/*  SPI1 Supported Pins
	 *  NSS 	PA4
	 *  SCK 	PA5
	 *  MISO 	PA6
	 *  MOSI 	PA7
	 *  Alt functionality = 0;
	 */

	GPIO_Handle_t SPI_Pins;
	memset(&SPI_Pins, 0, sizeof(SPI_Pins));

	SPI_Pins.pGPIOx = GPIOA;
	SPI_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPI_Pins.GPIO_PinConfig.GPIO_AltFuncMode = 0;
	SPI_Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OT_PP;
	SPI_Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPI_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	// NSS
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = 4;
	GPIO_Init(&SPI_Pins);

	// SCK
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = 5;
	GPIO_Init(&SPI_Pins);

	//MISO
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = 6;
	GPIO_Init(&SPI_Pins);

	//MOSI
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = 7;
	GPIO_Init(&SPI_Pins);


}

void SPI1_Init(void)
{
	SPI_Handle_t spi1_Handle;

	spi1_Handle.pSPIx = SPI1;
	spi1_Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	spi1_Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	spi1_Handle.SPI_Config.SPI_DFF = SPI_DFF_8BIT;
	spi1_Handle.SPI_Config.SPI_SCLK = SPI_SCLK_DIV2;
	spi1_Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	spi1_Handle.SPI_Config.SPI_CPHA = SPI_CPHA_FIRST;
	spi1_Handle.SPI_Config.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&spi1_Handle);
}


int main(void)
{
	char data[] = "Hello world";

	SPI1_GPIO_Inits();

	SPI1_Init();

	// making NSS internally high to avoid MODF Error
	SPI_SSI_Control(SPI1, ENABLE);

	 // enable SPI peripheral
	SPI_Perip_Control(SPI1, ENABLE);

	SPI_SendData(SPI1, (uint8_t*)data, strlen(data));

	SPI_Perip_Control(SPI1, DISABLE);

	while(1);

	return 0;


}

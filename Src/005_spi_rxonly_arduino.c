/*
 * 005_spi_rxonly_arduino.c
 *
 *  Created on: Jun 27, 2022
 *      Author: batuhansesli
 */


#include "stm32f05xx.h"
#include <string.h>

#define HIGH 				1
#define LOW					0
#define BTN_PRESSED 		LOW

void delay() {
	for(uint32_t i = 0; i < 500000; i++);
}

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
	//SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = 6;
	//GPIO_Init(&SPI_Pins);

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
	spi1_Handle.SPI_Config.SPI_SCLK = SPI_SCLK_DIV8;
	spi1_Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	spi1_Handle.SPI_Config.SPI_CPHA = SPI_CPHA_FIRST;
	spi1_Handle.SPI_Config.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&spi1_Handle);
}

void GPIO_ButtonInit(void)
{
		GPIO_Handle_t GpioBtn;
		memset(&GpioBtn, 0, sizeof(GpioBtn));

		GpioBtn.pGPIOx = GPIOB;
		GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_1;
		GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
		GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
		GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;


		GPIO_Init(&GpioBtn);
}


int main(void)
{

	GPIO_ButtonInit();

	SPI1_GPIO_Inits();

	SPI1_Init();

	SPI_SSOE_Control(SPI1, ENABLE);

	char data[] = "Hello world";

	while(1)
	{
		//wait until button is pressed
		while(! (GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_1) == BTN_PRESSED));
		delay();

		 // enable SPI peripheral
		SPI_Perip_Control(SPI1, ENABLE);

		// first send the lenght of data
		uint8_t dataLen = strlen(data);
		SPI_SendData(SPI1, &dataLen, 1);

		// send data
		SPI_SendData(SPI1, (uint8_t*)data, dataLen);

		// wait until spi is not busy
		while(SPI_GetFlagStatus(SPI1, SPIx_SR_BSY));

		//disable spi
		SPI_Perip_Control(SPI1, DISABLE);


	}

	return 0;


}

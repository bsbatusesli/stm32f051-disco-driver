/*
 * 002_led_button.c
 *
 *  Created on: Jun 8, 2022
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

int main(void)
{
	GPIO_Handle_t GpioLed;
	GPIO_Handle_t GpioBtn;
	memset(&GpioLed, 0, sizeof(GpioLed));
	memset(&GpioBtn, 0, sizeof(GpioBtn));

	GpioLed.pGPIOx = GPIOC;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_9;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OT_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	GPIO_PeripClockControl(GpioLed.pGPIOx, ENABLE);

	GPIO_Init(&GpioLed);

	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	GPIO_PeripClockControl(GpioBtn.pGPIOx, ENABLE);

	GPIO_Init(&GpioBtn);


	while(1)
	{
		if(GPIO_ReadFromInputPin(GpioBtn.pGPIOx, GpioBtn.GPIO_PinConfig.GPIO_PinNumber) == BTN_PRESSED)
		{
			GPIO_ToggleOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber);
			delay();
		}
	}
	return 0;
}


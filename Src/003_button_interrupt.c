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

	GPIO_PeripClockControl(GPIOC, ENABLE);

	GPIO_Init(&GpioLed);

	GpioBtn.pGPIOx = GPIOB;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_1;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	GPIO_PeripClockControl(GPIOB, ENABLE);

	GPIO_Init(&GpioBtn);

	//IRQ Configuration
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0_1, 15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0_1, ENABLE);



	while(1);

	return 0;
}

void EXTI0_1_IRQHandler(void)
{
	GPIO_IRQHandle(GPIO_PIN_1); // for clearing pending
	GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_9);
	delay();

}

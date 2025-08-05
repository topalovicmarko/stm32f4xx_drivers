/*
 * 004button_interrupt.c
 *
 *  Created on: Aug 5, 2025
 *      Author: marko
 */

#include<string.h>
#include "stm32f401xx.h"

#define LOW 0
#define BTN_PRESSED LOW

void delay(void)
{
	// this will introduce ~200ms delay when system clock (sysclk) is 16MHZ
	for(uint32_t i = 0 ; i < 500000/2 ; i++);
}

int main(void)
{

	GPIO_Handle_t GpioLed, GpioBtn;
	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GpioBtn,0,sizeof(GpioBtn));

	//This is Led gpio configuration
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GpioLed);

	//This is btn gpio configuration
	GpioBtn.pGPIOx = GPIOB;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOB, ENABLE);

	GPIO_Init(&GpioBtn);

	//IRQ configurations

	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);

	while(1);

	return 0;
}

void EXTI15_10_IRQHandler(void)
	{
		delay();	//200ms debouncing
		GPIO_IRQHandling(GPIO_PIN_NO_12);		//clear the pending event from exti line
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
	}

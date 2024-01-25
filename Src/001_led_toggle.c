/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Auto-generated by STM32CubeIDE
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */


#include<stm32f4xx.h>

void delay(void)
{
	for (uint32_t i = 0;  i < 500000; i++);
}

int main(void)
{
	GPIO_Handle_t gLED;
	gLED.pGPIOx = GPIOD;
	gLED.GPIO_PinCfg.GPIO_PinNumber = GPIO_PIN_12;
	gLED.GPIO_PinCfg.GPIO_PinMode = GPIO_MODE_OUT;
	gLED.GPIO_PinCfg.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gLED.GPIO_PinCfg.GPIO_PinOutputType = GPIO_OP_OD;
	gLED.GPIO_PinCfg.GPIO_PinPuPdControl =GPIO_NO_PUPD;

	GPIO_PeriClockCtrl(GPIOD, ENABLE);
	GPIO_Init(&gLED);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
		delay();
	}

}

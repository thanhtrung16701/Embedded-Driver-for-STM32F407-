/*
 * 003button_ext.c
 *
 *  Created on: Jan 7, 2024
 *      Author: ThanhPC
 */
#include <stm32f4xx.h>

void delay(void)
{
	for (uint32_t i = 0;  i < 500000/2; i++);
}


int main(int argc, char **argv) {
	GPIO_Handle_t gLED, gBTN;
		gLED.pGPIOx = GPIOA;
		gLED.GPIO_PinCfg.GPIO_PinNumber = GPIO_PIN_8;
		gLED.GPIO_PinCfg.GPIO_PinMode = GPIO_MODE_OUT;
		gLED.GPIO_PinCfg.GPIO_PinSpeed = GPIO_SPEED_FAST;
		gLED.GPIO_PinCfg.GPIO_PinOutputType = GPIO_OP_PP;
		gLED.GPIO_PinCfg.GPIO_PinPuPdControl = GPIO_NO_PUPD;

		GPIO_PeriClockCtrl(GPIOA, ENABLE);
		GPIO_Init(&gLED);


		gBTN.pGPIOx = GPIOB;
		gBTN.GPIO_PinCfg.GPIO_PinNumber = GPIO_PIN_12;
		gBTN.GPIO_PinCfg.GPIO_PinMode = GPIO_MODE_IN;
		gBTN.GPIO_PinCfg.GPIO_PinSpeed = GPIO_SPEED_FAST;
		gBTN.GPIO_PinCfg.GPIO_PinPuPdControl = GPIO_PU;

		GPIO_PeriClockCtrl(GPIOB, ENABLE);
		GPIO_Init(&gBTN);
		while(1)
		{
			if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_12) == 0){
		delay();
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_8);
			}
		}
		return 0;
}




/*
 * 005_button_interrupt.c
 *
 *  Created on: Jan 10, 2024
 *      Author: ThanhPC
 */

#include <stm32f4xx.h>
#include <string.h>

#define HIGH 1
#define LOW 0
#define BTN_PRESSED LOW

int a =0;
void delay(void)
{
	for (uint32_t i = 0;  i < 500000/2; i++); // ~ 200ms when system clock is 16 Mhz
}




int main(int argc, char **argv) {
	GPIO_Handle_t gLED, gBTN;

	memset(&gLED,0,sizeof(gLED));
	memset(&gBTN,0,sizeof(gBTN));

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
	gBTN.GPIO_PinCfg.GPIO_PinMode = GPIO_MODE_IT_FT;
	gBTN.GPIO_PinCfg.GPIO_PinSpeed = GPIO_SPEED_FAST;
	// INPUT has no OUTPUT type
	gBTN.GPIO_PinCfg.GPIO_PinPuPdControl = GPIO_PU;

	GPIO_PeriClockCtrl(GPIOB, ENABLE);
	GPIO_Init(&gBTN);

	GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	//IRQ config
	GPIO_IRQPriorityConfig(IRQ_NUM_EXTI15_10, NVIC_IRQ_PRI10);
	GPIO_IRQInterruptConfig(IRQ_NUM_EXTI15_10, ENABLE);


	while(1);

	// Implement ISR by searching "EXTI9_5" in startup file

}

void EXTI15_10_IRQHandler(void)
{
	delay();
	GPIO_IRQHanding(GPIO_PIN_12);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_8);
	a++;
}



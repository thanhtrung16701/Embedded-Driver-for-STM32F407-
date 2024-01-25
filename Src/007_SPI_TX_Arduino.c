/*
 * 007_SPI_TX_Arduino.c
 *
 *  Created on: Jan 15, 2024
 *      Author: ThanhPC
 */

/*SPI
 * PB12 -> NSS
 * PB13 -> SCK
 * PB14 -> MISO
 * PB15 -> MOSI
 * ALT Mode : AF5
 */
#include <stm32f4xx.h>
#include <string.h>

void delay(void)
{
	for (uint32_t i = 0;  i < 500000/2; i++); // ~ 200ms when system clock is 16 Mhz
}


void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinCfg.GPIO_PinMode = GPIO_MODE_ALFN;
	SPIPins.GPIO_PinCfg.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinCfg.GPIO_PinSpeed = GPIO_SPEED_FAST;
	SPIPins.GPIO_PinCfg.GPIO_PinOutputType = GPIO_OP_PP;
	SPIPins.GPIO_PinCfg.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	// PB15 -> MOSI
	SPIPins.GPIO_PinCfg.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPIPins);

	// PB13 -> SCK
	SPIPins.GPIO_PinCfg.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPIPins);
	//PB12 -> NSS
	SPIPins.GPIO_PinCfg.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;
	SPI2handle.SPIx = SPI2;
	SPI2handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPI_Config.SPI_BusConfig = SPI_BUS_CFG_FD;
	SPI2handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; // sclk 2Mhz
	SPI2handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPI_Config.SPI_SSM = SPI_SSM_DI; // HW enable for NSS
	SPI_Init(&SPI2handle);

}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t gBTN;
	gBTN.pGPIOx = GPIOA;
		gBTN.GPIO_PinCfg.GPIO_PinNumber = GPIO_PIN_0;
		gBTN.GPIO_PinCfg.GPIO_PinMode = GPIO_MODE_IN;
		gBTN.GPIO_PinCfg.GPIO_PinSpeed = GPIO_SPEED_FAST;
		gBTN.GPIO_PinCfg.GPIO_PinPuPdControl = GPIO_NO_PUPD;

		GPIO_Init(&gBTN);
}

int main(int argc, char **argv) {
	char data[]= "Hello world";
//	char data[] = "An Arduino Uno board is best  suited for beginners who have just started using microcontrollers, on the other hand, Arduino Mega board is for enthusiasts who require a lot of I/O pins for their projects";

	GPIO_ButtonInit();

	// init GPIO pin for SPI
	SPI2_GPIOInits();
	// init SPI peri parameters
	SPI2_Inits();



	/*
	 * enable NSS output
	 * NSS's automatically managed by hardware
	 *  SPE = 1 -> NSS is pulled to low
	 *  SPE = 0 -> NSS -> High
	 */
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		// Wait till button is pressed
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0)); // BUTTON = 1
		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		// enable SPI2 peripheral
			SPI_PeriControl(SPI2, ENABLE);

			// send length info
			uint8_t DataLen = strlen(data);
			SPI_SendData(SPI2, &DataLen, 1);

			// send data
			SPI_SendData(SPI2, (uint8_t*)data, strlen(data));

			//  after data is sent, confirm SPI isn't busy
			/*
			 * IF(FLAG = 0) -> Not busy -> come out of loop
			 * IF(FLAG = 1)	-> busy -> wait till send full data -> flag = 0
			 */
			while( SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG)); // flag = 0

			// , disable SPI peripheral control
			SPI_PeriControl(SPI2, DISABLE);

	}


}


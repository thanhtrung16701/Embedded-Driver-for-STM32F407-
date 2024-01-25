/*
 * 005_SPI_TX_testing.c
 *
 *  Created on: Jan 13, 2024
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


}

void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;
	SPI2handle.SPIx = SPI2;
	SPI2handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPI_Config.SPI_BusConfig = SPI_BUS_CFG_FD;
	SPI2handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; // sclk 8Mhz
	SPI2handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPI_Config.SPI_SSM = SPI_SSM_EN; // SW enable for NSS

	SPI_Init(&SPI2handle);

}

int main(int argc, char **argv) {
	char data[]= "Hello world";
	// init GPIO pin for SPI
	SPI2_GPIOInits();
	// init SPI peri parameters
	SPI2_Inits();

	//make NSS signal internally high and avoid MODF error
	SPI_SSIConfig(SPI2, ENABLE);
	// enable SPI2 peripheral
	SPI_PeriControl(SPI2, ENABLE);

	SPI_SendData(SPI2, (uint8_t*)data, strlen(data));
	//  after data is sent, confirm SPI isn't busy
	/*
	 * IF(FLAG = 0) -> Not busy -> come out of loop
	 * IF(FLAG = 1)	-> busy -> wait till send full data -> flag = 0
	 */
	while( SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG)); // flag = 0

	// , disable SPI peripheral control
	SPI_PeriControl(SPI2, DISABLE);

	while(1);
}


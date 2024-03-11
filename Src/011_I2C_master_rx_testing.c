/*
 * 011_I2C_master_receive.c
 *
 *  Created on: Feb 25, 2024
 *      Author: ThanhPC
 */

#include <stdio.h>
#include <string.h>
#include <stm32f4xx.h>

#define	SLAVE_ADDR 0x68
#define	MY_ADDR 0x61

I2C_Handle_t I2C1Handle;

// data
uint8_t rx_buffer[32] ;
/*SPI
 * PB6 -> SCL
 * PB7 -> SDA
 */
void delay(void)
{
	for (uint32_t i = 0; i < 500000/2; i++);
}

void I2C_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;
	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinCfg.GPIO_PinMode = GPIO_MODE_ALFN;
	I2CPins.GPIO_PinCfg.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinCfg.GPIO_PinOutputType = GPIO_OP_OD;
	I2CPins.GPIO_PinCfg.GPIO_PinPuPdControl = GPIO_PU;
	I2CPins.GPIO_PinCfg.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCL
	I2CPins.GPIO_PinCfg.GPIO_PinNumber = GPIO_PIN_6;
	GPIO_Init(&I2CPins);

	// SDA
	I2CPins.GPIO_PinCfg.GPIO_PinNumber = GPIO_PIN_7;
	GPIO_Init(&I2CPins);
}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddr = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
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

int main(void) {
	I2C_GPIOInits();
	// I2C peripheral config
	I2C1_Inits();
	GPIO_ButtonInit();

	// EN I2C peripheral
	I2C_PeriControl(I2C1, ENABLE);

	I2C_MangageACKing(I2C1, I2C_ACK_ENABLE);// ACK =1 after PE =1

	uint8_t commandcode, len;

	while(1)
	{
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0) ); //button pressed
		delay();


		// read 1 byte len info from slave
		commandcode = 0x51;
		// write data, master send command to slave
		I2C_MasterSendData(&I2C1Handle, &commandcode, 1, SLAVE_ADDR, I2C_ENABLE_SR );
		// read data, master read response from slave
		I2C_MasterReceiveData(&I2C1Handle, &len, 1, SLAVE_ADDR, I2C_ENABLE_SR);

		// read "length" byte data from slave
		commandcode = 0x52;
		I2C_MasterSendData(&I2C1Handle, &commandcode, 1, SLAVE_ADDR, I2C_ENABLE_SR);

		I2C_MasterReceiveData(&I2C1Handle, rx_buffer, len, SLAVE_ADDR, I2C_DISABLE_SR);

		rx_buffer[len + 1]='\0'; // gan null vào sau vị trí cuối cùng của data
		printf("Data: %s",rx_buffer);
	}

}


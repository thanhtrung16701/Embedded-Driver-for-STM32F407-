/*
 * 008_SPI_CMD_Handle.c
 *
 *  Created on: Jan 17, 2024
 *      Author: ThanhPC
 */


#include <stm32f4xx.h>
#include <string.h>

#define CMD_LED_CTRL 		0x50
#define CMD_SENSOR_READ  	0x51
#define CMD_LED_READ		0x52
#define CMD_PRINT			0x53
#define CMD_ID_READ			0x54

#define LED_OFF				0
#define LED_ON				1

// analog pin
#define ANALOG_PIN0			0
#define ANALOG_PIN1			1
#define ANALOG_PIN2			2
#define ANALOG_PIN3			3
#define ANALOG_PIN4			4
#define ANALOG_PIN5			5

#define LED_PIN 			9

extern void initialise_monitor_handles(void);

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

	SPIPins.GPIO_PinCfg.GPIO_PinNumber = GPIO_PIN_14;
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
	SPI2handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV32; // sclk 2Mhz
	SPI2handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPI_Config.SPI_SSM = SPI_SSM_DI; // HW enable for NSS
	SPI_Init(&SPI2handle);

}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t gBTN, GpioLed;
	gBTN.pGPIOx = GPIOA;
		gBTN.GPIO_PinCfg.GPIO_PinNumber = GPIO_PIN_0;
		gBTN.GPIO_PinCfg.GPIO_PinMode = GPIO_MODE_IN;
		gBTN.GPIO_PinCfg.GPIO_PinSpeed = GPIO_SPEED_FAST;
		gBTN.GPIO_PinCfg.GPIO_PinPuPdControl = GPIO_NO_PUPD;

		GPIO_Init(&gBTN);

		//this is led gpio configuration
			GpioLed.pGPIOx = GPIOD;
			GpioLed.GPIO_PinCfg.GPIO_PinNumber = GPIO_PIN_12;
			GpioLed.GPIO_PinCfg.GPIO_PinMode = GPIO_MODE_OUT;
			GpioLed.GPIO_PinCfg.GPIO_PinSpeed = GPIO_SPEED_FAST;
			GpioLed.GPIO_PinCfg.GPIO_PinOutputType = GPIO_OP_OD;
			GpioLed.GPIO_PinCfg.GPIO_PinPuPdControl = GPIO_NO_PUPD;

			GPIO_Init(&GpioLed);
}

uint8_t SPI_ResponseVerify(uint8_t ACK)
{
	if(ACK == (uint8_t)0xF5)
	{
		return 1;
	}
	return 0;
}

int main(int argc, char **argv) {

//	initialise_monitor_handles();
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

	printf("Application is running\n");


	GPIO_ButtonInit();
	SPI2_GPIOInits();
	SPI2_Inits();

	printf("SPI init. Done \n");
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		while( !(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0)) );
		delay();

		SPI_PeriControl(SPI2, ENABLE);

		// 1. CMD_LED_CTRL <pin_num (1byte)> <value (1byte)>

		uint8_t cmd_code = CMD_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];

		// send cmd (data and dummy byte)
		SPI_SendData(SPI2, &cmd_code, 1);

		// Read dummy byte to clear off RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// send dummy byte (báo có phan hoi tu slave)
		SPI_SendData(SPI2, &dummy_write, 1);

		// Read ACK byte received -> fetch ACK from slave
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if(SPI_ResponseVerify(ackbyte) )	// response return true
		{
			// send arguments
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_SendData(SPI2, args, 2);
			printf("CMD_LED_CTRL executed \n");
		}
		// end of CMD_LED_CTRL


		// 2. CMD_SENSOR_READ <analog pin(1)>

		while( !(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0)) );
		// to avoid button debouncing
		delay();

		cmd_code = CMD_SENSOR_READ;


		// send cmd (data and dummy byte)
		SPI_SendData(SPI2, &cmd_code, 1);

		// Read dummy byte to clear off RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// send dummy byte (báo có phan hoi tu slave)
		SPI_SendData(SPI2, &dummy_write, 1);

		// Read ACK byte received -> fetch ACK from slave
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if(SPI_ResponseVerify(ackbyte) )	// response return true
		{
			// send arguments to slave
			args[0] = ANALOG_PIN0;
			SPI_SendData(SPI2, args, 1); // send 1 byte

			// do dummy read to clear RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);
			// insert delay so that slave can read the analog value
			// < slave does ADC conversion on that pin >
			delay();

			// send dummy byte to fetch the response from slave
			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t analog_read;
			SPI_ReceiveData(SPI2, &analog_read, 1);
			printf("CMD_SENSOR_READ executed %d \n",analog_read);
		}
		// end of CMD_SENSOR_READ



		// 3. CMD_LED_READ <pin(1)>
		while( !(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0)) );
		// to avoid button debouncing
		delay();

		cmd_code = CMD_LED_READ;

		//send cmd
		SPI_SendData(SPI2, &cmd_code, 1);

		// read dummy byte to clear RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// send dummy byte (response from slave)
		SPI_SendData(SPI2, &dummy_write, 1);

		// read ACK received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		if (SPI_ResponseVerify(ackbyte) ) {
			args[0] = LED_PIN;

		// send argument
			SPI_SendData(SPI2, args, 1);

		// read dummy byte to clear RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			delay();	// slave can ready with data, ~ 200ms

		// send dummy byte  to fetch response
			SPI_SendData(SPI2, &dummy_write, 1);

			uint8_t led_status;
			SPI_ReceiveData(SPI2, &led_status, 1);
			printf("CMD_LED_READ executed : %d\n",led_status);
		}
		// end of CMD_LED_READ

		// 4. CMD_PRINT <len>  <message>
		//wait till button is pressed
		while(! (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0) ) );

		delay();	// to avoid button debouncing

		cmd_code = CMD_PRINT;

		// send cmd
		SPI_SendData(SPI2, &cmd_code, 1);

		//send dummy byte to clear RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// send dummy byte to fetch response
		SPI_SendData(SPI2, &dummy_write, 1);

		// read the ACK received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		uint8_t mes[] = "Hello! How are you";

		if (SPI_ResponseVerify(ackbyte)) {
			args[0]= strlen((char*)mes);

			//send arguments
			SPI_SendData(SPI2, args, 1); // send length

			// read dummy byte to clear RXNE
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			delay(); // slave can ready with data

			// send message
			for (int i = 0; i < args[0]; i++) {
				SPI_SendData(SPI2, &mes[i], 1);
				// read dummy byte
				SPI_ReceiveData(SPI2, &dummy_read, 1);
			}
			printf("CMD_PRINT executed \n");
		}
		// end of CMD_PRINT



		// 5. CMD_ID_READ
		// wait till button is pressed
		while(! (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0)) );

		delay(); // to avoid button debouncing

		cmd_code = CMD_ID_READ;

		// cmd send
		SPI_SendData(SPI2, &cmd_code, 1);

		// read dummy byte to clear RXNE
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// send dummy byte to fetch response
		SPI_SendData(SPI2, &dummy_write, 1);

		// read ACK received
		SPI_ReceiveData(SPI2, &ackbyte, 1);

		uint8_t ID[10];
		uint32_t i = 0;
		if (SPI_ResponseVerify(ackbyte) )
		{
			// read 10 byte from slave
			for (i = 0; i < 10; i++)
			{
				//send data to fetch response
				SPI_SendData(SPI2, &dummy_write, 1);
				SPI_ReceiveData(SPI2, &ID[i], 1);
			}
			ID[10]= '\0'; // gan null vào ptu thu 10
			printf("CMD_ID : %s \n",ID);
		}
		// end of CMD_ID

		// confirm SPI not busy
		while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG) );

		SPI_PeriControl(SPI2, DISABLE);

		printf("SPI communication closed \n");
	}
}


/*
 * stm32f407_spi_driver.h
 *
 *  Created on: Jan 12, 2024
 *      Author: ThanhPC
 */

#ifndef INC_STM32F407_SPI_DRIVER_H_
#define INC_STM32F407_SPI_DRIVER_H_

#include <stm32f4xx.h>

typedef struct
{
	uint8_t	SPI_DeviceMode;
	uint8_t	SPI_BusConfig;
	uint8_t	SPI_SclkSpeed;
	uint8_t	SPI_DFF;
	uint8_t	SPI_CPHA;
	uint8_t	SPI_CPOL;
	uint8_t	SPI_SSM;

}SPI_Config_t;

typedef struct
{
	SPI_RegDef_t	*SPIx;
	SPI_Config_t	SPI_Config;
	uint8_t 		*pTXBuffer; // to store the application of TX buffer addr
	uint8_t 		*pRXBuffer; // to store the application of RX buffer addr
	uint8_t 		TxLen;		// to store TX length
	uint8_t 		RxLen;		// to store RX length
	uint8_t			TxState;	// to store Tx State
	uint8_t			RxState;	// to store Rx State
}SPI_Handle_t;

/*
 * SPI application states
 */
#define 	SPI_READY			0
#define 	SPI_BUSY_IN_RX		1
#define 	SPI_BUSY_IN_TX		2

/*
 * SPI Application events
 */
#define SPI_EVENT_TX 		1
#define SPI_EVENT_RX 		2
#define SPI_EVENT_OVR_ER 	3
#define SPI_EVENT_CRC		4

/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER 		1
#define SPI_DEVICE_MODE_SLAVE		0

/*
 * @ SPI_BusConfig
 */
#define SPI_BUS_CFG_FD					1
#define SPI_BUS_CFG_HD					2
#define SPI_BUS_CFG_SIMPLEX_RX			3

/*
 * @SPI_SclkSpeed - Baud rate control
 */
#define SPI_SCLK_SPEED_DIV2			0
#define SPI_SCLK_SPEED_DIV4			1
#define SPI_SCLK_SPEED_DIV8			2
#define SPI_SCLK_SPEED_DIV16		3
#define SPI_SCLK_SPEED_DIV32		4
#define SPI_SCLK_SPEED_DIV64		5
#define SPI_SCLK_SPEED_DIV128		6
#define SPI_SCLK_SPEED_DIV256		7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS		0
#define SPI_DFF_16BITS		1

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_HIGH	1
#define SPI_CPHA_LOW	0

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_LOW	0
#define SPI_CPOL_HIGH	1

/*
 * @SPI_SSM
 */
#define SPI_SSM_DI	0
#define SPI_SSM_EN	1


/*
 * SPI related status flags definition
 */
#define 	SPI_TXE_FLAG		( 1 << SPI_SR_TXE)
#define 	SPI_RXNE_FLAG		( 1 << SPI_SR_RXNE)
#define 	SPI_BSY_FLAG		( 1 << SPI_SR_BSY)


/*
 *  Peripheral Clock setup
 */
void SPI_PeriClockCtrl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


/*
 * Data Send and Receive
 * Buffer : vung nho
 */

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTXBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t len);
// SPI INTERRUPT
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTXBuffer, uint32_t len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRXBuffer, uint32_t len);

/*
 * IRQ Config and ISR handling
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_Handling(SPI_Handle_t *pHandle);

/*
 * Other peripheral control APIs
 */
void SPI_PeriControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi); // SPE En or Di
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi); // SS op enable(EN or DI)
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi); // Internal SS EN or DI
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);


void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTranmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Application callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);

#endif /* INC_STM32F407_SPI_DRIVER_H_ */

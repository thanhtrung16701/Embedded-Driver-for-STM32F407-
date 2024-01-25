/*
 * stm32f407_spi_driver.c
 *
 *  Created on: Jan 12, 2024
 *      Author: ThanhPC
 */

#include <stm32f407_spi_driver.h>

// helper functions -> line 255
static void spi_tx_handleIT(SPI_Handle_t *pHandle);
static void spi_rx_handleIT(SPI_Handle_t *pHandle);
static void spi_ovr_handleIT(SPI_Handle_t *pHandle);

/*Peripheral clock setup */

/*********************************
 * @fn 				- SPI_PeriClockCtrl
 *
 * @brief			- enable or disable peripheral clock for given SPI port
 *
 * @param[in]		- base addr of GPIO peripheral
 * @param[in]		- En or Di Clock
 * @param[in]		-
 *
 * @return			- none
 *
 * @note			- none
 *
 */

void SPI_PeriClockCtrl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pSPIx == SPI1)			SPI1_PCLK_EN();
		else if(pSPIx == SPI2)		SPI2_PCLK_EN();
		else if(pSPIx == SPI3)		SPI3_PCLK_EN();
		else if(pSPIx == SPI4)		SPI4_PCLK_EN();
	} else {
		if(pSPIx == SPI1)				SPI1_PCLK_DI();
		else if(pSPIx == SPI2)			SPI2_PCLK_DI();
		else if(pSPIx == SPI3)			SPI3_PCLK_DI();
		else if(pSPIx == SPI4)			SPI4_PCLK_DI();
	}
}


/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	// Enable the peripheral clock control
	SPI_PeriClockCtrl(pSPIHandle->SPIx, ENABLE);

	// Config SPI_CR1 register
	uint32_t temp = 0;

	// 1. Config the device mode master or slave
		temp |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;

	// 2. Config the bus_cfg
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CFG_FD)
	{
	// clear the BIDI mode (bit 15)
		temp &= ~(1 << SPI_CR1_BIDI_MODE);

	} else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CFG_HD) {
	//set the BIBI mode
		temp |= 1 << SPI_CR1_BIDI_MODE;

	}else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CFG_SIMPLEX_RX) {
	// clear the bidi mode
		temp &= ~(1 << SPI_CR1_BIDI_MODE);
		// set the RX only
		temp |= 1 << SPI_CR1_RXONLY;
	}

	// 3. Config the serial Clock Speed (baud rate)
		temp |= pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR;

	// 4. Config the SPI_DFF
		temp |= pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF;

	// 5. Config SPI_CPHA
		temp |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;

	// 6. Config SPI_CPOL
		temp |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;

	// 7. Config SPI_SSM
		temp |= pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->SPIx->CR1 = temp;
}

void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1){
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2){
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3){
		SPI3_REG_RESET();
	}
	else if(pSPIx == SPI4){
		SPI4_REG_RESET();
	}
}


/*
 * Data Send and Receive
 * Buffer : vung nho
 */

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)	{
		return FLAG_SET;
	}
		return FLAG_RESET;
}
/*********************************
 * @fn 				- SPI_SendData
 *
 * @brief			-
 *
 * @param[in]		- pSPIx
 * @param[in]		- pTXBuffer
 * @param[in]		- len
 *
 * @return			- none
 *
 * @note			- this is blocking call
 *
 */

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTXBuffer, uint32_t len)
{
	while(len > 0)
	{
		// 1. Wait until TXE is set
//		while(! (pSPIx->DR & (1 << 1) )  ); // (pSPIx->DR & (1 << 1) != (1 << 1);

		// while = 1 true, = 0 false TX is set and come out of the loop
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		// 2. Check the DFF in CR1
		if( pSPIx->CR1 & ( 1 << SPI_CR1_DFF)) // true pSPIx->CR1 bit = 1
		{
			// 16 bit
			// load data from TX buffer (1 byte) to the DR : 8 bit  + pTX 8 bit
			pSPIx->DR = *((uint16_t*)pTXBuffer);
			len--;
			len--; // 16 bit -> minus 2 times
			(uint16_t*)pTXBuffer++;		// increment to  point to next data item
		}else {
			pSPIx->DR = *pTXBuffer;
			len--;
			pTXBuffer++;	// tro toi dia chi tiep theo
		}

	}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t len)
{
	while(len > 0)
	{
		// 1. Wait until RXNE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_SET);
		// if DFF set = 16 bits else = 8 bits
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF) )
		{
			// load 2 bytes from DR to RX buffer addr -> uint16_t
			// read from the DR
			*((uint16_t *)pRXBuffer) =  pSPIx->DR;
			len--;
			len--;
			(uint16_t*)pRXBuffer++;	// increment to receive new
		} else {
			*pRXBuffer = pSPIx->DR;
			len--;
			pRXBuffer++;
		}
	}
}



/*
 * IRQ Config and ISR handling
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE) {
		if (IRQNumber < 32) {
			*NVIC_ISER0 |= 1 << IRQNumber;
		}
		else if (IRQNumber >= 32 && IRQNumber < 64) {
			*NVIC_ISER1 |= 1 << (IRQNumber % 32);
		}
		else if (IRQNumber >= 64 && IRQNumber < 96) {
			*NVIC_ISER2 |= 1 << (IRQNumber % 64);
		}
	} else {
		if (IRQNumber < 32) {
			*NVIC_ICER0 |= 1 << IRQNumber;
		}
		else if (IRQNumber >= 32 && IRQNumber < 64) {
			*NVIC_ICER1 |= 1 << (IRQNumber % 32);
		}
		else if (IRQNumber >= 64 && IRQNumber < 96) {
			*NVIC_ICER2 |= 1 << (IRQNumber % 64);
		}

	}
}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	// find out ipr reg -> 32 bit IN a IPR, 4 section
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	// 8 bit in section and not use 4 bit low-level
	uint8_t shift_amount = 8 * iprx_section + (8 - NO_PR_BIT_IMPLEMENTED);

	*(NVIC_IPR_BASE_ADDR + iprx) |= IRQPriority << shift_amount;
}


void SPI_Handling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;
	// check for TXE
	temp1 = pHandle->SPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->SPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if (temp1 && temp2) {
		spi_tx_handleIT(pHandle);
	}
	// Check for RXNE
	temp1 = pHandle->SPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->SPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if (temp1 && temp2) {
		spi_rx_handleIT(pHandle);
	}

	// Check for Overrun
	temp1 = pHandle->SPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->SPIx->CR2 & (1 << SPI_CR2_ERRIE);
	if (temp1 && temp2) {
		spi_ovr_handleIT(pHandle);
	}

}


/*
 * Other peripheral control APIs
 */

void SPI_PeriControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE) // bit 6 CR1 -> SPI peripheral enable(SPE)
	{
		pSPIx->CR1 |= 1 << SPI_CR1_SPE;
	}else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}
/*********************************
 * @fn 				- SPI_SSIConfig
 *
 * @brief			-  EN SSM and master TX  before SPI EN
 *
 * @param[in]		- pSPIx
 * @param[in]		- EnOrDi
 * @param[in]		-
 *
 * @return			- none
 *
 * @note			- make NSS signal internally high and avoid MODF error
 *
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= 1 << SPI_CR1_SSI;
	}else {
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}
/*********************************
 * @fn 				- SPI_SSOEConfig
 *
 * @brief			-  make NSS output enable (pulled to low)
 * 						when disable SSM and master TX only
 *
 * @param[in]		- pSPIx
 * @param[in]		- EnOrDi
 * @param[in]		-
 *
 * @return			- none
 *
 * @note			- SPI = 1, EN -> NSS = 0 , SPI = 0 -> DI
 *
 */

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE) {
		pSPIx->CR2 |= 1 << SPI_CR2_SSOE;
	} else {
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

/*
 * SPI INTERRUPT
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTXBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX)
	{
	// 1. Save TX buffer addr and length info in some global vars
	pSPIHandle->pTXBuffer = pTXBuffer;
	pSPIHandle->TxLen = len;

	// 2. Mark SPI state as BUSY in transmission
	// 	-> no other code take over the same API TILL transmission is OVER
	pSPIHandle->TxState = SPI_BUSY_IN_TX;

	//3. En TXEIE bit (to get IT whenever TX flag is set in CR2)
	pSPIHandle->SPIx->CR2 |= 1 << SPI_CR2_TXEIE;
	// 4. Data transmission will be handled by the ISR
	}
	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRXBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->RxState;
	if (state != SPI_BUSY_IN_RX) {
		// 1. Save RX buffer addr and length info in some global vars
		pSPIHandle->pRXBuffer = pRXBuffer;
		pSPIHandle->RxLen = len;

			// 2. Mark SPI state as BUSY in reception
			// 	-> no other code take over the same API TILL reception is OVER
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. En RXNEIE bit (to get IT whenever RX flag is set in SR2)
		pSPIHandle->SPIx->CR2 |= 1 << SPI_CR2_RXNEIE;

			// 4. Data transmission will be handled by the ISR
	}

	return state;
}


void SPI_CloseTranmission(SPI_Handle_t *pSPIHandle)
{
	// 1. Reset TXEIE -> prevent IT from setting up of TXE flag
	pSPIHandle->SPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	// clear TX buffer
	pSPIHandle->pTXBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;

}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	// 1. Reset RXNEIE -> prevent IT from setting up of RXNE flag
	pSPIHandle->SPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	// clear TX buffer
	pSPIHandle->pRXBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	// clear the ovr flag
		temp = pSPIx->DR;
		temp = pSPIx->SR;
	(void)temp;
}

/***************** Helper function for SPI_Handling ***********************************
 * spi_tx_handleIT
 * spi_rx_handleIT
 * spi_ovr_handleIT
 *
 */

static void spi_tx_handleIT(SPI_Handle_t *pHandle)
{
	// if DFF set = 16 bits else = 8 bits
	if (pHandle->SPIx->CR1 & (1 << SPI_CR1_DFF) )
	{
		// load data from TX buffer (1 byte) to the DR : 8 bit  + pTX 8 bit

		pHandle->SPIx->DR = *((uint16_t*)pHandle->pTXBuffer);
		pHandle->TxLen--;
		pHandle->TxLen--;
		(uint16_t*)pHandle->pTXBuffer++;	// increment to receive new
	} else {
		pHandle->SPIx->DR = *pHandle->pTXBuffer;
		pHandle->TxLen--;
		pHandle->pTXBuffer++;
	}

	if(! pHandle->TxLen)	// TxLen is zero
	{
		// close the transmission AND inform TX is over
		SPI_CloseReception(pHandle);
		SPI_ApplicationEventCallback(pHandle,SPI_EVENT_TX);
	}
}



static void spi_rx_handleIT(SPI_Handle_t *pHandle)
{
	// if DFF = set = 16 bits else = 8 bits
	if(pHandle->SPIx->CR1 & (1 << SPI_CR1_DFF) )
	{
		// load 2 bytes from DR to RX buffer addr -> uint16_t
		// read from the DR
		*((uint16_t*)pHandle->pRXBuffer) = (uint16_t)pHandle->SPIx->DR;
		pHandle->RxLen -= 2 ;
		pHandle->pRXBuffer++;
		pHandle->pRXBuffer++;
	}
	else {
		*pHandle->pRXBuffer = (uint8_t)pHandle->SPIx->DR;
		pHandle->RxLen--;
		pHandle->pRXBuffer++;
	}

	if (! pHandle->RxLen) {	//RxLen is zero
		// close the reception and inform RX is over
		spi_rx_handleIT(pHandle);
		SPI_ApplicationEventCallback(pHandle,SPI_EVENT_RX);
	}
}

static void spi_ovr_handleIT(SPI_Handle_t *pHandle)
{
	uint8_t temp;
	// clear the ovr flag
	if (pHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pHandle->SPIx->DR;
		temp = pHandle->SPIx->SR;
	}
	(void)temp;

	// inform the app
	SPI_ApplicationEventCallback(pHandle,SPI_EVENT_TX);
}



__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEvent){
	// this app can override this function
}


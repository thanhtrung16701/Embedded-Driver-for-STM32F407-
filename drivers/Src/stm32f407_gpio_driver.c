/*
 * stm32f407_gpio_driver.c
 *
 *  Created on: Jan 4, 2024
 *      Author: ThanhPC
 */
#include <stm32f407_gpio_driver.h>

/*Peripheral clock setup */

/*********************************
 * @fn 				- GPIO_PeriClockCtrl
 *
 * @brief			- enable or disable peripheral clock for given GPIO port
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
void GPIO_PeriClockCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		if(pGPIOx == GPIOA)				GPIOA_PCLK_EN();
		else if (pGPIOx == GPIOB)	 	GPIOB_PCLK_EN();
		else if (pGPIOx == GPIOC) 		GPIOC_PCLK_EN();
		else if (pGPIOx == GPIOD)	 	GPIOD_PCLK_EN();
		else if (pGPIOx == GPIOE) 		GPIOE_PCLK_EN();
		else if (pGPIOx == GPIOF)	 	GPIOF_PCLK_EN();
		else if (pGPIOx == GPIOG) 		GPIOG_PCLK_EN();
		else if (pGPIOx == GPIOH)	 	GPIOH_PCLK_EN();
		else if (pGPIOx == GPIOI) 		GPIOI_PCLK_EN();
	} else {
		if(pGPIOx == GPIOA)				GPIOA_PCLK_DI();
		else if (pGPIOx == GPIOB)	 	GPIOB_PCLK_DI();
		else if (pGPIOx == GPIOC) 		GPIOC_PCLK_DI();
		else if (pGPIOx == GPIOD)	 	GPIOD_PCLK_DI();
		else if (pGPIOx == GPIOE) 		GPIOE_PCLK_DI();
		else if (pGPIOx == GPIOF)	 	GPIOF_PCLK_DI();
		else if (pGPIOx == GPIOG) 		GPIOG_PCLK_DI();
		else if (pGPIOx == GPIOH)	 	GPIOH_PCLK_DI();
		else if (pGPIOx == GPIOI) 		GPIOI_PCLK_DI();
		}
	}
/*********************************
 * @fn 				- GPIO_Init
 *
 * @brief			- detect interrupt or not. Init functions
 * @param[in]		- gpio pin	-> GPIO_PinMode
 * @param[in]		- speed    	-> GPIO_PinSpeed
 * @param[in]		- pupd	   	-> GPIO_PinPuPdControl
 * @param[in]		- optype	-> GPIO_PinOutputType
 * @param[in]		- alt func	-> GPIO_MODE_ALFN
 *
 * @return			- none
 *
 * @note			- none
 *
 */

/*
 * Init and de-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	// Enable the peripheral clock
	GPIO_PeriClockCtrl(pGPIOHandle->pGPIOx, ENABLE);

	uint32_t temp = 0;
	/*
	 * cfg the mode of gpio pin
	 */
	if (pGPIOHandle->GPIO_PinCfg.GPIO_PinMode <= GPIO_MODE_ANALOG) {
		temp = (pGPIOHandle->GPIO_PinCfg.GPIO_PinMode << (2 *pGPIOHandle->GPIO_PinCfg.GPIO_PinNumber));
		//clear bit pin_num
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinCfg.GPIO_PinNumber); // clr 2 bit
		pGPIOHandle->pGPIOx->MODER |=  temp; // -> MODER : physical register
		temp = 0;
	}
	else {
	// interrupt
		if (pGPIOHandle->GPIO_PinCfg.GPIO_PinMode == GPIO_MODE_IT_FT) {
			// 1. config the FT selection register
			EXTI->FTSR |= 1 << pGPIOHandle->GPIO_PinCfg.GPIO_PinNumber;
			//clear the corresponding FTSR bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinCfg.GPIO_PinNumber);

		}else if (pGPIOHandle->GPIO_PinCfg.GPIO_PinMode == GPIO_MODE_IT_RT) {

			// 2. config the RTSR
			EXTI->RTSR |= 1 << pGPIOHandle->GPIO_PinCfg.GPIO_PinNumber;
			// clear the corresponding RTSR bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinCfg.GPIO_PinNumber);

		}else if (pGPIOHandle->GPIO_PinCfg.GPIO_PinMode == GPIO_MODE_IT_FRT) {

			// config both FTSR and RTSR
			EXTI->FTSR |= 1 << pGPIOHandle->GPIO_PinCfg.GPIO_PinNumber;

			EXTI->RTSR |= 1 << pGPIOHandle->GPIO_PinCfg.GPIO_PinNumber;
		}
		// 2. config the GPIO port selection in SYSCFG_EXTI control register

		uint8_t temp1 = pGPIOHandle->GPIO_PinCfg.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinCfg.GPIO_PinNumber % 4;

		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		//  before config SYSCFG, enable SYSCFG clock
		SYSCFG_PCLK_EN();

		SYSCFG->EXTICR[temp1] = portcode << (4 * temp2);

		//3. Enable the EXTI delivery using interrupt mask reg (IMR)
			EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinCfg.GPIO_PinNumber;
	}


	/*
	 * cfg the speed
	 */
	temp = (pGPIOHandle->GPIO_PinCfg.GPIO_PinSpeed << (2 *pGPIOHandle->GPIO_PinCfg.GPIO_PinNumber));
	//clear bit pin_num
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinCfg.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;


	/*
	 * cfg the pupd settings
	 */
	temp = (pGPIOHandle->GPIO_PinCfg.GPIO_PinPuPdControl << (2 *pGPIOHandle->GPIO_PinCfg.GPIO_PinNumber));
	//clear bit pin_num
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinCfg.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;


	/*
	 * cfg the optype
	 */
	temp = (pGPIOHandle->GPIO_PinCfg.GPIO_PinOutputType << pGPIOHandle->GPIO_PinCfg.GPIO_PinNumber);
	//clear bit pin_num
	pGPIOHandle->pGPIOx->OTYPER &= ~(1 << pGPIOHandle->GPIO_PinCfg.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;


	/*
	 * cfg alt func
	 */
	if (pGPIOHandle->GPIO_PinCfg.GPIO_PinMode == GPIO_MODE_ALFN) {
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinCfg.GPIO_PinNumber / 8;		//integer
		temp2 = pGPIOHandle->GPIO_PinCfg.GPIO_PinNumber % 8;	//remainder or balance
		// clear bit
		pGPIOHandle->pGPIOx->AFR[temp1]	&= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinCfg.GPIO_PinAltFunMode << (4 * temp2));

	}
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)				GPIOA_REG_RESET();
	else if (pGPIOx == GPIOB)	 	GPIOB_REG_RESET();
	else if (pGPIOx == GPIOC) 		GPIOC_REG_RESET();
	else if (pGPIOx == GPIOD)	 	GPIOD_REG_RESET();
	else if (pGPIOx == GPIOE) 		GPIOE_REG_RESET();
	else if (pGPIOx == GPIOF)	 	GPIOF_REG_RESET();
	else if (pGPIOx == GPIOG) 		GPIOG_REG_RESET();
	else if (pGPIOx == GPIOH)	 	GPIOH_REG_RESET();
	else if (pGPIOx == GPIOI) 		GPIOI_REG_RESET();
}

/*********************************
 * @fn 				- read input pin and port
 *
 * @brief			-
 * @param[in]		-
 *
 * @return			- 0 or 1
 *
 * @note			- read entire port -> point to IDR / pin need mask with 0
 *
 */

/*
 *Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber ) & 0x0000001);
	return value;
}


uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	// write 1 to to the output data register at the bit field
	if (Value == GPIO_PIN_SET) {
		pGPIOx->ODR |= (1 << PinNumber);
	} else {
		// write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;	// copy Value to ODR cuz write to whole port
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= 1 << PinNumber;	// Toggle bit ^= 1 << n
}


/********************************* IRQ - Interrupt Request *******************
 * @fn 				- GPIO_IRQInterruptConfig
 *
 * @brief			- Set enable and clear enale
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- none
 *
 * @note			- 0 - 31, 32 - 63, 64 - 95
 *
 */


void GPIO_IRQInterruptConfig(uint8_t IRQNumber , uint8_t EnorDi)
{
	if (EnorDi == ENABLE) {
		if(IRQNumber < 32) 	{
			*NVIC_ISER0 |= 1 << IRQNumber;
		}

		else if (IRQNumber >= 32 && IRQNumber < 64) {
			*NVIC_ISER1 |= 1 << (IRQNumber % 32);
		}
		else if (IRQNumber >=64 && IRQNumber < 96) {
			*NVIC_ISER2 |= 1 << (IRQNumber % 64);
		}
	} else {
		if(IRQNumber < 32) {
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

/*********************************
 * @fn 				- GPIO_IRQPriorityConfig
 *
 * @brief			- know which Priority value
 * @param[in]		-
 * @param[in]		-
 * @param[in]		-
 *
 * @return			- none
 *
 * @note			- none
 *
 */

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// find out ipr reg -> 32 bit IN a IPR, 4 section
		uint8_t iprx = IRQNumber / 4;
		uint8_t iprx_section = IRQNumber % 4;
		uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BIT_IMPLEMENTED); // 8 bit in section

		*(NVIC_IPR_BASE_ADDR + iprx) |= IRQPriority << shift_amount;
}

/*********************************
 * @fn 				- GPIO_IRQHanding
 *
 * @brief			- check bit tương ứng  PinNumber trong thanh ghi PR
 * 						của EXTI có được đặt (pending) hay không.

 * @return			- none
 *
 * @note			- // EXTI->PR & (1 << PinNumber) == (1 << PinNumber)
 * 						giả sử thanh ghi PR có giá trị là 0b00110010. Khi PinNumber là 3, ta có:

						//(1 << PinNumber) = 0b00001000
						//EXTI->PR & (1 << PinNumber) = 0b00110010 & 0b00001000 = 0b00000000
						 * kết quả của phép AND là 0, tức là bit tương ứng trong
						 * thanh ghi PR không được đặt (pending)
 *						->  if sẽ không được thực hiện
 */

void GPIO_IRQHanding(uint8_t PinNumber)
{
	// clear the pr register corresponding the pin number
	if (EXTI->PR & (1 << PinNumber)) {

		EXTI->PR |= 1 << PinNumber; // set pending bit
	}
}


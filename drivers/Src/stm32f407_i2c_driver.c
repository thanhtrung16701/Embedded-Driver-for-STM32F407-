/*
 * stm32f407_i2c_driver.c
 *
 *  Created on: Jan 25, 2024
 *      Author: ThanhPC
 */
#include <stm32f407_i2c_driver.h>
/*
 *  Peripheral Clock setup
 */

void I2C_PeriClockCtrl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE) {
		if(pI2Cx == I2C1) 			I2C1_PCLK_EN();
		else if(pI2Cx == I2C2)		I2C2_PCLK_EN();
		else if(pI2Cx == I2C3)		I2C3_PCLK_EN();
	} else {
		 if(pI2Cx == I2C1)				I2C1_PCLK_DI();
		 else if(pI2Cx == I2C2)			I2C2_PCLK_DI();
		 else if(pI2Cx == I2C3)			I2C3_PCLK_DI();
	}
}

/*
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{

}

void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if (pI2Cx == I2C1) {
		I2C1_REG_RESET();
	}else if (pI2Cx == I2C2) {
		I2C2_REG_RESET();
	}else if (pI2Cx == I2C3) {
		I2C3_REG_RESET();
	}
}



/*
 * IRQ Config and ISR handling
 */

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if(IRQNumber < 32){
			*NVIC_ICER0 |= 1 << IRQNumber;
		}else if (IRQNumber >= 32 && IRQNumber < 64) {
			*NVIC_ICER1 |= 1 << (IRQNumber % 32);
		}else if (IRQNumber >= 64 && IRQNumber < 96) {
			*NVIC_ICER2 |= 1 << (IRQNumber % 64);
		}
	} else {
		if (IRQNumber < 32) {
			*NVIC_ICER0 &= ~(1 << IRQNumber);
		}else if (IRQNumber >= 32 && IRQNumber < 64) {
			*NVIC_ICER1 &= ~(1 <<(IRQNumber % 32) );
		}else if (IRQNumber >= 64 && IRQNumber < 96) {
			*NVIC_ICER2 &= ~(1 << (IRQNumber % 64) );
		}
	}
}

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	// find out ipr reg -> 32 bit IN a IPR, 4 section
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	// 8 bit in section and not use 4 bit low-level
	uint8_t shift_amount = 8 * iprx_section + (8 - NO_PR_BIT_IMPLEMENTED);
	*(NVIC_IPR_BASE_ADDR + iprx) |= IRQPriority << shift_amount;
}

/*
 * Other peripheral control APIs
 */
void I2C_PeriControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi); // SPE En or Di
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

/*
 * stm32f407_i2c_driver.h
 *
 *  Created on: Jan 25, 2024
 *      Author: ThanhPC
 */

#ifndef INC_STM32F407_I2C_DRIVER_H_
#define INC_STM32F407_I2C_DRIVER_H_
#include <stm32f4xx.h>



/*
 * Config struct I2Cx peripherals
 */
typedef struct{
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddr;	// 7 bit wide -> uint8_t
	uint8_t  I2C_ACKControl;
	uint16_t I2C_FMDutyCycle;
}I2C_Config_t;

/*
 * Handle struct for I2Cx
 */
typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
}I2C_Handle_t;

/*
 * @ I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM 	100000		// Standard mode
#define I2C_SCL_SPEED_FM4K 	400000		// Fast mode
#define I2C_SCL_SPEED_FM2K 	200000

/*
 * @I2C_ACKControl -> I2C_CR1 ACK
 */
#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

/*
 * @I2C_FMDutyCycle -> I2C_CCR DUTY
 */
#define I2C_FM_DUTY_2			0
#define I2C_FM_DUTY_16_9		1


//******************************* APIs ********************************
/*
 *  Peripheral Clock setup
 */
void I2C_PeriClockCtrl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

/*
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);



/*
 * IRQ Config and ISR handling
 */

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);

/*
 * Other peripheral control APIs
 */
void I2C_PeriControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi); // SPE En or Di
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);


/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv);

#endif /* INC_STM32F407_I2C_DRIVER_H_ */

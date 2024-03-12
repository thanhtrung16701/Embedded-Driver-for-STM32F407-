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
	uint8_t 	*pTXBuffer; /* store the application Tx buffer addr */
	uint8_t 	*pRXBuffer; /* store the application Rx buffer addr */
	uint32_t  	TxLen; 		/* store Tx len  */
	uint32_t  	RxLen; 		/* store Rx len  */
	uint8_t		TxRxState;	/* store Communication state */
	uint8_t		DevAddr;	/* store master/slave device addr */
	uint32_t 	RxSize;		/*	store Rx size */
	uint32_t 	Sr;			/* store repeated start value */


}I2C_Handle_t;
/*
 * I2C states
 */
#define I2C_READY		0
#define I2C_BUSY_TX		1
#define I2C_BUSY_RX		2

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

/*
 * I2C related status flags definition
 */
#define I2C_FLAG_ADDR		(1 << I2C_SR1_ADDR	)#define I2C_FLAG_SB	 		(1 << I2C_SR1_SB	)#define	I2C_FLAG_ADD10		(1 << I2C_SR1_ADD10 )		// 10-bit header sent (Master mode)
#define I2C_FLAG_STOPF		(1 << I2C_SR1_STOPF )		// Stop detection (slave mode)
#define I2C_FLAG_RXNE		(1 << I2C_SR1_RXNE	)	// Data register not empty (receivers)
#define I2C_FLAG_TXE		(1 << I2C_SR1_TXE	)	// Data register empty (transmitters)
#define I2C_FLAG_BTF		(1 << I2C_SR1_BTF	)	// Byte transfer finished
#define I2C_FLAG_BERR		(1 << I2C_SR1_BERR	)	// Bus error
#define I2C_FLAG_BERR		(1 << I2C_SR1_BERR	)	// Arbitration lost (master mode)
#define I2C_FLAG_AF			(1 << I2C_SR1_AF	)			// Acknowledge failure
#define I2C_FLAG_OVR		(1 << I2C_SR1_OVR	)				// Overrun/Underrun
#define I2C_FLAG_TIMEOUT	(1 << I2C_SR1_TIMEOUT)		// Timeout or Tlow error

#define I2C_DISABLE_SR  	RESET
#define I2C_ENABLE_SR   	SET

/*
 * I2C application events macros
 */
#define I2C_EV_TX_CMPLT  	 	  0
#define I2C_EV_RX_CMPLT  	  	1
#define I2C_EV_STOP       		2
#define I2C_ERROR_BERR 	 		  3
#define I2C_ERROR_ARLO  		  4
#define I2C_ERROR_AF    		  5
#define I2C_ERROR_OVR   		  6
#define I2C_ERROR_TIMEOUT 		  7
#define I2C_EV_DATA_REQ         8
#define I2C_EV_DATA_RCV         9

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
 * Data send and receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr);

// return application states: busy in tx, rx -> uint8_t
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr);

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);


/*
 * IRQ Config and ISR handling
 */

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);

/*
 * Other peripheral control APIs
 */
void I2C_PeriControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi); // SPE En or Di
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
void I2C_MangageACKing(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx); // En STOP bit in CR1


/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv);

#endif /* INC_STM32F407_I2C_DRIVER_H_ */

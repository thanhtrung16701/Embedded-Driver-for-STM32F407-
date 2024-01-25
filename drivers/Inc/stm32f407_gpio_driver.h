/*
 * stm32f407_gpio_driver.h
 *
 *  Created on: Jan 4, 2024
 *      Author: ThanhPC
 */

/*
****************************** Driver Specific Data ******************************
 */


#ifndef INC_STM32F407_GPIO_DRIVER_H_
#define INC_STM32F407_GPIO_DRIVER_H_

#include <stm32f4xx.h>

typedef struct
{
	uint8_t GPIO_PinNumber;				// possible from @GPIO PIN NUM
	uint8_t GPIO_PinMode;				// possible from @GPIO PIN MODE
	uint8_t GPIO_PinSpeed;				// possible from @GPIO OUTPUT SPEED
	uint8_t GPIO_PinPuPdControl;		// possible from @GPIO PUPD
	uint8_t GPIO_PinOutputType;			// possible from @GPIO OUTPUT TYPE
	uint8_t GPIO_PinAltFunMode;			// possible when GPIO_PinMode == GPIO_MODE_ALFN
}GPIO_PinConfig_t;

typedef struct
{
				// pointer to hold the base addr of peripheral
	GPIO_RegDef_t 		*pGPIOx; 				// hold base addr of GPIO port
	GPIO_PinConfig_t 	GPIO_PinCfg; 			// hold configuration setting
}GPIO_Handle_t;



/*
 * @GPIO PIN NUM
 *************** @Do not use for GPIO, use for debugging************
 *
•			 PA15: JTDI in pull-up
•			 PA14: JTCK/SWCLK in pull-down
• 			 PA13: JTMS/SWDAT in pull-up
• 			 PB4:  NJTRST in pull-up
• 			 PB3:  JTDO in floating state
 */
#define GPIO_PIN_0		0
#define GPIO_PIN_1		1
#define GPIO_PIN_2		2
#define GPIO_PIN_3		3
#define GPIO_PIN_4		4
#define GPIO_PIN_5		5
#define GPIO_PIN_6		6
#define GPIO_PIN_7		7
#define GPIO_PIN_8		8
#define GPIO_PIN_9		9
#define GPIO_PIN_10		10
#define GPIO_PIN_11		11
#define GPIO_PIN_12		12
#define GPIO_PIN_13		13
#define GPIO_PIN_14		14
#define GPIO_PIN_15		15


/*
 * @GPIO PIN MODE
 */
#define GPIO_MODE_IN 				0
#define GPIO_MODE_OUT 				1
#define GPIO_MODE_ALFN 				2
#define GPIO_MODE_ANALOG 			3
// INTERRUPT
#define GPIO_MODE_IT_FT				4
#define GPIO_MODE_IT_RT				5
#define GPIO_MODE_IT_FRT			6


 /*
  * @GPIO OUTPUT TYPE
  */
#define GPIO_OP_PP					0
#define GPIO_OP_OD					1

/*
 * @GPIO OUTPUT SPEED
 */
#define GPIO_SPEED_LOW 			0
#define GPIO_SPEED_MED 			1
#define GPIO_SPEED_FAST 		2
#define GPIO_SPEED_HGIGH	 	3

/*
 * @GPIO PUPD
 */

#define GPIO_NO_PUPD			0
#define GPIO_PU					1
#define GPIO_PD					2




//Peripheral clock setup
void GPIO_PeriClockCtrl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);


// Init and de-init
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


// Data read and write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

// IRQ config and IRQ handling
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t Priority);
void GPIO_IRQHanding(uint8_t PinNumber);	//know which pin interrupt is triggered



#endif /* INC_STM32F407_GPIO_DRIVER_H_ */

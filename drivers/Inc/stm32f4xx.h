/*
 * stm32f4xx.h
 *
 *  Created on: Jan 2, 2024
 *      Author: ThanhPC
 */

/****************************** MCU Specific Data *******************************/

#ifndef INC_STM32F4XX_H_
#define INC_STM32F4XX_H_

#include <stddef.h>
#include <stdint.h>

#define __weak  __attribute__((weak))
#define __vo volatile

// *************** PROCESSOR M4 SPECIFIC DETAIL ON   *********
/*
 * Arm cortex Mx NVIC ISERx register ADDR (interrupt set enable)
 */

#define NVIC_ISER0			(__vo uint32_t*)0xE000E100		//0 - 31
#define NVIC_ISER1			(__vo uint32_t*)0xE000E104		//32 - 63
#define NVIC_ISER2			(__vo uint32_t*)0xE000E108		//64 - 95
#define NVIC_ISER3			(__vo uint32_t*)0xE000E10C

/*
 * Arm cortex Mx NVIC ICERx register ADDR (interrupt clear enable)
 */

#define NVIC_ICER0			(__vo uint32_t*)0XE000E180
#define NVIC_ICER1			(__vo uint32_t*)0xE000E184
#define NVIC_ICER2			(__vo uint32_t*)0xE000E188
#define NVIC_ICER3			(__vo uint32_t*)0xE000E18C

/*
 * Arm cortex Mx NVIC IPRx register ADDR (interrupt priority )
 */
#define NVIC_IPR_BASE_ADDR			(__vo uint32_t*)0xE000E400

#define NO_PR_BIT_IMPLEMENTED		4			// 4 low-ordered bit

//Buoc 1: Dia chi chung

#define FLASH_BASE_ADDR 	0x08000000U
#define SRAM1_BASE_ADDR 	0x20000000U
#define SRAM 				SRAM1_BASE_ADDR
#define SRAM2_BASE_ADDR 	0x2001C000U
#define ROM_BASE_ADDR 		0x1FFF0000U

//Buoc 2: Dia chi bus chung

#define PER_BASE_ADDR 			0x40000000U
#define APB1PER_BASE_ADDR 		PER_BASE_ADDR
#define APB2PER_BASE_ADDR 		0x40010000U
#define AHB1PER_BASE_ADDR 		0x40020000U
#define AHB2PER_BASE_ADDR 		0x50000000U

//Buoc 3: GPIO - AHB1

#define GPIOA_BASE_ADDR 		(AHB1PER_BASE_ADDR + 0x0000) // AHB1 + OFFSET
#define GPIOB_BASE_ADDR 		(AHB1PER_BASE_ADDR + 0x0400)
#define GPIOC_BASE_ADDR 		(AHB1PER_BASE_ADDR + 0x0800)
#define GPIOD_BASE_ADDR 		(AHB1PER_BASE_ADDR + 0x0C00)
#define GPIOE_BASE_ADDR 		(AHB1PER_BASE_ADDR + 0x1000)
#define GPIOF_BASE_ADDR 		(AHB1PER_BASE_ADDR + 0x1400)
#define GPIOG_BASE_ADDR 		(AHB1PER_BASE_ADDR + 0x1800)
#define GPIOH_BASE_ADDR 		(AHB1PER_BASE_ADDR + 0x1C00)
#define GPIOI_BASE_ADDR 		(AHB1PER_BASE_ADDR + 0x2000)

#define RCC_BASE_ADDR 			(AHB1PER_BASE_ADDR + 0x3800)

// Buoc 4: APB1

#define  I2C1_BASE_ADDR 		(APB1PER_BASE_ADDR + 0x5400)
#define  I2C2_BASE_ADDR 		(APB1PER_BASE_ADDR + 0x5800)
#define  I2C3_BASE_ADDR 		(APB1PER_BASE_ADDR + 0x5C00)

#define  SPI2_BASE_ADDR 		(APB1PER_BASE_ADDR + 0x3800)
#define  SPI3_BASE_ADDR 		(APB1PER_BASE_ADDR + 0x3C00)

#define USART2_BASE_ADDR		(APB1PER_BASE_ADDR + 0x4400)
#define USART3_BASE_ADDR		(APB1PER_BASE_ADDR + 0x4800)
#define UART4_BASE_ADDR			(APB1PER_BASE_ADDR + 0x4C00)
#define UART5_BASE_ADDR			(APB1PER_BASE_ADDR + 0x5000)

// Buoc 5: APB2

#define EXTI_BASE_ADDR 			(APB2PER_BASE_ADDR + 0x3C00)
#define SYSCFG_BASE_ADDR  		(APB2PER_BASE_ADDR + 0x3800)
#define SPI1_BASE_ADDR 			(APB2PER_BASE_ADDR + 0x3000)
#define SPI4_BASE_ADDR 			(APB2PER_BASE_ADDR + 0x3400)
#define USART1_BASE_ADDR 		(APB2PER_BASE_ADDR + 0x1000)
#define USART6_BASE_ADDR 		(APB2PER_BASE_ADDR + 0x1400)

/*
 * ******************* Define peripheral ****************************
 */

#define GPIOA 		((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB 		((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC 		((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD 		((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOE 		((GPIO_RegDef_t*)GPIOE_BASE_ADDR)
#define GPIOF 		((GPIO_RegDef_t*)GPIOF_BASE_ADDR)
#define GPIOG 		((GPIO_RegDef_t*)GPIOG_BASE_ADDR)
#define GPIOH 		((GPIO_RegDef_t*)GPIOH_BASE_ADDR)
#define GPIOI 		((GPIO_RegDef_t*)GPIOI_BASE_ADDR)

#define SPI1		((SPI_RegDef_t*)SPI1_BASE_ADDR)
#define SPI2		((SPI_RegDef_t*)SPI2_BASE_ADDR)
#define SPI3		((SPI_RegDef_t*)SPI3_BASE_ADDR)
#define SPI4		((SPI_RegDef_t*)SPI4_BASE_ADDR)

#define I2C1		((I2C_RegDef_t*)I2C1_BASE_ADDR)
#define I2C2		((I2C_RegDef_t*)I2C2_BASE_ADDR)
#define I2C3		((I2C_RegDef_t*)I2C3_BASE_ADDR)

#define USART1    ((USART_RegDef_t*)USART1_BASE_ADDR)
#define USART2    ((USART_RegDef_t*)USART2_BASE_ADDR)
#define USART3    ((USART_RegDef_t*)USART3_BASE_ADDR)
#define UART4     ((USART_RegDef_t*)UART4_BASE_ADDR)
#define UART5     ((USART_RegDef_t*)UART5_BASE_ADDR)
#define USART6    ((USART_RegDef_t*)USART6_BASE_ADDR)

#define RCC 		((RCC_RegDef_t*)RCC_BASE_ADDR)

#define EXTI		((EXTI_RegDef_t*)EXTI_BASE_ADDR)

#define SYSCFG		((SYSCFG_RegDef_t*)SYSCFG_BASE_ADDR)

/*
 * ********************* Peripheral Register Definition**************
 * */
typedef struct {
__vo uint32_t MODER;		//  GPIO port mode    				offset: 0x00
__vo uint32_t OTYPER;		//  output type 							offset: 0x04
__vo uint32_t OSPEEDR;		//  output speed  							offset: 0x08
__vo uint32_t PUPDR;		//  pull-up/pull-down  						offset: 0x0C
__vo uint32_t IDR;			//  input data 		 						offset: 0x10
__vo uint32_t ODR;			//  output data 		 					offset: 0x14
__vo uint32_t BSRR;			//  bit set/reset 		 					offset: 0x18
__vo uint32_t LCKR;			//  configuration lock 						offset: 0x1C
__vo uint32_t AFR[2];		//  alternate function  			offset: 0x20

} GPIO_RegDef_t;

/*
 ****************************RCC Register Definition *****************
 **/

typedef struct {
__vo uint32_t CR; 			//clock control 							offset: 0x00
__vo uint32_t PLLCFGR; 		//configuration 							offset: 0x04
__vo uint32_t CFGR; 		//clock configuration 						offset: 0x08
__vo uint32_t CIR; 			//clock interrupt 							offset: 0x0C
__vo uint32_t AHB1RSTR; 		//AHB1 peripheral reset 				offset: 0x10
__vo uint32_t AHB2RSTR; 		//AHB2 peripheral reset 				offset: 0x14
__vo uint32_t AHB3RSTR; 		//AHB3 peripheral reset  				offset: 0x18

uint32_t RESERVED0; 		//Reserved				 					offset: 0x1C

__vo uint32_t APB1RSTR; 		//APB1 peripheral reset 				offset: 0x20
__vo uint32_t APB2RSTR; 		//APB2 peripheral reset 				offset: 0x24

uint32_t RESERVED1[2]; 		//Reserved				 					offset: 0x28,0x2C

__vo uint32_t AHB1ENR; 		// AHB1 peripheral clock enable				offset: 0x30
__vo uint32_t AHB2ENR; 		// AHB2 peripheral clock enable				offset: 0x34
__vo uint32_t AHB3ENR; 		// AHB3 peripheral clock enable				offset: 0x38

uint32_t RESERVED2; 		//											offset: 0x3C

__vo uint32_t APB1ENR; 		// APB1 peripheral clock enable 			offset: 0x40
__vo uint32_t APB2ENR; 		// APB2 peripheral clock enable 			offset: 0x44

uint32_t RESERVED3[2]; 		// 										offset: 0x48, 0x4C

__vo uint32_t AHB1LPENR; 		//AHB1 -- in low power mode				offset: 0x50
__vo uint32_t AHB2LPENR; 		//AHB2 -- in low power mode				offset: 0x54
__vo uint32_t AHB3LPENR; 		//AHB3 -- in low power mode				offset: 0x58

uint32_t RESERVED4; 		//											offset: 0x5C

__vo uint32_t APB1LPENR; 		//APB1 -- in low power mode	 			offset: 0x60
__vo uint32_t APB2LPENR; 		//APB2 -- in low power mode	 			offset: 0x64

uint32_t RESERVED5[2]; 		//											offset: 0x68, 0x6C

__vo uint32_t BDCR; 			//Backup domain control 				offset: 0x70
__vo uint32_t CSR; 				// clock control & status				offset: 0x74

uint32_t RESERVED6[2]; 				// 									offset: 0x78, 0x7C

__vo uint32_t SSCGR; 			// spread spectrum clock generation		offset: 0x80
__vo uint32_t PLLI2SCFGR; 		//PLLI2S configuration					offset: 0x84

} RCC_RegDef_t;

/*
 * ********************EXTI Register Structure Definition *************
 */

typedef struct {
__vo uint32_t IMR;	//		Interrupt mask				offset: 0x00
__vo uint32_t EMR;	//		Event mask					offset: 0x04
__vo uint32_t RTSR;	//		Rising trigger selection	offset: 0x08
__vo uint32_t FTSR;	//		Falling trigger selection	offset: 0x0C
__vo uint32_t SWIER;	//		Software interrupt event 	offset: 0x10
__vo uint32_t PR;	//		Pending					 	offset: 0x14

} EXTI_RegDef_t;

/*
 * ********************** SYSCFG Register Structure Definition ********************
 */

typedef struct {
__vo uint32_t MEMRMP;	// memory remap										offset: 0x00
__vo uint32_t PMC;		// peripheral mode configuration					offset: 0x04
__vo uint32_t EXTICR[4];		// external interrupt configuration register 	offset: 0x08,0x0C,0x10,0x14

__vo uint32_t CMPCR;	// Compensation cell control						offset: 0x20
} SYSCFG_RegDef_t;

typedef struct {
__vo uint32_t CR1; 		// SPI control					offset: 0x00
__vo uint32_t CR2; 		// SPI control 2				offset: 0x04
__vo uint32_t SR; 		// SPI status					offset: 0x08
__vo uint32_t DR; 		// SPI data						offset: 0x0C
__vo uint32_t CRCPR; 	// CRC polynomial 				offset: 0x10
__vo uint32_t RXCRCR; 	// RX CRC  						offset: 0x14
__vo uint32_t TXCRCR; 	// TX CRC  						offset: 0x18
__vo uint32_t I2SCFGR; 	// SPI_I2S configuration  		offset: 0x1C
__vo uint32_t I2SPR; 	// SPI_I2S prescaler   			offset: 0x20

} SPI_RegDef_t;

/*
 * ********************** I2C Register Structure Definition ********************
 */

typedef struct {
__vo uint32_t CR1;			// Control register 1  			offset: 0x00
__vo uint32_t CR2;			// Control register 2  			offset: 0x04
__vo uint32_t OAR1;			// Own address register 1 		 offset: 0x08
__vo uint32_t OAR2;			// Own address register 2 		 offset: 0x0C
__vo uint32_t DR;			// Data register 				 offset: 0x10
__vo uint32_t SR1;			// Status register 1 			 offset: 0x14
__vo uint32_t SR2;			// Status register 2 			 offset: 0x18
__vo uint32_t CCR;			// Clock control register		offset: 0x1C
__vo uint32_t TRISE;		// TRISE register				offset: 0x20

} I2C_RegDef_t;

typedef struct {
__vo uint32_t SR;        // Status register
__vo uint32_t DR;           // Data register           offset: 0x04
__vo uint32_t BRR;           // Baud rate          offset: 0x08
__vo uint32_t CR1;           // Control register 1          offset: 0x0C
__vo uint32_t CR2;           // Control register 2          offset: 0x10
__vo uint32_t CR3;           // Control register 3          offset: 0x14
__vo uint32_t GTPR;           // Guard time and prescaler          offset: 0x18

} USART_RegDef_t;

//-----------------------------------------------------------------------------
/********Clock Enable Macro for GPIOx - PAGE 145 ***************/
// GPIO -> AHB1 -> AHB1ENR
#define GPIOA_PCLK_EN() 			(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN() 			(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN() 			(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN() 			(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN() 			(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN() 			(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN() 			(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN() 			(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN() 			(RCC->AHB1ENR |= (1 << 8))

/******** I2C Enable Macro PCLK  *****************/

#define I2C1_PCLK_EN() 			(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() 			(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() 			(RCC->APB1ENR |= (1 << 23))

/******** SPI Enable Macro PCLK  *****************/

#define SPI1_PCLK_EN() 			(RCC->APB2ENR |= (1 << 12))

#define SPI2_PCLK_EN() 			(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() 			(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN() 			(RCC->APB1ENR |= (1 << 13))

/******** USARTx Enable Macro PCLK  *****************/

#define USART1_PCLK_EN() 			(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN() 			(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN() 			(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN() 			(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN() 			(RCC->APB1ENR |= (1 << 20))

#define USART6_PCLK_EN()			(RCC->APB2ENR |= (1 << 5))

/******** SYSCFG  Enable Macro PCLK  *****************/

#define SYSCFG_PCLK_EN() 			(RCC->APB2ENR |= (1 << 14))

//------------------------------------------------------------------------

/********Clock Disable Macro for GPIOx  *****************/
#define GPIOA_PCLK_DI() 			(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI() 			(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI() 			(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI() 			(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI() 			(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI() 			(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI() 			(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI() 			(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI() 			(RCC->AHB1ENR &= ~(1 << 8))

/******** I2C Disable Macro PCLK  *****************/

#define I2C1_PCLK_DI() 			(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI() 			(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI() 			(RCC->APB1ENR &= ~(1 << 23))

/******** SPI Disable Macro PCLK  *****************/

#define SPI1_PCLK_DI() 			(RCC->APB2ENR &= ~(1 << 12))

#define SPI2_PCLK_DI() 			(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI() 			(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI() 			(RCC->APB1ENR &= ~(1 << 13))

/******** USARTx Disable Macro PCLK  *****************/

#define USART2_PCLK_DI() 			(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI() 			(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI() 			(RCC->APB1ENR &= ~(1 << 20))

#define USART1_PCLK_DI() 			(RCC->APB2ENR &= ~(1 << 4))
#define USART6_PCLK_DI() 			(RCC->APB2ENR &= ~(1 << 5))

/******** SYSCFG  Disable Macro PCLK  *****************/

#define SYSCFG_PCLK_DI() 			(RCC->APB2ENR &= ~(1 << 14))

/*
 * Reset GPIO peripheral - while(0) execute only one time
 */
#define GPIOA_REG_RESET()		do{RCC->AHB1RSTR |= (1 << 0); RCC-> AHB1RSTR &= ~(1 << 0);}while(0)
#define GPIOB_REG_RESET()		do{RCC->AHB1RSTR |= (1 << 1); RCC-> AHB1RSTR &= ~(1 << 1);}while(0)
#define GPIOC_REG_RESET()		do{RCC->AHB1RSTR |= (1 << 2); RCC-> AHB1RSTR &= ~(1 << 2);}while(0)
#define GPIOD_REG_RESET()		do{RCC->AHB1RSTR |= (1 << 3); RCC-> AHB1RSTR &= ~(1 << 3);}while(0)
#define GPIOE_REG_RESET()		do{RCC->AHB1RSTR |= (1 << 4); RCC-> AHB1RSTR &= ~(1 << 4);}while(0)
#define GPIOF_REG_RESET()		do{RCC->AHB1RSTR |= (1 << 5); RCC-> AHB1RSTR &= ~(1 << 5);}while(0)
#define GPIOG_REG_RESET()		do{RCC->AHB1RSTR |= (1 << 6); RCC-> AHB1RSTR &= ~(1 << 6);}while(0)
#define GPIOH_REG_RESET()		do{RCC->AHB1RSTR |= (1 << 7); RCC-> AHB1RSTR &= ~(1 << 7);}while(0)
#define GPIOI_REG_RESET()		do{RCC->AHB1RSTR |= (1 << 8); RCC-> AHB1RSTR &= ~(1 << 8);}while(0)

/*
 * Reset SPI peripheral - while(0) execute only one time
 */
#define SPI1_REG_RESET()		do{RCC->APB2RSTR |= (1 << 6); RCC->APB2RSTR &= ~(1 << 6);}while(0);
#define SPI2_REG_RESET()		do{RCC->APB1RSTR |= (1 << 13); RCC->APB1RSTR &= ~(1 << 13);}while(0);
#define SPI3_REG_RESET()		do{RCC->APB1RSTR |= (1 << 14); RCC->APB1RSTR &= ~(1 << 14);}while(0);
#define SPI4_REG_RESET()		do{RCC->APB2RSTR |= (1 << 7); RCC->APB2RSTR &= ~(1 << 7);}while(0);

/*
 * Reset I2C peripheral - while(0) execute only one time
 */
#define I2C1_REG_RESET()		do{RCC->APB1RSTR |= (1 << 20); RCC->APB1RSTR &= ~(1 << 20);}while(0);
#define I2C2_REG_RESET()		do{RCC->APB1RSTR |= (1 << 21); RCC->APB1RSTR &= ~(1 << 21);}while(0);
#define I2C3_REG_RESET()		do{RCC->APB1RSTR |= (1 << 22); RCC->APB1RSTR &= ~(1 << 22);}while(0);

/*
 * return port code for given GPIOx baseaddr
 */

#define GPIO_BASEADDR_TO_CODE(x)	   ((x = GPIOA)? 0:\
										(x = GPIOB)? 1:\
										(x = GPIOD)? 2:\
										(x = GPIOE)? 3:\
										(x = GPIOF)? 4:\
										(x = GPIOG)? 5:\
										(x = GPIOH)? 7:\
										(x = GPIOI)? 7:0 )

/*
 * IRQ (Interrupt Request) Number STM32F407
 */

#define 	IRQ_NUM_EXTI0			6
#define 	IRQ_NUM_EXTI1			7
#define 	IRQ_NUM_EXTI2			8
#define 	IRQ_NUM_EXTI3			9
#define 	IRQ_NUM_EXTI4			10
#define 	IRQ_NUM_EXTI9_5			23
#define 	IRQ_NUM_EXTI15_10		40

/*
 * IRQ SPI
 */
#define 	IRQ_NUM_SPI1				35
#define 	IRQ_NUM_SPI2				36
#define 	IRQ_NUM_SPI3				51
#define 	IRQ_NUM_SPI4				84

/*
 * IRQ I2C
 */
#define 	IRQ_NUM_I2C1_EV			31		// I2C event interrupt
#define 	IRQ_NUM_I2C1_ER			32		// I2C error interrupt

/*
 *  NVIC Priority
 */

#define 	NVIC_IRQ_PRI0			0
#define 	NVIC_IRQ_PRI1			1
#define 	NVIC_IRQ_PRI2			2
#define 	NVIC_IRQ_PRI3			3
#define 	NVIC_IRQ_PRI4			4
#define 	NVIC_IRQ_PRI5			5
#define 	NVIC_IRQ_PRI6			6
#define 	NVIC_IRQ_PRI7			8
#define 	NVIC_IRQ_PRI8			7
#define 	NVIC_IRQ_PRI9			9
#define 	NVIC_IRQ_PRI10			10
#define 	NVIC_IRQ_PRI11			11
#define 	NVIC_IRQ_PRI12			12
#define 	NVIC_IRQ_PRI13			13
#define 	NVIC_IRQ_PRI14			14
#define 	NVIC_IRQ_PRI15			15

//------------------------------ Generic macros ---------------------------------------

#define ENABLE						1
#define DISABLE						0
#define SET							ENABLE
#define RESET						DISABLE
#define	GPIO_PIN_SET				SET
#define	GPIO_PIN_RESET				RESET
#define FLAG_SET					SET
#define FLAG_RESET					RESET

/*
 *  Bit pos definitions of SPI peripheral
 * ************************************************************
 */

// Bit position def SPI_CR1
#define SPI_CR1_CPHA			0
#define SPI_CR1_CPOL			1
#define SPI_CR1_MSTR			2
#define SPI_CR1_BR				3
#define SPI_CR1_SPE				6
#define SPI_CR1_LSBFIRST		7
#define SPI_CR1_SSI				8
#define SPI_CR1_SSM				9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRC_NEXT		12
#define SPI_CR1_CRC_EN			13
#define SPI_CR1_BIDI_EN			14
#define SPI_CR1_BIDI_MODE		15

// Bit pos def SPI_CR2
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

// Bit pos def SPI_SR
#define SPI_SR_RXNE 				0		//Receive buffer empty
#define SPI_SR_TXE 					1		//Transmit buffer empty
#define SPI_SR_CHSIDE 				2
#define SPI_SR_UDR 					3
#define SPI_SR_CRCERR 				4
#define SPI_SR_MODF 				5
#define SPI_SR_OVR 					6
#define SPI_SR_BSY 					7
#define SPI_SR_FRE 					8

/*
 *  Bit pos definitions of I2C peripheral
 * ************************************************************
 */

// Bit pos def I2C_CR1
#define I2C_CR1_PE							0
#define I2C_CR1_NOSTRETCH					7
#define I2C_CR1_START						8
#define I2C_CR1_STOP						9
#define I2C_CR1_ACK							10
#define I2C_CR1_SWRST						15

// Bit pos def I2C_CR2
#define I2C_CR2_FREQ					0
#define I2C_CR2_ITERREN					8		// Error interrupt enable
#define I2C_CR2_ITEVTEN					9		// Event interrupt enable
#define I2C_CR2_ITBUFEN					10		// Buffer interrupt enable

// Bit pos def I2C_SR1
#define I2C_SR1_SB							0
#define I2C_SR1_ADDR						1
#define I2C_SR1_BTF							2		// Byte transfer finished
#define I2C_SR1_ADD10						3	// 10-bit header sent (Master mode)
#define I2C_SR1_STOPF						4	// Stop detection (slave mode)
#define I2C_SR1_RXNE						6	// Data register not empty (receivers)
#define I2C_SR1_TXE							7	// Data register empty (transmitters)
#define I2C_SR1_BERR						8	// Bus error
#define I2C_SR1_ARLO						9	// Arbitration lost (master mode)
#define I2C_SR1_AF							10	// Acknowledge failure
#define I2C_SR1_OVR							11	// Overrun/Underrun
#define I2C_SR1_TIMEOUT					14	//  Timeout or Tlow error

// Bit pos def I2C_SR2
#define I2C_SR2_MSL						0 	//  Master/slave
#define I2C_SR2_BUSY					1 	//  Bus busy
#define I2C_SR2_TRA						2 	//  Transmitter/receiver
#define I2C_SR2_GENCALL					4 	//  General call address (Slave mode)
#define I2C_SR2_DUALF					7 	//  Dual flag (Slave mode)

// Bit pos def I2C_CCR
#define I2C_CCR_CCR 					0 // Clock control register in Fm/Sm mode`
#define I2C_CCR_DUTY 					14 //  Fm mode duty cycle
#define I2C_CCR_FS 					15 //  I2C master mode selection 0: Sm -- 1: Fm

/*
 *  Bit pos definitions of USART peripheral
 * ************************************************************
 */

#define USART_CR1_SBK           0
#define USART_CR1_RWU           1
#define USART_CR1_RE            2       //  Receiver enable
#define USART_CR1_TE            3       // Transmitter enable
#define USART_CR1_IDLEIE        4
#define USART_CR1_RXNEIE         5
#define USART_CR1_TCIE          6
#define USART_CR1_TXEIE         7
#define USART_CR1_PEIE          8
#define USART_CR1_PS             9    // Parity selection: 0: Even parity, 1: Odd parity
#define USART_CR1_PCE           10    // Parity control enable
#define USART_CR1_WAKE          11
#define USART_CR1_M             12    // Word length: 0: 1 Start bit, 8 Data bits
                                       //              1: 1 Start bit, 9 Data bits,
#define USART_CR1_UE            13      // USART enable
#define USART_CR1_OVER8         15

/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD           0
#define USART_CR2_LBDL          5
#define USART_CR2_LBDIE         6
#define USART_CR2_LBCL          8
#define USART_CR2_CPHA          9
#define USART_CR2_CPOL          10
#define USART_CR2_STOP          12    // STOP bits
#define USART_CR2_LINEN           14

/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE           0
#define USART_CR3_IREN          1
#define USART_CR3_IRLP          2
#define USART_CR3_HDSEL          3
#define USART_CR3_NACK          4
#define USART_CR3_SCEN          5
#define USART_CR3_DMAR          6
#define USART_CR3_DMAT          7
#define USART_CR3_RTSE          8
#define USART_CR3_CTSE          9     // CTS enable, data is only transmitted when the CTS input is asserted
#define USART_CR3_CTSIE           10
#define USART_CR3_ONEBIT          11

/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE               0
#define USART_SR_FE               1
#define USART_SR_NE               2
#define USART_SR_ORE              3
#define USART_SR_IDLE             4
#define USART_SR_RXNE             5
#define USART_SR_TC               6
#define USART_SR_TXE              7
#define USART_SR_LBD              8
#define USART_SR_CTS              9

#include "stm32f407_gpio_driver.h"
#include "stm32f407_spi_driver.h"
#include "stm32f407_i2c_driver.h"
#include "stm32f407_uart_driver.h"
#endif /* INC_STM32F4XX_H_ */

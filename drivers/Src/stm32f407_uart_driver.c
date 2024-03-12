/*
 * stm32f407_uart_driver.c
 *
 *  Created on: Mar 11, 2024
 *      Author: ThanhPC
 */

#include "stm32f407_uart_driver.h"

void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
   if(EnOrDi == ENABLE)
   {
      pUSARTx->CR1 |= 1 << 13;
   }
   else
   {
      pUSARTx->CR1 &= ~(1 << 13);
   }
}

void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi)
{
   if(EnorDi == ENABLE)
   {
      if(pUSARTx == USART1)
      {
         USART1_PCLK_EN();
      }
      else if(pUSARTx == USART2)
      {
         USART2_PCLK_EN();
      }
      else if(pUSARTx == USART3)
      {
         USART3_PCLK_EN();
      }
      else if(pUSARTx == UART4)
      {
         UART4_PCLK_EN();
      }
   }
   else
   {

      if(pUSARTx == USART1)
      {
         USART1_PCLK_DI();
      }
      else if(pUSARTx == USART2)
      {
         USART2_PCLK_DI();
      }
      else if(pUSARTx == USART3)
      {
         USART3_PCLK_DI();
      }
      else if(pUSARTx == UART4)
      {
         UART4_PCLK_DI();
      }
   }
}

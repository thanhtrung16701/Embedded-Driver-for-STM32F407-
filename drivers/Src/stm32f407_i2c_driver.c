/*
 * stm32f407_i2c_driver.c
 *
 *  Created on: Jan 25, 2024
 *      Author: ThanhPC
 */
#include <stm32f407_i2c_driver.h>

uint16_t AHB_Prescaler[8] = { 2, 4, 8, 16, 64, 128, 256, 512 };	//RCC->CFGR-> HPRE
uint8_t APB1_Prescaler[4] = { 2, 4, 8, 16 };					//RCC->CFGR-> PPRE1
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx); // En Start bit in CR1
static void I2C_ExecuteADDRPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteADDRPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);

static void I2C_MasterHandleRXNE_IT(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXE_IT(I2C_Handle_t *pI2CHandle);

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
   pI2Cx->CR1 |= 1 << I2C_CR1_START;
}

static void I2C_ExecuteADDRPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
   // send 7 bits address with 1 read or write bit
   SlaveAddr <<= 1;	// make space for read or write bit
   /*
    * clear the zero bit -> lsb is R&W must be set to 0 for WRITE
    * SlaveAddr = Slave addr + bit R/nW
    */
   SlaveAddr &= ~1;

   pI2Cx->DR = SlaveAddr; // put it to DR
}

static void I2C_ExecuteADDRPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
   // send 7 bits address with 1 read or write bit
   SlaveAddr <<= 1;	// make space for read or write bit
   /*
    * clear the zero bit -> lsb is R&W must be set to 1 for READ
    * SlaveAddr = Slave addr + bit R/nW
    */
   SlaveAddr |= 1;

   pI2Cx->DR = SlaveAddr; // put it to DR
}
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
   uint32_t dummyRead;
   // check mode master or slave
   if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
   {
      // master mode
      if(pI2CHandle->TxRxState == I2C_BUSY_RX)
      {
         if(pI2CHandle->RxSize == 1)
         {
            // disable the ack
            I2C_MangageACKing(pI2CHandle->pI2Cx, DISABLE);
            // clear ADDR flag -> read SR1, SR2
            dummyRead = pI2CHandle->pI2Cx->SR1;
            dummyRead = pI2CHandle->pI2Cx->SR2;
            (void) dummyRead;	// avoid unused variable
         }
      }
      else
      {
         // clear ADDR flag -> read SR1, SR2
         dummyRead = pI2CHandle->pI2Cx->SR1;
         dummyRead = pI2CHandle->pI2Cx->SR2;
         (void) dummyRead;	// avoid unused variable
      }
   }
   else
   {
      // slave mode
      // clear ADDR flag -> read SR1, SR2
      dummyRead = pI2CHandle->pI2Cx->SR1;
      dummyRead = pI2CHandle->pI2Cx->SR2;
      (void) dummyRead;	// avoid unused variable
   }
}

 void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
   pI2Cx->CR1 |= 1 << I2C_CR1_STOP;
}

/*
 *  Peripheral Clock setup
 */

void I2C_PeriClockCtrl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
   if(EnOrDi == ENABLE)
   {
      if(pI2Cx == I2C1)
         I2C1_PCLK_EN();
      else if(pI2Cx == I2C2)
         I2C2_PCLK_EN();
      else if(pI2Cx == I2C3)
         I2C3_PCLK_EN();
   }
   else
   {
      if(pI2Cx == I2C1)
         I2C1_PCLK_DI();
      else if(pI2Cx == I2C2)
         I2C2_PCLK_DI();
      else if(pI2Cx == I2C3)
         I2C3_PCLK_DI();
   }
}

/*
 * Init and De-init
 */
uint32_t I2C_GetPLLOutputClk()
{
   uint32_t pllclk = 1;
   return pllclk;
}

uint32_t RCC_GetPCLK1Value()	// I2C on APB1-> PCLK1 -> f MCU
{
   uint32_t pclk1, SystemClk;
   uint8_t clksrc; // System clock switch status(bit 2,3) in RCC_CFGR
   uint8_t temp, ahb_pre, abp1_pre;

   clksrc = (RCC->CFGR >> 2) & 0X3;
   if(clksrc == 0)
   {
      SystemClk = 16000000;	// HSI = 16Mhz
   }
   else if(clksrc == 1)
   {
      SystemClk = 8000000; // HSE = 8Mhz;
   }
   else if(clksrc == 2)
   {
      SystemClk = I2C_GetPLLOutputClk();
   }

   temp = (RCC->CFGR >> 4) & 0xF;			// HPRE bit 4:7
   if(temp < 8)
   {		// 0xxx: system clock not divided -> 0 to 7
      ahb_pre = 1;
   }
   else
   {
      ahb_pre = AHB_Prescaler[temp - 8];	// 1xxx more than or equal 8
   }

   temp = (RCC->CFGR >> 10) & 0x7;			// PPRE1 bit 12:10
   if(temp < 4)
   {
      abp1_pre = 1;		// 0xxx not divided -> 0 to 3
   }
   else
   {
      abp1_pre = APB1_Prescaler[temp - 4];
   }
   pclk1 = (SystemClk / ahb_pre) / abp1_pre;
   return pclk1;
}

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
   /*
    uint32_t I2C_SCLSpeed;
    uint8_t  I2C_DeviceAddr;	// 7 bit wide -> uint8_t
    uint8_t  I2C_ACKControl;
    uint16_t I2C_FMDutyCycle;
    */
   uint32_t temp = 0;

   //enable the clock for the i2cx peripheral
   I2C_PeriClockCtrl(pI2CHandle->pI2Cx, ENABLE);

   // 1. set ACK in CR1
   temp |= pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;
   pI2CHandle->pI2Cx->CR1 = temp;

   // 2. Config the FREQ of CR2 (I2C_SCLSpeed)
   temp = 0;
   temp |= RCC_GetPCLK1Value() / 1000000U; 	// return 16 (Mhz)
   pI2CHandle->pI2Cx->CR2 = temp & 0x3F;		// freg 6 bit

   // 3. Device own address
   temp |= pI2CHandle->I2C_Config.I2C_DeviceAddr << 1;
   temp |= 1 << 14;				// bit 14 always be kept at 1
   pI2CHandle->pI2Cx->OAR1 = temp;

   //************************
   // 4. Config Clock Control (CCR) -> using CCR 0:11 bit
   uint16_t ccr_value = 0;		// CCR is 12 bit
   temp = 0;
   if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
   {
      /*
       *  Standard mode: T_high = T_low= ccr_value*T_pclk, T_Scl = T_h + T_l, T= 1/f
       * -> ccr_value = f_pclk / (2 * f_Scl)
       */
      ccr_value = (RCC_GetPCLK1Value()) / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
      temp |= ccr_value & (0xFFF);	// 12 bit CCR
   }
   else
   {
      // Fast mode
      if(I2C_CCR_DUTY == I2C_FM_DUTY_2)
      {
         /*
          * T_h = 2 T_l -> ccr_value = f / (3 f_Scl)
          */
         ccr_value = (RCC_GetPCLK1Value()) / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
      }
      else
      {
         /*
          * T_h = 16 ccr_value*T_pclk , T_l = 9 ccr_value*T_pclk
          * -> ccr_value = f / (25 f_Scl)
          */
         ccr_value = (RCC_GetPCLK1Value()) / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed);
      }
      temp |= ccr_value & 0xFFF; 	// 12 bit CCR
   }
   pI2CHandle->pI2Cx->CCR = temp;

   // 5. Config Trise
   /*  trise = T_Scl/ T_PCLK1 +1 = f_pclk1 * T_Scl +1
    * 		= f_pclk1/ f_scl +1
    *   SM, T_SCL = 1000 nS= 1 uS -> f = 1 Mhz= 1000000
    *   FM, T_SCL = 300  nS = 300 / 10^9
    */
   if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
   {
      // Standard mode
      temp = (RCC_GetPCLK1Value() / 1000000U) + 1;
   }
   else
   {
      // Fast mode
      temp = ((RCC_GetPCLK1Value() * 300) / (10 ^ 9U)) + 1;
   }
   pI2CHandle->pI2Cx->TRISE = (temp & 0x3F); // 6 bit

}

void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
   if(pI2Cx == I2C1)
   {
      I2C1_REG_RESET();
   }
   else if(pI2Cx == I2C2)
   {
      I2C2_REG_RESET();
   }
   else if(pI2Cx == I2C3)
   {
      I2C3_REG_RESET();
   }
}
/*
 * Data send and receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
   // 1. Generate Start condition
   I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

   // 2. Confirm Start Gen completed by check SB flag in SR1
   // Until SB is cleared, SCL'll be pulled to LOW
   while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB))); // wait to SB = 1 then

   // 3. Send data of slave with r&w bit set to WRITE (= 0) (8 bit)
   I2C_ExecuteADDRPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

   //4. Confirm that address phase is completed by checking the ADDR flag in SR
   while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR))); // wait to ADDR = 1 then

   //5. clear the ADDR flag according to its software sequence
   //   Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
   I2C_ClearADDRFlag(pI2CHandle);

   // 6. Send data till Len equals 0
   while(Len > 0)
   {
      while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE))); // wait TXE is set
      pI2CHandle->pI2Cx->DR = *pTXBuffer;		// send data by using Tx buffer to get data
      pTXBuffer++;
      Len--;
   }

   /*
    7. when Len = 0  wait for TXE=1 and BTF=1 before generating  STOP
    Note: TXE=1 , BTF=1 -> both SR and DR are empty and next transmission begin
    when BTF=1 ,SCL will be pulled to LOW
    */
   while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE))); // wait TXE is set
   while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF)));

   //8. Generate STOP condition and master need not to wait for the completion of stop condition.
   //   Note: generating STOP, automatically clears the BTF
   if(Sr == I2C_DISABLE_SR)
      I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRXBuffer, uint8_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
   // 1. Gen START condition
   I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
   // 2. Confirm START is completed by checking SB flag in SR1
   //	  till SB is cleared, SCL -> LOW
   while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

   // 3. Send the addr of slave with r/nw bit set to READ (=1)
   I2C_ExecuteADDRPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

   // 4. Wait until addr phase is completed by checking ADDR flag in SR1
   while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

   // Read 1 byte from slave

   if(Len == 1)
   {
      // disable ACK
      I2C_MangageACKing(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

      // Gen STOP
      if(Sr == I2C_DISABLE_SR)
         I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

      // clear ADDR flag
      I2C_ClearADDRFlag(pI2CHandle);

      // Wait until RXNE becomes 1
      while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

      // Read data in to buffer
      *pRXBuffer = pI2CHandle->pI2Cx->DR;
   }

   if(Len > 1)
   {
      // clear ADDR flag
      I2C_ClearADDRFlag(pI2CHandle);

      // Read data till Len = 0
      for(uint32_t i = Len; i > 0; i--)
      {
         // wait till RXNE = 1
         while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

         if(i == 2) // 2 bytes (last) remains
         {
            // clear ACK
            I2C_MangageACKing(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

            // Gen STOP
            if(Sr == I2C_DISABLE_SR)
               I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
         }
         // read the data from data register in to buffer
         *pRXBuffer = pI2CHandle->pI2Cx->DR;

         // increment the buffer addr
         pRXBuffer++;
      }

      //re-enable ACKing
      I2C_MangageACKing(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
   }

}

/*
 * IRQ Config and ISR handling
 */

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDi)
{
   if(EnOrDi == ENABLE)
   {
      if(IRQNumber < 32)
      {
         *NVIC_ICER0 |= 1 << IRQNumber;
      }
      else if(IRQNumber >= 32 && IRQNumber < 64)
      {
         *NVIC_ICER1 |= 1 << (IRQNumber % 32);
      }
      else if(IRQNumber >= 64 && IRQNumber < 96)
      {
         *NVIC_ICER2 |= 1 << (IRQNumber % 64);
      }
   }
   else
   {
      if(IRQNumber < 32)
      {
         *NVIC_ICER0 &= ~(1 << IRQNumber);
      }
      else if(IRQNumber >= 32 && IRQNumber < 64)
      {
         *NVIC_ICER1 &= ~(1 << (IRQNumber % 32));
      }
      else if(IRQNumber >= 64 && IRQNumber < 96)
      {
         *NVIC_ICER2 &= ~(1 << (IRQNumber % 64));
      }
   }
}

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
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
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
   if(pI2Cx->SR1 & FlagName)
   {
      return FLAG_SET;
   }
   return FLAG_RESET;
}

void I2C_PeriControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi) // PE En or Di
{
   if(EnOrDi == ENABLE) // bit 0 CR1 -> i2c peripheral enable(PE)
   {
      pI2Cx->CR1 |= 1 << I2C_CR1_PE;
   }
   else
   {
      pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
   }
}

void I2C_MangageACKing(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
   if(EnOrDi == I2C_ACK_ENABLE)
   {
      pI2Cx->CR1 |= 1 << I2C_CR1_ACK;
   }
   else
   {
      pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
   }
}

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTXBuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
   uint8_t busystate = pI2CHandle->TxRxState;

   if(busystate != I2C_BUSY_TX && busystate != I2C_BUSY_RX)
   {
      pI2CHandle->pTXBuffer = pTXBuffer;
      pI2CHandle->TxLen = Len;
      pI2CHandle->TxRxState = I2C_BUSY_TX;
      pI2CHandle->DevAddr = SlaveAddr;
      pI2CHandle->Sr = Sr;

      // Gen START Condition
      I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

      // Enable ITBUFEN Control bit (Buffer interrupt enable)
      pI2CHandle->pI2Cx->CR2 |= 1 << I2C_CR2_ITBUFEN;

      //Implement the code to enable ITEVFEN Control Bit
      pI2CHandle->pI2Cx->CR2 |= 1 << I2C_CR2_ITEVTEN;

      //Implement the code to enable ITERREN Control Bit
      pI2CHandle->pI2Cx->CR2 |= 1 << I2C_CR2_ITERREN;
   }
   return busystate;
}

uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer,uint8_t Len,uint8_t SlaveAddr, uint8_t Sr)
{
   uint8_t busystate = pI2CHandle->TxRxState;
   if((busystate != I2C_BUSY_TX) && (busystate != I2C_BUSY_RX))
   {
      pI2CHandle->pRXBuffer = pRxBuffer;
      pI2CHandle->RxLen = Len;
      pI2CHandle->TxRxState = I2C_BUSY_RX;
      pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
      pI2CHandle->DevAddr = SlaveAddr;
      pI2CHandle->Sr = Sr;

      //Implement code to Generate START Condition
      I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

      //Implement the code to enable ITBUFEN Control Bit
      pI2CHandle->pI2Cx->CR2 |= 1 << I2C_CR2_ITBUFEN;

      //Implement the code to enable ITEVFEN Control Bit
      pI2CHandle->pI2Cx->CR2 |= 1 << I2C_CR2_ITEVTEN;

      //Implement the code to enable ITERREN Control Bit
      pI2CHandle->pI2Cx->CR2 |= 1 << I2C_CR2_ITERREN;
   }
   return busystate;
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
   // Handle IT for master and slave mode
   uint32_t temp1, temp2, temp3;
   temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
   temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

   temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);	// check SB flag is set or not(only in Master)
   //	1. Handle IT generated by SB event
   // 		Master: addr is sent
   //		Slave: addr match with own addr
   if(temp1 && temp3)
   {
      // IT gen by SB event (NOT in SM cuz SB always = 0)
      // execute the addr phase
      if(pI2CHandle->TxRxState == I2C_BUSY_TX)
      {
         I2C_ExecuteADDRPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
      }
      else if(pI2CHandle->TxRxState == I2C_BUSY_RX)
      {
         I2C_ExecuteADDRPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
      }
   }

   temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
   // 2. Handle IT gen by ADDR event
   if(temp1 && temp3)
   {
      // ADDR flag is set
      I2C_ClearADDRFlag(pI2CHandle);
   }

   temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
   // 3. Handle IT gen by BTF(Byte Transfer Finished) event
   if(temp1 && temp3)
   {
      // BTF flag is set
      if(pI2CHandle->TxRxState == I2C_BUSY_TX)
      {
         // make sure TXE is set
         if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE))
         {
            // BTF, TXE = 1 -> SR and DR are empty
            if(pI2CHandle->TxLen == 0)
            {
               // 1. gen STOP condition
               if(pI2CHandle->Sr == I2C_DISABLE_SR)
                  I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
               // 2. reset all elements of handle structure
               I2C_CloseSendData(pI2CHandle);
               // 3. notify about tx complete
               I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
            }
         }

      }
      else if(pI2CHandle->TxRxState == I2C_BUSY_RX)
      {
         ;
      }
   }

   temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
   // 4. Handle IT gen by STOPF (Stop detection only slave mode)
   if(temp1 && temp3)
   {
      // STOPF flag is set
      // Clear STOPF : Read SR1 then Write to CR1 (Read SR1 is done by temp3)
      pI2CHandle->pI2Cx->CR1 |= 0x0000;
      // Notify STOP detected
      I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
   }

   // 5. Handle IT gen by TXE event
   temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
   if(temp1 && temp2 && temp3)
   {
      // check for Master or slave
      if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
      {
         // TXE flag is set -> transmit data
         I2C_MasterHandleTXE_IT(pI2CHandle);

      }
   }

   // 6. Handle IT gen by RXNE event
   temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
   if(temp1 && temp2 && temp3)
   {
      if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)) // master
      {
         // RXNE flag is set -> receive data
         if(pI2CHandle->TxRxState == I2C_BUSY_RX)
         {
            I2C_MasterHandleRXNE_IT(pI2CHandle);
         }
      }
   }
}

static void I2C_MasterHandleTXE_IT(I2C_Handle_t *pI2CHandle)
{
   if(pI2CHandle->TxLen > 0)
   {
      // 1. Load data into DR
      pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTXBuffer);
      // 2. decrement TxLen
      pI2CHandle->TxLen--;
      // 3. Increment TXbuffer addr
      pI2CHandle->pTXBuffer++;
   }
}

static void I2C_MasterHandleRXNE_IT(I2C_Handle_t *pI2CHandle)
{
   if(pI2CHandle->RxSize == 1)
   {
      // read 1 btye into RXBuffer from DR
      *pI2CHandle->pRXBuffer = pI2CHandle->pI2Cx->DR;
      // reduce length
      pI2CHandle->RxLen--;
   }

   if(pI2CHandle->RxSize > 1)
   {
      if(pI2CHandle->RxLen == 2)
      {
         // clear the Ack
         I2C_MangageACKing(pI2CHandle->pI2Cx, DISABLE);
      }
      // read DR
      *pI2CHandle->pRXBuffer = pI2CHandle->pI2Cx->DR;
      pI2CHandle->pRXBuffer++;
      pI2CHandle->RxLen--;
   }

   if(pI2CHandle->RxLen == 0)
   {
      // close I2C RX and notify
      // 1. gen STOP
      if(pI2CHandle->Sr == I2C_DISABLE_SR)
      {
         I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
      }
      // 2. Close I2C RX
      I2C_CloseReceiveData(pI2CHandle);

      // 3. Notify
      I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
   }
}
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
   uint32_t temp1, temp2;

   //Know the status of  ITERREN control bit in the CR2
   temp2 = (pI2CHandle->pI2Cx->CR2) & (1 << I2C_CR2_ITERREN);

   /***********************Check for Bus error************************************/
   temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_BERR);
   if(temp1 && temp2)
   {
      //This is Bus error
      //Implement the code to clear the buss error flag
      pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);

      //Implement the code to notify the application about the error
      I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
   }

   /***********************Check for arbitration lost error************************************/
   temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_ARLO);
   if(temp1 && temp2)
   {
      //This is arbitration lost error

      //Implement the code to clear the arbitration lost error flag
      pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

      //Implement the code to notify the application about the error
      I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
   }

   /***********************Check for ACK failure  error************************************/

   temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_AF);
   if(temp1 && temp2)
   {
      //This is ACK failure error

      //Implement the code to clear the ACK failure error flag
      pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

      //Implement the code to notify the application about the error
      I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
   }

   /***********************Check for Overrun/underrun error************************************/
   temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_OVR);
   if(temp1 && temp2)
   {
      //This is Overrun/underrun

      //Implement the code to clear the Overrun/underrun error flag
      pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

      //Implement the code to notify the application about the error
      I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
   }

   /***********************Check for Time out error************************************/
   temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_TIMEOUT);
   if(temp1 && temp2)
   {
      //This is Time out error

      //Implement the code to clear the Time out error flag
      pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

      //Implement the code to notify the application about the error
      I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
   }
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle) // stop interrupts
{
   // clear all the IT enable bit

   // prevent generating TXE or RXNE interrupt
   pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
   // prevent different EV
   pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

   pI2CHandle->TxRxState = I2C_READY;
   pI2CHandle->pRXBuffer = NULL;
   pI2CHandle->RxLen = 0;
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
   // clear all the IT enable bit

   // prevent generating TXE or RXNE interrupt
   pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
   // prevent different EV
   pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

   pI2CHandle->TxRxState = I2C_READY;
   pI2CHandle->pRXBuffer = NULL;
   pI2CHandle->RxLen = 0;
   pI2CHandle->RxSize = 0;
   // during reception, disable ACKing -> when close must enable
   if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
   {
      I2C_MangageACKing(pI2CHandle->pI2Cx, ENABLE);
   }
}

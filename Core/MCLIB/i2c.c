/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    i2c.c
  * @brief   This file provides code for the configuration
  *          of the I2C instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

/* USER CODE BEGIN 0 */
//#include "i2c_ex.h"
/* USER CODE END 0 */


void I2C1_Start(void)
{
  LL_I2C_Enable(I2C1);
}

void I2C1_ClearCR(void)
{
  (I2C1->CR2 &= (uint32_t)~((uint32_t)(I2C_CR2_SADD | I2C_CR2_HEAD10R | I2C_CR2_NBYTES | I2C_CR2_RELOAD | I2C_CR2_RD_WRN)));
}

uint8_t I2C1_TransmitData(uint8_t device_id ,uint8_t *pdata, uint8_t size, uint16_t timeout)
{
  // busyフラグをチェック
  while(LL_I2C_IsActiveFlag_BUSY(I2C1) == SET) {
    if(LL_SYSTICK_IsActiveCounterFlag())
    {
        if (timeout-- == 0)
        {
            I2C1_ClearCR();
            return 1;
        }
    }       
  }

  LL_I2C_HandleTransfer(I2C1, device_id, LL_I2C_ADDRSLAVE_7BIT, size, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);

  for(uint8_t i = 0; i < size; i++){
    while(LL_I2C_IsActiveFlag_TXE(I2C1)== RESET) {
        if(LL_SYSTICK_IsActiveCounterFlag())
        {
            if (timeout-- == 0)
            {
                I2C1_ClearCR();
                return 1;
            }
        }         
    }
    LL_I2C_TransmitData8(I2C1, *pdata++);
  }

  while(LL_I2C_IsActiveFlag_STOP(I2C1)==RESET) {
    if(LL_SYSTICK_IsActiveCounterFlag())
    {
        if (timeout-- == 0)
        {
            I2C1_ClearCR();
            return 1;
        }
    }       
  }
  LL_I2C_ClearFlag_STOP(I2C1);
  
  // 設定をリセットする
  I2C1_ClearCR();

  return 0;
}

uint8_t I2C1_TransmitData_RepeatedStart(uint8_t device_id ,uint8_t *pdata, uint8_t size, uint16_t timeout)
{
  // busyフラグをチェック
  while(LL_I2C_IsActiveFlag_BUSY(I2C1) == SET) {
    if(LL_SYSTICK_IsActiveCounterFlag())
    {
        if (timeout-- == 0)
        {
            I2C1_ClearCR();
            return 1;
        }
    }
  }

  LL_I2C_HandleTransfer(I2C1, device_id, LL_I2C_ADDRSLAVE_7BIT, size, LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_WRITE);

  for(uint8_t i = 0; i < size; i++){
    while(LL_I2C_IsActiveFlag_TXE(I2C1)== RESET) {
        if(LL_SYSTICK_IsActiveCounterFlag())
        {
            if (timeout-- == 0)
            {
                I2C1_ClearCR();
                return 1;
            }
        }        
    }
    LL_I2C_TransmitData8(I2C1, *pdata++);
  }
  while(LL_I2C_IsActiveFlag_TC(I2C1)==RESET) {
    if(LL_SYSTICK_IsActiveCounterFlag())
    {
        if (timeout-- == 0)
        {
            I2C1_ClearCR();
            return 1;
        }
    }
  }
  
  return 0;
}

uint8_t I2C1_ReceiveData(uint8_t device_id , uint8_t *pdata, uint8_t size, uint8_t timeout)
{
  // 初期設定 
  LL_I2C_HandleTransfer(I2C1, device_id, LL_I2C_ADDRSLAVE_7BIT, size, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);

  for(uint8_t i = 0; i < size; i++){
    while(LL_I2C_IsActiveFlag_RXNE(I2C1) == RESET) {
        if(LL_SYSTICK_IsActiveCounterFlag())
        {
            if (timeout-- == 0)
            {
                I2C1_ClearCR();
                return 1;
            }
        }        
    }
    *pdata++ = LL_I2C_ReceiveData8(I2C1);
  }
  
  while(LL_I2C_IsActiveFlag_STOP(I2C1)==RESET) {
    if(LL_SYSTICK_IsActiveCounterFlag())
    {
        if (timeout-- == 0)
        {
            I2C1_ClearCR();
            return 1;
        }
    }      
  }
  LL_I2C_ClearFlag_STOP(I2C1);
  
  // 設定をリセットする
  I2C1_ClearCR();

  return 0;
}

uint8_t I2C_Write_Bytes(uint8_t DeviceAddress, uint8_t MemAddress, uint8_t *pData, uint8_t TxLen,uint16_t Timeout)
{
    uint8_t tx_buffer[TxLen+1];

    tx_buffer[0] = MemAddress;
    memcpy(&tx_buffer[1], pData, TxLen);

    return I2C1_TransmitData(DeviceAddress, tx_buffer, TxLen+1, Timeout);	
}

uint8_t I2C_Write_16bits_reg_Bytes(uint8_t DeviceAddress, uint16_t MemAddress, uint8_t *pData, uint8_t TxLen,uint16_t Timeout)
{
    uint8_t tx_buffer[TxLen+2];

    tx_buffer[0] = MemAddress;
    tx_buffer[1] = ((MemAddress >> 8) & 0xff);
    memcpy(&tx_buffer[2], pData, TxLen);

    return I2C1_TransmitData(DeviceAddress, tx_buffer, TxLen+1, Timeout);	
}

uint8_t I2C_Read_Bytes(uint8_t DeviceAddress, uint8_t MemAddress, uint8_t *pData, uint8_t RxLen,uint16_t Timeout)
{
    if (!I2C1_TransmitData_RepeatedStart(DeviceAddress, &MemAddress, 1, Timeout)) {
        return I2C1_ReceiveData(DeviceAddress ,pData, RxLen, Timeout);

    }
    else {
        return 1;
    }

}
uint8_t I2C_Read_16bits_reg_Bytes(uint8_t DeviceAddress, uint16_t MemAddress, uint8_t *pData, uint8_t RxLen,uint16_t Timeout)
{
    if (!I2C1_TransmitData_RepeatedStart(DeviceAddress, &MemAddress, 2, Timeout)) {
        return I2C1_ReceiveData(DeviceAddress ,pData, RxLen, Timeout);

    }
    else {
        return 1;
    }
}
/* USER CODE END 1 */

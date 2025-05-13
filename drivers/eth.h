/**
  ******************************************************************************
  * @file    eth.h
  * @brief   This file contains all the function prototypes for
  *          the eth.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ETH_H__
#define __ETH_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "string.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern ETH_HandleTypeDef heth;

/* USER CODE BEGIN Private defines */

#define RMII_TXD1_Pin GPIO_PIN_13
#define RMII_TXD1_GPIO_Port GPIOB
#define RMII_TXD0_Pin GPIO_PIN_12
#define RMII_TXD0_GPIO_Port GPIOB

#define RMII_TX_EN_Pin GPIO_PIN_11
#define RMII_TX_EN_GPIO_Port GPIOB

#define LAN_nRST_Pin GPIO_PIN_0
#define LAN_nRST_GPIO_Port GPIOC

#define RMII_MDC_Pin GPIO_PIN_1
#define RMII_MDC_GPIO_Port GPIOC

#define RMII_REF_CLK_Pin GPIO_PIN_1
#define RMII_REF_CLK_GPIO_Port GPIOA

#define RMII_RXD0_Pin GPIO_PIN_4
#define RMII_RXD0_GPIO_Port GPIOC

#define RMII_MDIO_Pin GPIO_PIN_2
#define RMII_MDIO_GPIO_Port GPIOA

#define RMII_RXD1_Pin GPIO_PIN_5
#define RMII_RXD1_GPIO_Port GPIOC

#define RMII_CRS_DV_Pin GPIO_PIN_7
#define RMII_CRS_DV_GPIO_Port GPIOA
/* USER CODE END Private defines */

void MX_ETH_Init(void);

/* USER CODE BEGIN Prototypes */
void PHY_Init(void);

void low_level_output(uint8_t *p,uint32_t length);

int bfin_EMAC_send (void *packet, int length);
int bfin_EMAC_recv (uint8_t * packet, size_t size);

void printfBuffer(uint8_t *dat, uint32_t len );
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ETH_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

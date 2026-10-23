/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

extern CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

/* USER CODE BEGIN Prototypes */
void can_filter_init(void);
void Set_FEED_Motor_Current(int16_t i1);
void PHC_GIMBAL(int16_t i1,int16_t i2);
//void PHC_GIMBAL_PITCH(int16_t i1);
//void PHC_GIMBAL_YAW(int16_t i1);
//void PHC_GIMBAL_PITCH(int16_t i2);
void Set_Shoot_Motor_Current(int16_t i1,int16_t i2);
void Send_Current_Contorl_Data_1(int16_t x,int16_t y,int8_t accelerate,int8_t spin_mode,int8_t gimbal_mode,int8_t fire) ;

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */


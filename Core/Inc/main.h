/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void Timer_Start(void);
void Timer_Stop(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RGB1_Pin GPIO_PIN_13
#define RGB1_GPIO_Port GPIOC
#define RGB2_Pin GPIO_PIN_14
#define RGB2_GPIO_Port GPIOC
#define RGB3_Pin GPIO_PIN_15
#define RGB3_GPIO_Port GPIOC
#define F11_Pin GPIO_PIN_0
#define F11_GPIO_Port GPIOC
#define F12_Pin GPIO_PIN_1
#define F12_GPIO_Port GPIOC
#define F13_Pin GPIO_PIN_2
#define F13_GPIO_Port GPIOC
#define F14_Pin GPIO_PIN_3
#define F14_GPIO_Port GPIOC
#define Motor1F_Pin GPIO_PIN_0
#define Motor1F_GPIO_Port GPIOA
#define Motor1_Pin GPIO_PIN_1
#define Motor1_GPIO_Port GPIOA
#define Motor3F_Pin GPIO_PIN_2
#define Motor3F_GPIO_Port GPIOA
#define Motor3_Pin GPIO_PIN_3
#define Motor3_GPIO_Port GPIOA
#define ADC1_Pin GPIO_PIN_4
#define ADC1_GPIO_Port GPIOA
#define ADC2_Pin GPIO_PIN_5
#define ADC2_GPIO_Port GPIOA
#define ADC3_Pin GPIO_PIN_6
#define ADC3_GPIO_Port GPIOA
#define ADC4_Pin GPIO_PIN_7
#define ADC4_GPIO_Port GPIOA
#define TR_Pin GPIO_PIN_4
#define TR_GPIO_Port GPIOC
#define ec_Pin GPIO_PIN_5
#define ec_GPIO_Port GPIOC
#define BUTTON1_Pin GPIO_PIN_0
#define BUTTON1_GPIO_Port GPIOB
#define BUTTON2_Pin GPIO_PIN_1
#define BUTTON2_GPIO_Port GPIOB
#define F21_Pin GPIO_PIN_12
#define F21_GPIO_Port GPIOB
#define F22_Pin GPIO_PIN_13
#define F22_GPIO_Port GPIOB
#define F23_Pin GPIO_PIN_14
#define F23_GPIO_Port GPIOB
#define F24_Pin GPIO_PIN_15
#define F24_GPIO_Port GPIOB
#define Motor2F_Pin GPIO_PIN_6
#define Motor2F_GPIO_Port GPIOC
#define Motor2_Pin GPIO_PIN_7
#define Motor2_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_8
#define LED_GPIO_Port GPIOC
#define BUZZER_Pin GPIO_PIN_9
#define BUZZER_GPIO_Port GPIOC
#define BM11_Pin GPIO_PIN_8
#define BM11_GPIO_Port GPIOA
#define BM12_Pin GPIO_PIN_9
#define BM12_GPIO_Port GPIOA
#define BM21_Pin GPIO_PIN_15
#define BM21_GPIO_Port GPIOA
#define OLED_SCL_Pin GPIO_PIN_12
#define OLED_SCL_GPIO_Port GPIOC
#define OLED_SDA_Pin GPIO_PIN_2
#define OLED_SDA_GPIO_Port GPIOD
#define BM22_Pin GPIO_PIN_3
#define BM22_GPIO_Port GPIOB
#define BM31_Pin GPIO_PIN_4
#define BM31_GPIO_Port GPIOB
#define BM32_Pin GPIO_PIN_5
#define BM32_GPIO_Port GPIOB
#define BM41_Pin GPIO_PIN_6
#define BM41_GPIO_Port GPIOB
#define BM42_Pin GPIO_PIN_7
#define BM42_GPIO_Port GPIOB
#define Motor4F_Pin GPIO_PIN_8
#define Motor4F_GPIO_Port GPIOB
#define Motor4_Pin GPIO_PIN_9
#define Motor4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define Read_Pin(port, pin) HAL_GPIO_ReadPin(port, pin)
#define Set_Pin(port, pin) HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET)
#define Reset_Pin(port, pin) HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET)
#define Write_Pin(port, pin, value) HAL_GPIO_WritePin(port, pin ,value);
#define I2C_master_transmit(i2c, addr, txbuf, len) HAL_I2C_Master_Transmit(i2c, addr, txbuf, len, 1000)
#define I2C_master_receive(i2c, addr, rxbuf, len) HAL_I2C_Master_Receive(i2c, addr, rxbuf, len, 1000)
#define Pro_delay(ms) HAL_Delay(ms)
#define get_encoder_count(encoder) __HAL_TIM_GET_COUNTER(encoder)
#define get_ADC_value(adc) HAL_ADC_GetValue(adc)
#define set_pwm_compare(htim,ch,ccr) __HAL_TIM_SET_COMPARE(htim,ch,ccr)
#define get_encoder(htim ) __HAL_TIM_GET_COUNTER(htim)
#define clear_encoder(htim) __HAL_TIM_SET_COUNTER(htim, 0)
#define ec_Pin GPIO_PIN_5
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

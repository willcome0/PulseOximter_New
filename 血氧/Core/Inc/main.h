/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LCD_LEDA_Pin GPIO_PIN_2
#define LCD_LEDA_GPIO_Port GPIOA
#define DB_Pin GPIO_PIN_0
#define DB_GPIO_Port GPIOB
#define DBB1_Pin GPIO_PIN_1
#define DBB1_GPIO_Port GPIOB
#define DBB2_Pin GPIO_PIN_2
#define DBB2_GPIO_Port GPIOB
#define DBB10_Pin GPIO_PIN_10
#define DBB10_GPIO_Port GPIOB
#define DBB11_Pin GPIO_PIN_11
#define DBB11_GPIO_Port GPIOB
#define DBB12_Pin GPIO_PIN_12
#define DBB12_GPIO_Port GPIOB
#define DBB13_Pin GPIO_PIN_13
#define DBB13_GPIO_Port GPIOB
#define DBB14_Pin GPIO_PIN_14
#define DBB14_GPIO_Port GPIOB
#define DBB15_Pin GPIO_PIN_15
#define DBB15_GPIO_Port GPIOB
#define CHARGE_Pin GPIO_PIN_6
#define CHARGE_GPIO_Port GPIOC
#define SCL_Pin GPIO_PIN_7
#define SCL_GPIO_Port GPIOC
#define SDA_Pin GPIO_PIN_8
#define SDA_GPIO_Port GPIOC
#define INT_Pin GPIO_PIN_9
#define INT_GPIO_Port GPIOC
#define LCD_RST_Pin GPIO_PIN_15
#define LCD_RST_GPIO_Port GPIOA
#define LCD_CS_Pin GPIO_PIN_10
#define LCD_CS_GPIO_Port GPIOC
#define LCD_RS_Pin GPIO_PIN_11
#define LCD_RS_GPIO_Port GPIOC
#define LCD_WR_Pin GPIO_PIN_12
#define LCD_WR_GPIO_Port GPIOC
#define LCD_RD_Pin GPIO_PIN_2
#define LCD_RD_GPIO_Port GPIOD
#define DBB3_Pin GPIO_PIN_3
#define DBB3_GPIO_Port GPIOB
#define DBB4_Pin GPIO_PIN_4
#define DBB4_GPIO_Port GPIOB
#define DBB5_Pin GPIO_PIN_5
#define DBB5_GPIO_Port GPIOB
#define DBB6_Pin GPIO_PIN_6
#define DBB6_GPIO_Port GPIOB
#define DBB7_Pin GPIO_PIN_7
#define DBB7_GPIO_Port GPIOB
#define DBB8_Pin GPIO_PIN_8
#define DBB8_GPIO_Port GPIOB
#define DBB9_Pin GPIO_PIN_9
#define DBB9_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

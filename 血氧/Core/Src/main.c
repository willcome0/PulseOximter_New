/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "rtc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LCD.h"
#include "FT6206.h"
#include "myiic.h"
#include <stdio.h>
#include <math.h>
#include "max30102.h"
#include "myiic.h"
#include "key.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/**************************************/
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
void Updata_BatIco(void)
{
	if (last_charge_state != charge_state)
	{
		last_charge_state = charge_state;
		if (charge_state)
			LCD_ShowStr(187, 1, WHITE, BLACK, "電", 16);
		else
			LCD_ShowStr(187, 1, WHITE, BLACK, "箜", 16);
	}
}

void Updata_Time(void)
{
	char str[10] = {0};
	HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);
	if (last_seconds != Time.Seconds)
	{
		last_seconds = Time.Seconds;
		sprintf(str, "%02d:%02d", Time.Hours, Time.Minutes);
		LCD_ShowStr(0, 2, WHITE, BLACK, str, 16);
	}
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_DAC_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

	LCD_Init();
	
	printf("初始化完成\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  HAL_TIM_Base_Start_IT(&htim1);
  
	

	// 喇叭方波测试
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 2048);
	HAL_TIM_Base_Start(&htim2);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
	

begin: ;
	LCD_Clear(WHITE);
	LCD_Fill(0, 0, 240, 18, BLACK);
	LCD_ShowBat(210, 4, 4);
	LCD_ShowStr(80, 1, WHITE, BLACK, "血氧检测仪", 16);
	LCD_Fill(65, 268, 175, 306, GRAYBLUE);
	LCD_ShowStr(72, 275, WHITE, GRAYBLUE, "开始检测", 24);
	
	/********************心率界面结果区域*******************************/
	LCD_Fill(0, 110, 240, 174, LGRAYBLUE); // 画背景区域
	LCD_ShowStr(5, 115, BLACK, LGRAYBLUE, "心率: --次/min ", 24);
	LCD_ShowStr(5, 145, BLACK, LGRAYBLUE, "血氧浓度: ---%", 24);
	LCD_ShowPic_Heart(1); // 显示心跳图片
	
	LCD_DrawLine(4, 24, 4, 100, BLACK); // 画线
	LCD_DrawLine(5, 24, 5, 100, BLACK);
	LCD_DrawLine(2, 97, 238, 97, BLACK);
	LCD_DrawLine(2, 98, 238, 98, BLACK);
	/******************************************************************/
	last_seconds = 66;
	while (1) // 测一次或多次
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		Max30102_Measure();
		while (1)
		{


			// 显示充电标识
			Updata_BatIco();
			// 更新时间显示
			Updata_Time();
			
			// 显示触摸坐标
//			if (CTP.ctpxy.ctp_x != 999 && CTP.ctpxy.ctp_y != 999)
//			{
//				char str[30];
//				sprintf(str, "%3d %3d", CTP.ctpxy.ctp_x, CTP.ctpxy.ctp_y);
//				LCD_ShowStr(0, 1, WHITE, BLACK, str, 16);
//			}
			
			/*********************************************/
			// 点击标题后显示
			if (ScanKey_Title())
			{
				LCD_Fill(20, 60, 220, 240, LIGHTGRAY); // 窗体
				LCD_Fill(20, 60, 220, 80, DARKBLUE);   // 窗头
				LCD_ShowStr(88, 62,  WHITE, DARKBLUE, "毕设关于", 16);
				
				LCD_ShowStr(35, 106, BLACK, LIGHTGRAY, "制作人：甄益凡", 16);
				LCD_ShowStr(35, 130, BLACK, LIGHTGRAY, "专  业：电子信息工程", 16);
				LCD_ShowStr(35, 154, BLACK, LIGHTGRAY, "学  号：15160200314", 16);
				
				
				LCD_ShowStr(72, 275, WHITE, GRAYBLUE, " 返  回 ", 24);
				while (!ScanKey_Begin())  // 等待按下返回
				{
					Updata_BatIco();
					Updata_Time();
					char str[30] = {0};
					HAL_RTC_GetDate(&hrtc, &Date, RTC_FORMAT_BIN);
					sprintf(str, "日  期：20%02d年%02d月%02d日", Date.Year, Date.Month, Date.Date);
					LCD_ShowStr(35, 178, BLACK, LIGHTGRAY, str, 16);
					HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);
					sprintf(str, "时  间：%02d时%02d分%02d秒", Time.Hours, Time.Minutes, Time.Seconds);
					LCD_ShowStr(35, 202, BLACK, LIGHTGRAY, str, 16);
				}
				goto begin;
//				LCD_Fill(0, 20, 240, 270, WHITE);
//				LCD_ShowStr(72, 275, WHITE, GRAYBLUE, "开始检测", 24);
			}
			/*********************************************/
			
			/*********************************************/
			// 点击电池图标后显示电池信息
			if (ScanKey_Bat())
			{
				LCD_Fill(35, 60, 205, 240, LIGHTGRAY); // 窗体
				LCD_Fill(35, 60, 205, 80, DARKBLUE);   // 窗头
				LCD_ShowStr(88, 62,  WHITE, DARKBLUE, "电池相关", 16);
				
				LCD_ShowStr(60, 106, BLACK, LIGHTGRAY, "电池容量：400mA", 16);
				
				
				LCD_ShowStr(72, 275, WHITE, GRAYBLUE, " 返  回 ", 24);
				while (!ScanKey_Begin())  // 等待按下返回
				{
					Updata_BatIco();
					Updata_Time();
					LCD_ShowStr(60, 126, BLACK, LIGHTGRAY, "电池电压：3.73V", 16);
					LCD_ShowStr(60, 146, BLACK, LIGHTGRAY, "截止电压：3.20V", 16);
					if (charge_state)
						LCD_ShowStr(60, 166, BLACK, LIGHTGRAY, "电池状态：充电中", 16);
					else
						LCD_ShowStr(60, 166, BLACK, LIGHTGRAY, "电池状态：放电中", 16);
				}
				goto begin;
//				LCD_Fill(0, 20, 240, 270, WHITE);
//				LCD_ShowStr(72, 275, WHITE, GRAYBLUE, "开始检测", 24);
			}
			/*********************************************/
		}
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

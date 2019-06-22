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
//#define WK_FLASH_SIZE          128                                         // flash��С��KB��
//#define WK_FLASH_PAGE_SIZE     2                                           // ��ҳ��С��KB��
//#define WK_FLASH_PAGE_NUM      (WK_FLASH_SIZE/WK_FLASH_PAGE_SIZE)                // ��ҳ����

//#define WK_FLASH_DATA_ADDR     (FLASH_BASE+(WK_FLASH_SIZE-WK_FLASH_PAGE_SIZE)*1024)  // ����������ʼ��ַ

//void Flash_DataWrite(uint16_t offsetAddr, uint64_t writeDate)
//{
//    HAL_FLASH_Unlock(); // ����

//    FLASH_EraseInitTypeDef f;

//    f.TypeErase = FLASH_TYPEERASE_PAGES;
//    f.Page = WK_FLASH_PAGE_NUM-1;      // ѡ�����һ����ҳ
//    f.NbPages = 1;                  // ֻ����һҳ
//    
//    uint32_t PageError = 0;
//    HAL_FLASHEx_Erase(&f, &PageError);  // ����
//    
//    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, WK_FLASH_DATA_ADDR+offsetAddr, writeDate); // д��

//    HAL_FLASH_Lock();   // ��ס
//}

//uint32_t Flash_DataRead(uint16_t offsetAddr)
//{
//    assert_param(IS_FLASH_DATA_AREA(offsetAddr));
//    
//    return *(__IO uint32_t*)(WK_FLASH_DATA_ADDR+offsetAddr);
//}

void Updata_BatIco(void)
{
	if (last_charge_state != charge_state)
	{
		last_charge_state = charge_state;
		if (charge_state)
			LCD_ShowStr(187, 1, WHITE, BLACK, "�", 16);
		else
			LCD_ShowStr(187, 1, WHITE, BLACK, "��", 16);
	}

//	uint16_t bat = (float)adc_value*0.161;
	uint16_t bat = (float)adc_value*0.24;
	if (bat > 390)
		LCD_ShowBat(210, 4, 4);
	else if (bat > 376)
		LCD_ShowBat(210, 4, 3);
	else if (bat > 367)
		LCD_ShowBat(210, 4, 2);
	else if (bat > 350)
		LCD_ShowBat(210, 4, 1);
	else
		LCD_ShowBat(210, 4, 0);
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

void Show_ScreenCloseTime(uint8_t time)
{
	LCD_ShowStr(133, 135, BLACK, LIGHTGRAY, "     ", 24);
	switch (time)
	{
		case 1:
			LCD_ShowStr(139, 135, BLACK, LIGHTGRAY, "30��", 24);
			break;
		case 2:
			LCD_ShowStr(139, 135, BLACK, LIGHTGRAY, "60��", 24);
			break;
		case 3:
			LCD_ShowStr(133, 135, BLACK, LIGHTGRAY, "100��", 24);
			break;
		case 4:
			LCD_ShowStr(133, 135, BLACK, LIGHTGRAY, "200��", 24);
			break;
		case 5:
			LCD_ShowStr(133, 135, BLACK, LIGHTGRAY, "500��", 24);
			break;
		case 6:
			LCD_ShowStr(139, 135, BLACK, LIGHTGRAY, "����", 24);
			break;
		default: break;
	}
}

void Show_TurnOffTime(uint8_t time)
{
	LCD_ShowStr(133, 170, BLACK, LIGHTGRAY, "     ", 24);
	switch (time)
	{
		case 1:
			LCD_ShowStr(139, 170, BLACK, LIGHTGRAY, "30��", 24);
			break;
		case 2:
			LCD_ShowStr(139, 170, BLACK, LIGHTGRAY, "60��", 24);
			break;
		case 3:
			LCD_ShowStr(133, 170, BLACK, LIGHTGRAY, "100��", 24);
			break;
		case 4:
			LCD_ShowStr(133, 170, BLACK, LIGHTGRAY, "200��", 24);
			break;
		case 5:
			LCD_ShowStr(133, 170, BLACK, LIGHTGRAY, "500��", 24);
			break;
		case 6:
			LCD_ShowStr(139, 170, BLACK, LIGHTGRAY, "����", 24);
			break;
		default: break;
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
	
	printf("��ʼ�����\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3); // ��Ļ����pwm
  
begin: ;
	LCD_Clear(WHITE);
	LCD_Fill(0, 0, 240, 18, BLACK);
	LCD_ShowStr(80, 1, WHITE, BLACK, "Ѫ�������", 16);
	
	/********************����"��ʼ���"����*******************************/
	LCD_Fill(65, 268, 175, 306, GRAYBLUE);
	LCD_ShowStr(72, 275, WHITE, GRAYBLUE, "��ʼ���", 24);
	
	/********************����"����"����*******************************/
	LCD_Fill(214, 262, 240, 320, GRAYBLUE);
	LCD_ShowStr(220, 271, WHITE, GRAYBLUE, "��", 16);
	LCD_ShowStr(220, 295, WHITE, GRAYBLUE, "��", 16);
	
	/********************���ʽ���������*******************************/
	LCD_Fill(0, 110, 240, 174, LGRAYBLUE); // ����������
	LCD_ShowStr(5, 115, BLACK, LGRAYBLUE, "����:    ��/min ", 24);
	LCD_ShowStr(5, 145, BLACK, LGRAYBLUE, "Ѫ��Ũ��:    %", 24);
	LCD_ShowPic_Heart(1); // ��ʾ����ͼƬ
	
	LCD_DrawLine(4, 24, 4, 100, BLACK); // ����
	LCD_DrawLine(5, 24, 5, 100, BLACK);
	LCD_DrawLine(2, 97, 238, 97, BLACK);
	LCD_DrawLine(2, 98, 238, 98, BLACK);
	/******************************************************************/
	last_seconds = 66;
	while (1) // ��һ�λ���
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		while (1)
		{
			// ��ʾ����ʶ
			Updata_BatIco();
			// ����ʱ����ʾ
			Updata_Time();
			
			/***********���"��ʼ���"**********************************/
			if (ScanKey_Begin())
			{
				LCD_Fill(65, 268, 175, 306, BRRED);
				LCD_ShowStr(72, 275, WHITE, BRRED, "���ڼ��", 24);
				LCD_Fill(6, 25, 239, 96, WHITE);
				
				Max30102_Measure(); // Ѫ����⣬����ͼ����
				
				LCD_ShowNum(77, 115, BLACK, LGRAYBLUE, HeartRate_Value, 3, 24, ' ');
				LCD_ShowNum(122, 145, BLACK, LGRAYBLUE, SP02_Value, 3, 24, ' ');
				char str_temp[30];
				printf("������\r\n");
				sprintf(str_temp, "����:%d��/��  Ѫ��:%d%%\r\n", HeartRate_Value, SP02_Value);
				printf(str_temp);

				LCD_Fill(65, 268, 175, 306, GRAYBLUE);
				LCD_ShowStr(72, 275, WHITE, GRAYBLUE, "��ʼ���", 24);
			}
			
			/***********����������ʾ**********************************/
			if (ScanKey_Title())
			{
				LCD_Fill(20, 60, 220, 240, LIGHTGRAY); // ����
				LCD_Fill(20, 60, 220, 80, DARKBLUE);   // ��ͷ
				LCD_ShowStr(88, 62,  WHITE, DARKBLUE, "�������", 16);
				
				LCD_ShowStr(35, 106, BLACK, LIGHTGRAY, "�����ˣ����淲", 16);
				LCD_ShowStr(35, 130, BLACK, LIGHTGRAY, "ר  ҵ��������Ϣ����", 16);
				LCD_ShowStr(35, 154, BLACK, LIGHTGRAY, "ѧ  �ţ�15160200314", 16);
				
				LCD_ShowStr(72, 275, WHITE, GRAYBLUE, " ��  �� ", 24);
				while (!ScanKey_Begin())  // �ȴ����·���
				{
					Updata_BatIco();
					Updata_Time();
					char str[30] = {0};
					HAL_RTC_GetDate(&hrtc, &Date, RTC_FORMAT_BIN);
					sprintf(str, "��  �ڣ�20%02d��%02d��%02d��", Date.Year, Date.Month, Date.Date);
					LCD_ShowStr(35, 178, BLACK, LIGHTGRAY, str, 16);
					HAL_RTC_GetTime(&hrtc, &Time, RTC_FORMAT_BIN);
					sprintf(str, "ʱ  �䣺%02dʱ%02d��%02d��", Time.Hours, Time.Minutes, Time.Seconds);
					LCD_ShowStr(35, 202, BLACK, LIGHTGRAY, str, 16);
				}
				goto begin;
//				LCD_Fill(0, 20, 240, 270, WHITE);
//				LCD_ShowStr(72, 275, WHITE, GRAYBLUE, "��ʼ���", 24);
			}
			/*********����������ʾ ����************************************/
			
			/*********������ͼ�����ʾ�����Ϣ************************************/
			if (ScanKey_Bat())
			{
				LCD_Fill(35, 60, 205, 240, LIGHTGRAY); // ����
				LCD_Fill(35, 60, 205, 80, DARKBLUE);   // ��ͷ
				LCD_ShowStr(88, 62,  WHITE, DARKBLUE, "������", 16);
				
				LCD_ShowStr(60, 106, BLACK, LIGHTGRAY, "���������400mA", 16);
				
				
				LCD_ShowStr(72, 275, WHITE, GRAYBLUE, " ��  �� ", 24);
				while (!ScanKey_Begin())  // �ȴ����·���
				{
					Updata_BatIco();
					Updata_Time();
					
					char str[30];
					sprintf(str, "��ص�ѹ��%1.2fV", (float)adc_value*0.001886);
					LCD_ShowStr(60, 130, BLACK, LIGHTGRAY, str, 16);

					LCD_ShowStr(60, 154, BLACK, LIGHTGRAY, "��ֹ��ѹ��3.20V", 16);
					if (charge_state)
						LCD_ShowStr(60, 178, BLACK, LIGHTGRAY, "���״̬�������", 16);
					else
						LCD_ShowStr(60, 178, BLACK, LIGHTGRAY, "���״̬���ŵ���", 16);
				}
				goto begin;
//				LCD_Fill(0, 20, 240, 270, WHITE);
//				LCD_ShowStr(72, 275, WHITE, GRAYBLUE, "��ʼ���", 24);
			}
			/**********������ͼ�� ����***********************************/
			
			/**********������ú���ʾ***********************************/
			if (ScanKey_Set())
			{
				LCD_Fill(20, 60, 220, 240, LIGHTGRAY); // ����
				LCD_Fill(20, 60, 220, 80, DARKBLUE);   // ��ͷ
				LCD_ShowStr(88, 62,  WHITE, DARKBLUE, "  ����  ", 16);
				
				LCD_ShowStr(35, 105, BLACK, LIGHTGRAY, "���ȵ��ڣ�", 16);
				LCD_ShowStr(35, 140, BLACK, LIGHTGRAY, "�Զ�Ϣ����", 16);
				LCD_ShowStr(35, 175, BLACK, LIGHTGRAY, "�Զ��ػ���", 16);
				
				LCD_Fill(115-5, 105-5, 115+8+5, 105+16+5, GRAYBLUE);
				LCD_ShowStr(115, 105, WHITE, GRAYBLUE, "-", 16);
				LCD_Fill(165-5, 105-5, 165+8+5, 105+16+5, GRAYBLUE);
				LCD_ShowStr(165, 105, LIGHTGRAY, GRAYBLUE, "+", 16);
				
				LCD_Fill(115-5, 140-5, 115+8+5, 140+16+5, GRAYBLUE);
				LCD_ShowStr(115, 140, WHITE, GRAYBLUE, "-", 16);
				LCD_Fill(202-5, 140-5, 202+8+5, 140+16+5, GRAYBLUE);
				LCD_ShowStr(202, 140, LIGHTGRAY, GRAYBLUE, "+", 16);
				
				LCD_Fill(115-5, 175-5, 115+8+5, 175+16+5, GRAYBLUE);
				LCD_ShowStr(115, 175, WHITE, GRAYBLUE, "-", 16);
				LCD_Fill(202-5, 175-5, 202+8+5, 175+16+5, GRAYBLUE);
				LCD_ShowStr(202, 175, LIGHTGRAY, GRAYBLUE, "+", 16);
				
				LCD_ShowStr(72, 275, WHITE, GRAYBLUE, " ��  �� ", 24);
				
				
				int8_t Light_Num = 1;
				int8_t ScreenClose_Time = 3;
				int8_t TurnOff_Time = 5;
				
				LCD_ShowNum(139, 100, BLACK, LIGHTGRAY, Light_Num, 1, 24, ' ');
				Show_ScreenCloseTime(ScreenClose_Time);
				Show_TurnOffTime(TurnOff_Time);
				
				while (!ScanKey_Begin())  // �ȴ����·���
				{
					Updata_BatIco();
					Updata_Time();
					
					/**************��������********************/
					if (ScanKey_LightAdd()) // ����������
					{
						if (Light_Num >= 9)
						{
							Light_Num = 9;
						}
						else
						{
							LCD_ShowNum(139, 100, BLACK, LIGHTGRAY, ++Light_Num, 1, 24, ' ');
							__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, Light_Num*99);
							while (ScanKey_LightAdd()); // �ȴ������ɿ�
						}
					}
					if (ScanKey_LightSub()) // �������ȼ�
					{
						if (Light_Num <= 1)
						{
							Light_Num = 1;
						}
						else
						{
							LCD_ShowNum(139, 100, BLACK, LIGHTGRAY, --Light_Num, 1, 24, ' ');
							__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, Light_Num*99);
							while (ScanKey_LightSub()); // �ȴ������ɿ�
						}
					}
					
					// Ϣ��ʱ��һ���֣�30��60��100��200��500�롢����
					if (ScanKey_ScreenCloseAdd())	// ����Ϣ��ʱ����
					{
						if (ScreenClose_Time >= 6)
						{
							ScreenClose_Time = 6;
						}
						else
						{
							Show_ScreenCloseTime(++ScreenClose_Time);
							while (ScanKey_ScreenCloseAdd()); // �ȴ������ɿ�
						}
					}
					if (ScanKey_ScreenCloseSub())	// ����Ϣ��ʱ���
					{
						if (ScreenClose_Time <= 1)
						{
							ScreenClose_Time = 1;
						}
						else
						{
							Show_ScreenCloseTime(--ScreenClose_Time);
							while (ScanKey_ScreenCloseSub()); // �ȴ������ɿ�
						}
					}
					
					// �ػ�ʱ��֣�30��60��100��200��500�롢����
					if (ScanKey_TurnOffAdd())	// ����Ϣ��ʱ����
					{
						if (TurnOff_Time >= 6)
						{
							TurnOff_Time = 6;
						}
						else
						{
							Show_TurnOffTime(++TurnOff_Time);
							while (ScanKey_TurnOffAdd()); // �ȴ������ɿ�
						}
					}
					if (ScanKey_TurnOffSub())	// ����Ϣ��ʱ���
					{
						if (TurnOff_Time <= 1)
						{
							TurnOff_Time = 1;
						}
						else
						{
							Show_TurnOffTime(--TurnOff_Time);
							while (ScanKey_TurnOffSub()); // �ȴ������ɿ�
						}
					}
					
					/******************************************/
				}
				goto begin;
			}
			/****************������� ����*****************************/
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

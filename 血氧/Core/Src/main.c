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
#include "dac.h"
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
#include "algorithm.h"
#include "max30102.h"
#include "myiic.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// 血氧相关
uint32_t aun_ir_buffer[500]; //IR LED sensor data
int32_t n_ir_buffer_length;    //data length
uint32_t aun_red_buffer[500];    //Red LED sensor data
int32_t n_sp02; //SPO2 value
int8_t ch_spo2_valid;   //indicator to show if the SP02 calculation is valid
int32_t n_heart_rate;   //heart rate value
int8_t  ch_hr_valid;    //indicator to show if the heart rate calculation is valid
uint8_t uch_dummy;

#define MAX_BRIGHTNESS 255
uint32_t red_min, red_max;
uint32_t ir_min, ir_max;
void dis_DrawCurve(uint32_t* data,uint8_t x, uint16_t color);
/**************************************/
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

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
	


	LCD_Fill(0, 0, 240, 18, BLACK);
	LCD_ShowBat(210, 4, 4);
	LCD_ShowStr(80, 1, WHITE, BLACK, "血氧检测仪", 16);
	LCD_Fill(65, 268, 175, 306, GRAYBLUE);
	LCD_ShowStr(72, 275, WHITE, GRAYBLUE, "开始检测", 24);


	
	////////////////////////////////////
	uint32_t un_prev_data;  
	int i;
	int32_t n_brightness;
	float f_temp;
	uint8_t temp_num=0;
	uint8_t temp[6];
	uint8_t str[100];
	uint8_t dis_hr=0,dis_spo2=0;
	
	max30102_init();
	red_min=0x3FFFF;
	red_max=0;
	
	n_ir_buffer_length=500; //buffer length of 100 stores 5 seconds of samples running at 100sps
	
	//read the first 500 samples, and determine the signal range
    for(i=0;i<n_ir_buffer_length;i++)
    {
        while(HAL_GPIO_ReadPin(INT_GPIO_Port, INT_Pin)==1);   //wait until the interrupt pin asserts
        
		max30102_FIFO_ReadBytes(REG_FIFO_DATA,temp);
		aun_red_buffer[i] =  (long)((long)((long)temp[0]&0x03)<<16) | (long)temp[1]<<8 | (long)temp[2];    // Combine values to get the actual number
		aun_ir_buffer[i] = (long)((long)((long)temp[3] & 0x03)<<16) |(long)temp[4]<<8 | (long)temp[5];   // Combine values to get the actual number
            
        if(red_min>aun_red_buffer[i])
            red_min=aun_red_buffer[i];
        if(red_max<aun_red_buffer[i])
            red_max=aun_red_buffer[i];
		
		if(ir_min>aun_ir_buffer[i])
            ir_min=aun_ir_buffer[i];
        if(ir_max<aun_ir_buffer[i])
            ir_max=aun_ir_buffer[i];
    }
	un_prev_data=aun_red_buffer[i];
	//calculate heart rate and SpO2 after first 500 samples (first 5 seconds of samples)
    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
	
	//////////////////////////////////////
//	while (1)
	{
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */
//	sprintf(str, "%3d %3d", CTP.ctpxy.ctp_x, CTP.ctpxy.ctp_y);
//	LCD_ShowStr(0, 1, WHITE, BLACK, str, 16);

		i=0;
        red_min=0x3FFFF;
        red_max=0;
		
		//dumping the first 100 sets of samples in the memory and shift the last 400 sets of samples to the top
        for(i=100;i<500;i++)
        {
            aun_red_buffer[i-100]=aun_red_buffer[i];
            aun_ir_buffer[i-100]=aun_ir_buffer[i];
            

            if(red_min>aun_red_buffer[i])
				red_min=aun_red_buffer[i];
            if(red_max<aun_red_buffer[i])
				red_max=aun_red_buffer[i];
        }
		//take 100 sets of samples before calculating the heart rate.
        for(i=400;i<500;i++)
        {
            un_prev_data=aun_red_buffer[i-1];
            while(HAL_GPIO_ReadPin(INT_GPIO_Port, INT_Pin)==1);
			
            max30102_FIFO_ReadBytes(REG_FIFO_DATA,temp);
			aun_red_buffer[i] =  (long)((long)((long)temp[0]&0x03)<<16) | (long)temp[1]<<8 | (long)temp[2];    // Combine values to get the actual number
			aun_ir_buffer[i] = (long)((long)((long)temp[3] & 0x03)<<16) |(long)temp[4]<<8 | (long)temp[5];   // Combine values to get the actual number
        
            if(aun_red_buffer[i]>un_prev_data)
            {
                f_temp=aun_red_buffer[i]-un_prev_data;
                f_temp/=(red_max-red_min);
                f_temp*=MAX_BRIGHTNESS;
                n_brightness-=(int)f_temp;
                if(n_brightness<0)
                    n_brightness=0;
            }
            else
            {
                f_temp=un_prev_data-aun_red_buffer[i];
                f_temp/=(red_max-red_min);
                f_temp*=MAX_BRIGHTNESS;
                n_brightness+=(int)f_temp;
                if(n_brightness>MAX_BRIGHTNESS)
                    n_brightness=MAX_BRIGHTNESS;
            }
			//send samples and calculation result to terminal program through UART
			if(ch_hr_valid == 1 && ch_spo2_valid ==1 && n_heart_rate<120 && n_sp02<110)
			{
				dis_hr = n_heart_rate;
				dis_spo2 = n_sp02;
			}
			else
			{
				dis_hr = 0;
				dis_spo2 = 0;
			}
				printf("B%i\r\n", n_heart_rate); 
				//printf("HRvalid=%i, ", ch_hr_valid);
				printf("S%i\r\n", n_sp02);
				//printf("SPO2Valid=%i\r\n", ch_spo2_valid);
		}
        maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
		
		//显示刷新
		if(dis_hr == 0 && dis_spo2 == 0)
		{
			sprintf((char *)str,"HR:--- SpO2:--- ");
		}
		else{
			sprintf((char *)str,"HR:%3d SpO2:%3d ",dis_hr,dis_spo2);
		}
		dis_DrawCurve(aun_red_buffer, 20, RED);
		dis_DrawCurve(aun_ir_buffer,   0, BLACK);
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

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
}

/* USER CODE BEGIN 4 */
void dis_DrawCurve(uint32_t* data,uint8_t x, uint16_t color)
{
	uint16_t i;
	uint32_t max=0, min=262144;
	uint32_t temp;
	uint32_t compress;
	
//	for(i=20;i<500;i++)
//	{
//		if(data[i]>max)
//		{
//			max = data[i];
//		}
//		if(data[i]<min)
//		{
//			min = data[i];
//		}
//	}
	min = red_min;
	max = red_max;
	compress = (max-min)/30;
	printf("max:%6d min:%6d com:%6d max-min:%6d", max, min, compress, max-min);
	
	for(i=10;i<250;i++)
	{
		temp = (data[i*2] + data[i*2+1])/2 - min;
		temp/=compress;
		if(temp>40)temp=40;
		LCD_DrawPoint(i, 83-x-temp, color);
	}
}
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


///* USER CODE BEGIN Header */
///**
//  ******************************************************************************
//  * @file           : main.c
//  * @brief          : Main program body
//  ******************************************************************************
//  ** This notice applies to any and all portions of this file
//  * that are not between comment pairs USER CODE BEGIN and
//  * USER CODE END. Other portions of this file, whether 
//  * inserted by the user or by software development tools
//  * are owned by their respective copyright owners.
//  *
//  * COPYRIGHT(c) 2019 STMicroelectronics
//  *
//  * Redistribution and use in source and binary forms, with or without modification,
//  * are permitted provided that the following conditions are met:
//  *   1. Redistributions of source code must retain the above copyright notice,
//  *      this list of conditions and the following disclaimer.
//  *   2. Redistributions in binary form must reproduce the above copyright notice,
//  *      this list of conditions and the following disclaimer in the documentation
//  *      and/or other materials provided with the distribution.
//  *   3. Neither the name of STMicroelectronics nor the names of its contributors
//  *      may be used to endorse or promote products derived from this software
//  *      without specific prior written permission.
//  *
//  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
//  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//  *
//  ******************************************************************************
//  */
///* USER CODE END Header */

///* Includes ------------------------------------------------------------------*/
//#include "main.h"
//#include "dac.h"
//#include "tim.h"
//#include "usart.h"
//#include "gpio.h"

///* Private includes ----------------------------------------------------------*/
///* USER CODE BEGIN Includes */
//#include "LCD.h"
//#include "MAX30100.h"
//#include "FT6206.h"
//#include "iic.h"
//#include <stdio.h>
//#include <math.h>
//#include "algorithm.h"
///* USER CODE END Includes */

///* Private typedef -----------------------------------------------------------*/
///* USER CODE BEGIN PTD */


///* USER CODE END PTD */

///* Private define ------------------------------------------------------------*/
///* USER CODE BEGIN PD */
//uint16_t g_fft_index=0;
///* USER CODE END PD */

///* Private macro -------------------------------------------------------------*/
///* USER CODE BEGIN PM */

///* USER CODE END PM */

///* Private variables ---------------------------------------------------------*/

///* USER CODE BEGIN PV */
//#define CORRECTED_VALUE	50
//#define MAX_BRIGHTNESS 255

//void sp02_treated_fun(uint16_t max_index)
//{
//	float sp02_num=0;
// 
//   
// 	printf("\r\n zhiliu s1=%f,s2=%f \r\n",s1[0].real,s2[0].real);
//	printf("\r\n s1=%f,s2=%f \r\n",s1[max_index].real,s2[max_index].real);
//	if((s1[max_index].real*s2[0].real)>(s2[max_index].real*s1[0].real))  //if   ir>red      sp02>75%
//	{
//		sp02_num = (s2[max_index].real*s1[0].real)/(s1[max_index].real*s2[0].real);
//		printf("\r\nsp02_num  : %f\r\n",sp02_num*100);
//		printf("\r\n血氧含量为: %f\r\n",(1-sp02_num)*100+CORRECTED_VALUE);
//		
////		OLED_ShowString(0,0, "SpO2:",16);
////		if((1-sp02_num)*100+CORRECTED_VALUE>99)
////			OLED_ShowString(40,0, "99",16);
////		else
////			OLED_ShowNum(40,0,(1-sp02_num)*100+CORRECTED_VALUE,4,16);

////		OLED_ShowString(80,0,"%",16); 
////		OLED_ShowString(0,30,"Heart Rate:",12);   
////		OLED_Refresh_Gram();//更新显示到OLED 	 
//		
//	}
//	else   // sp02<75%
//	{
//		printf("\r\n 严重缺氧! \r\n");
//		
////		OLED_ShowString(0,0, "SpO2:",16);
////		OLED_ShowString(40,0,"ANOXIA!",16);
////		OLED_ShowString(0,30,"Heart Rate:",12);   
////		OLED_Refresh_Gram();//更新显示到OLED 
//	}	
//}
///* USER CODE END PV */

///* Private function prototypes -----------------------------------------------*/
//void SystemClock_Config(void);
///* USER CODE BEGIN PFP */

///* USER CODE END PFP */

///* Private user code ---------------------------------------------------------*/
///* USER CODE BEGIN 0 */
//uint32_t aun_ir_buffer[500];  //IR LED数据
//int32_t n_ir_buffer_length;   //IR LED数据长度
//uint32_t aun_red_buffer[500]; //Red LED数据

//int32_t n_sp02;               //血氧浓度值
//int8_t ch_spo2_valid;         //血氧浓度值是否有效
//int32_t n_heart_rate;         //心跳值
//int8_t ch_hr_valid;           //心跳值是否有效
//uint8_t uch_dummy;

//extern uint8_t g_INT;
///* USER CODE END 0 */

///**
//  * @brief  The application entry point.
//  * @retval int
//  */
//int main(void)
//{
//  /* USER CODE BEGIN 1 */

//  /* USER CODE END 1 */

//  /* MCU Configuration--------------------------------------------------------*/

//  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
//  HAL_Init();

//  /* USER CODE BEGIN Init */

//  /* USER CODE END Init */

//  /* Configure the system clock */
//  SystemClock_Config();

//  /* USER CODE BEGIN SysInit */

//  /* USER CODE END SysInit */

//  /* Initialize all configured peripherals */
//  MX_GPIO_Init();
//  MX_USART1_UART_Init();
//  MX_TIM3_Init();
//  MX_DAC_Init();
//  MX_TIM2_Init();
//  MX_TIM1_Init();
//  /* USER CODE BEGIN 2 */

//	LCD_Init();
//	
//	printf("初始化完成\r\n");
//  /* USER CODE END 2 */

//  /* Infinite loop */
//  /* USER CODE BEGIN WHILE */
//  
//  HAL_TIM_Base_Start_IT(&htim1);
//  
//	uint32_t time_count = 0;
//	double temp, temp1, temp2;
//	

//	// 喇叭方波测试
//	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 2048);
//	HAL_TIM_Base_Start(&htim2);
//	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);
//	

//	LCD_Fill(0, 0, 240, 18, BLACK);
//	LCD_ShowBat(210, 4, 4);
//	LCD_ShowStr(80, 1, WHITE, BLACK, "血氧检测仪", 16);
//	LCD_Fill(65, 268, 175, 306, GRAYBLUE);
//	LCD_ShowStr(72, 275, WHITE, GRAYBLUE, "开始检测", 24);



//#if (1)
//{
//	char str[30] = {0};
//	
//	////////////////////////////////////

//	Max10300_Reset();
//	HAL_Delay(200);
//	Max10300_Init();
//	HAL_Delay(200);
//	
//	uint32_t un_min, un_max;
//	uint32_t un_prev_data;
//	n_ir_buffer_length = 500;
//	
//	uint16_t i;
//	for (i = 0; i < n_ir_buffer_length; i++)
//    {
//		while (!g_INT);
//		g_INT = 0;
//        maxim_max30102_read_fifo((aun_red_buffer + i), (aun_ir_buffer + i)); // 读取数据
//		
//        if (un_min > aun_red_buffer[i])
//            un_min = aun_red_buffer[i]; // 更新最小数据
//        if (un_max < aun_red_buffer[i])
//            un_max = aun_red_buffer[i]; // 更新最大数据
//    }
//	un_prev_data = aun_red_buffer[i];
//	maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, 
//					aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
//	
//    while (1)
//    {
//        un_min = 0x3FFFF;
//        un_max = 0;
//		int32_t n_brightness;
//		float f_temp;

//        // 将前100组样本转储到内存中，并将最后400组样本移到顶部

//		for (uint16_t i = 100; i < 500; i++)
//		{
//			aun_red_buffer[i - 100] = aun_red_buffer[i];
//			aun_ir_buffer[i - 100] = aun_ir_buffer[i];

//			// 更新最大和最小值
//			if (un_min > aun_red_buffer[i])
//				un_min = aun_red_buffer[i];
//			if (un_max < aun_red_buffer[i])
//				un_max = aun_red_buffer[i];
//		}

//        // 在计算心率之前取100组样本。
//        for (uint16_t i = 400; i < 500; i++)
//        {
//            un_prev_data = aun_red_buffer[i - 1];
////            while (HAL_GPIO_ReadPin(INT_GPIO_Port, INT_Pin)==1);
//			while (!g_INT);
//			g_INT = 0;
//            maxim_max30102_read_fifo((aun_red_buffer + i), (aun_ir_buffer + i));

//            if (aun_red_buffer[i] > un_prev_data) // 只是根据相邻两个AD数据的偏差来确定LED的亮度
//            {
//                f_temp = aun_red_buffer[i] - un_prev_data;
//                f_temp /= (un_max - un_min);
//                f_temp *= MAX_BRIGHTNESS;
//                n_brightness -= (int)f_temp;
//                if (n_brightness < 0)
//                    n_brightness = 0;
//            }
//            else
//            {
//                f_temp = un_prev_data - aun_red_buffer[i];
//                f_temp /= (un_max - un_min);
//                f_temp *= MAX_BRIGHTNESS;
//                n_brightness += (int)f_temp;
//                if (n_brightness > MAX_BRIGHTNESS)
//                    n_brightness = MAX_BRIGHTNESS;
//            }

////            pwmled.write(1 - (float)n_brightness / 256); //pwm control led brightness
//			LCD_ShowPic_Heart(n_brightness < 120);

//			
//			uint8_t temp[30] = {0};
//			sprintf(temp, "%6d", aun_red_buffer[i]);
//			LCD_ShowStr(0, 50, WHITE, BLACK, temp, 16);
//			sprintf(temp, "%6d", aun_ir_buffer[i]);
//			LCD_ShowStr(0, 66, WHITE, BLACK, temp, 16);
//			
//			sprintf(temp, "%4d %1d", n_heart_rate, ch_hr_valid);
//			LCD_ShowStr(0, 82, WHITE, BLACK, temp, 16);
//			
//			sprintf(temp, "%4d %1d", n_sp02, ch_spo2_valid);
//			LCD_ShowStr(0, 98, WHITE, BLACK, temp, 16);

////			temp[0] = 0xAA;
////			temp[1] = 0x05;
////			temp[2] = 0xAF;
////			temp[3] = 0x07;
////			temp[4] = 0x0A; // 数据位数
////			
////			temp[5] = (aun_red_buffer[i]*100)>>(8*3);
////			temp[6] = (aun_red_buffer[i]*100)>>(8*2);
////			temp[7] = (aun_red_buffer[i]*100)>>(8*1);
////			temp[8] = (aun_red_buffer[i]*100)>>(8*0);

////			temp[9] =  (aun_ir_buffer[i]*100)>>(8*3);
////			temp[10] = (aun_ir_buffer[i]*100)>>(8*2);
////			temp[11] = (aun_ir_buffer[i]*100)>>(8*1);
////			temp[12] = (aun_ir_buffer[i]*100)>>(8*0);
////			
////			temp[13] = 0;
////			temp[14] = 0;

////			temp[15] = 0;
////			for (uint8_t i=0; i<15; i++)
////				temp[15] += temp[i];
////			HAL_UART_Transmit(&huart1, (uint8_t *)temp, 16, 0xffff);

////	// 检测"开始检测"被按下
////	if (65 < CTP.ctpxy.ctp_x && CTP.ctpxy.ctp_x < 175 &&
////		268 < CTP.ctpxy.ctp_y && CTP.ctpxy.ctp_y < 306)
////	{
////		LCD_ShowStr(0, 0, WHITE, BLACK, "1", 16);
////	}
////	else
////	{
////		LCD_ShowStr(0, 0, WHITE, BLACK, "0", 16);
////	}

//        }
//        maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
//    }
//}	
//#endif	
//	
//#if 0
//	uint16_t temp_num=0;
//	uint16_t fifo_word_buff[15][2];
//	uint16_t Heart_Rate=0;
//	uint16_t s1_max_index=0;
//	uint16_t s2_max_index=0;
//	//////////////////////////////////////
//  while (1)
//  {
//    /* USER CODE END WHILE */

//    /* USER CODE BEGIN 3 */
////	sprintf(str, "%3d %3d", CTP.ctpxy.ctp_x, CTP.ctpxy.ctp_y);
////	LCD_ShowStr(0, 1, WHITE, BLACK, str, 16);



//	uint8_t temp_num = Max30100_ReadByte(INTERRUPT_REG);
//	if (INTERRUPT_REG_A_FULL&temp_num)
//	{
//		Max10300_ReadFIFO(0x05, fifo_word_buff, 15);

//		for(uint16_t i=0; i<15; i++)
//		{ 
//			if(g_fft_index < FFT_N)
//			{
//				s1[g_fft_index].real = fifo_word_buff[i][0];
//				s1[g_fft_index].imag= 0;
//				s2[g_fft_index].real = fifo_word_buff[i][1];
//				s2[g_fft_index].imag= 0;
//				g_fft_index++;
//			}
//		}
//		if(g_fft_index>=FFT_N)
//		{
//			
//			FFT(s1);
//			FFT(s2);
//			for(uint16_t i=0;i<FFT_N;i++) 
//			{
//				s1[i].real=sqrtf(s1[i].real*s1[i].real+s1[i].imag*s1[i].imag);
//				s2[i].real=sqrtf(s2[i].real*s2[i].real+s2[i].imag*s2[i].imag);
//			}
//			{
//				#define START_INDEX    10   //滤出低频干扰
//				for(uint16_t index=START_INDEX; index<60; index++)
//				{	
////				#ifdef SAMPLE_50
//						printf("f=%3.3f HZ,s1[%3d] = %f \r\n", 50.0/FFT_N*index, index,s1[index].real);
//						printf("f=%3.3f HZ,s2[%3d] = %f \r\n", 50.0/FFT_N*index, index,s2[index].real);
////				#else
////						printf("f=%3.3f HZ,s1[%3d] = %f \r\n",100.0/FFT_N*index,index,s1[index].real);
////						printf("f=%3.3f HZ,s2[%3d] = %f \r\n",100.0/FFT_N*index,index,s2[index].real);
////				#endif
//				}
//			}
//			s1_max_index = find_max_num_index(s1, 60);
//			s2_max_index = find_max_num_index(s2, 60);
////			if(s1_max_index == s2_max_index)	
//			{
//				Heart_Rate =  60*50*((s1_max_index+s2_max_index )/2)/FFT_N;
//				sp02_treated_fun(s1_max_index);
//				
//				sprintf(str, "%5d", Heart_Rate-10);
//				LCD_ShowStr(0, 20, WHITE, BLACK, str, 16);
//			}
////			else
////			{
////				sprintf(str, "%5d", 9999);
////				LCD_ShowStr(0, 20, WHITE, BLACK, str, 16);
////			}
//			g_fft_index = 0;
//		}
//	}
//	  
////		uint8_t rda;
////		Max30100_WriteByte(0x06, 0x0b); 
////		Max30100_ReadByte(0xff);	// 返回ID

////		//点亮芯片LED
////		Max30100_WriteByte(0x07, 0x43);       // 设置电流，点亮LED
////		//温度功能测试
////		Max30100_WriteByte(0x09,0x66);       // 0X06地址B3位TEMP_EN置1
////		HAL_Delay(50);                                // 等待温度转换完成，不等待，读出数据有误
////		
////		temp1 = Max30100_ReadByte(0x16);            // 读出温度信号

////		temp2 = Max30100_ReadByte(0x17);            // 读出温度小数部分数据


////		temp=temp1+(temp2*0.0625);                  // 计算温度，小数部分最小温度值0.0625
////		
////		sprintf(str, "%5d",(int)(temp1*100));
////		LCD_ShowStr(0, 20, WHITE, BLACK, str, 16);

//  }
//  
//#endif
//  /* USER CODE END 3 */
//}

///**
//  * @brief System Clock Configuration
//  * @retval None
//  */
//void SystemClock_Config(void)
//{
//  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

//  /**Initializes the CPU, AHB and APB busses clocks 
//  */
//  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
//  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
//  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
//  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /**Initializes the CPU, AHB and APB busses clocks 
//  */
//  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
//  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

//  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
//  {
//    Error_Handler();
//  }
//}

///* USER CODE BEGIN 4 */

///* USER CODE END 4 */

///**
//  * @brief  This function is executed in case of error occurrence.
//  * @retval None
//  */
//void Error_Handler(void)
//{
//  /* USER CODE BEGIN Error_Handler_Debug */
//  /* User can add his own implementation to report the HAL error return state */

//  /* USER CODE END Error_Handler_Debug */
//}

//#ifdef  USE_FULL_ASSERT
///**
//  * @brief  Reports the name of the source file and the source line number
//  *         where the assert_param error has occurred.
//  * @param  file: pointer to the source file name
//  * @param  line: assert_param error line source number
//  * @retval None
//  */
//void assert_failed(uint8_t *file, uint32_t line)
//{ 
//  /* USER CODE BEGIN 6 */
//  /* User can add his own implementation to report the file name and line number,
//     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//  /* USER CODE END 6 */
//}
//#endif /* USE_FULL_ASSERT */

///************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

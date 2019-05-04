#include "max30102.h"
#include "myiic.h"
#include "algorithm.h"
#include "LCD.h"
#include "tim.h"
uint8_t max30102_Bus_Write(uint8_t Register_Address, uint8_t Word_Data)
{

	/* ���ô���EEPROM�漴��ȡָ�����У�������ȡ�����ֽ� */

	/* ��1��������I2C���������ź� */
	NVIC_DisableIRQ(TIM1_UP_IRQn);
	IIC_Start();

	/* ��2������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
	IIC_Send_Byte(max30102_WR_address | I2C_WR);	/* �˴���дָ�� */

	/* ��3��������ACK */
	if (IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}

	/* ��4���������ֽڵ�ַ */
	IIC_Send_Byte(Register_Address);
	if (IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}
	
	/* ��5������ʼд������ */
	IIC_Send_Byte(Word_Data);

	/* ��6��������ACK */
	if (IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}

	/* ����I2C����ֹͣ�ź� */
	IIC_Stop();
	NVIC_EnableIRQ(TIM1_UP_IRQn);
	return 1;	/* ִ�гɹ� */

cmd_fail: /* ����ִ��ʧ�ܺ��мǷ���ֹͣ�źţ�����Ӱ��I2C�����������豸 */
	/* ����I2C����ֹͣ�ź� */
	IIC_Stop();
	NVIC_EnableIRQ(TIM1_UP_IRQn);
	return 0;
}



uint8_t max30102_Bus_Read(uint8_t Register_Address)
{
	uint8_t  data;


	/* ��1��������I2C���������ź� */
	NVIC_DisableIRQ(TIM1_UP_IRQn);
	IIC_Start();

	/* ��2������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
	IIC_Send_Byte(max30102_WR_address | I2C_WR);	/* �˴���дָ�� */

	/* ��3��������ACK */
	if (IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}

	/* ��4���������ֽڵ�ַ�� */
	IIC_Send_Byte((uint8_t)Register_Address);
	if (IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}
	

	/* ��6������������I2C���ߡ����濪ʼ��ȡ���� */
	IIC_Start();

	/* ��7������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
	IIC_Send_Byte(max30102_WR_address | I2C_RD);	/* �˴��Ƕ�ָ�� */

	/* ��8��������ACK */
	if (IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}

	/* ��9������ȡ���� */
	{
		data = IIC_Read_Byte(0);	/* ��1���ֽ� */

		IIC_NAck();	/* ���1���ֽڶ����CPU����NACK�ź�(����SDA = 1) */
	}
	/* ����I2C����ֹͣ�ź� */
	IIC_Stop();
	NVIC_EnableIRQ(TIM1_UP_IRQn);
	return data;	/* ִ�гɹ� ����dataֵ */

cmd_fail: /* ����ִ��ʧ�ܺ��мǷ���ֹͣ�źţ�����Ӱ��I2C�����������豸 */
	/* ����I2C����ֹͣ�ź� */
	IIC_Stop();
	NVIC_EnableIRQ(TIM1_UP_IRQn);
	return 0;
}


void max30102_FIFO_ReadWords(uint8_t Register_Address,uint16_t Word_Data[][2],uint8_t count)
{
	uint8_t i=0;
	uint8_t no = count;
	uint8_t data1, data2;
	/* ��1��������I2C���������ź� */
	NVIC_DisableIRQ(TIM1_UP_IRQn);
	IIC_Start();

	/* ��2������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
	IIC_Send_Byte(max30102_WR_address | I2C_WR);	/* �˴���дָ�� */

	/* ��3��������ACK */
	if (IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}

	/* ��4���������ֽڵ�ַ�� */
	IIC_Send_Byte((uint8_t)Register_Address);
	if (IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}
	

	/* ��6������������I2C���ߡ����濪ʼ��ȡ���� */
	IIC_Start();

	/* ��7������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
	IIC_Send_Byte(max30102_WR_address | I2C_RD);	/* �˴��Ƕ�ָ�� */

	/* ��8��������ACK */
	if (IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}

	/* ��9������ȡ���� */
	while (no)
	{
		data1 = IIC_Read_Byte(0);	
		IIC_Ack();
		data2 = IIC_Read_Byte(0);
		IIC_Ack();
		Word_Data[i][0] = (((uint16_t)data1 << 8) | data2);  //

		
		data1 = IIC_Read_Byte(0);	
		IIC_Ack();
		data2 = IIC_Read_Byte(0);
		if(1==no)
			IIC_NAck();	/* ���1���ֽڶ����CPU����NACK�ź�(����SDA = 1) */
		else
			IIC_Ack();
		Word_Data[i][1] = (((uint16_t)data1 << 8) | data2); 

		no--;	
		i++;
	}
	/* ����I2C����ֹͣ�ź� */
	IIC_Stop();
	NVIC_EnableIRQ(TIM1_UP_IRQn);

cmd_fail: /* ����ִ��ʧ�ܺ��мǷ���ֹͣ�źţ�����Ӱ��I2C�����������豸 */
	/* ����I2C����ֹͣ�ź� */
	IIC_Stop();
	NVIC_EnableIRQ(TIM1_UP_IRQn);
}

void max30102_FIFO_ReadBytes(uint8_t Register_Address,uint8_t* Data)
{	
	NVIC_DisableIRQ(TIM1_UP_IRQn);
	max30102_Bus_Read(REG_INTR_STATUS_1);
	max30102_Bus_Read(REG_INTR_STATUS_2);
	
	/* ��1��������I2C���������ź� */
	IIC_Start();

	/* ��2������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
	IIC_Send_Byte(max30102_WR_address | I2C_WR);	/* �˴���дָ�� */

	/* ��3��������ACK */
	if (IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}

	/* ��4���������ֽڵ�ַ�� */
	IIC_Send_Byte((uint8_t)Register_Address);
	if (IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}
	

	/* ��6������������I2C���ߡ����濪ʼ��ȡ���� */
	IIC_Start();

	/* ��7������������ֽڣ���7bit�ǵ�ַ��bit0�Ƕ�д����λ��0��ʾд��1��ʾ�� */
	IIC_Send_Byte(max30102_WR_address | I2C_RD);	/* �˴��Ƕ�ָ�� */

	/* ��8��������ACK */
	if (IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM������Ӧ�� */
	}

	/* ��9������ȡ���� */
	Data[0] = IIC_Read_Byte(1);	
	Data[1] = IIC_Read_Byte(1);	
	Data[2] = IIC_Read_Byte(1);	
	Data[3] = IIC_Read_Byte(1);
	Data[4] = IIC_Read_Byte(1);	
	Data[5] = IIC_Read_Byte(0);
	/* ���1���ֽڶ����CPU����NACK�ź�(����SDA = 1) */
	/* ����I2C����ֹͣ�ź� */
	IIC_Stop();
	NVIC_EnableIRQ(TIM1_UP_IRQn);

cmd_fail: /* ����ִ��ʧ�ܺ��мǷ���ֹͣ�źţ�����Ӱ��I2C�����������豸 */
	/* ����I2C����ֹͣ�ź� */
	IIC_Stop();
	NVIC_EnableIRQ(TIM1_UP_IRQn);

//	uint8_t i;
//	uint8_t fifo_wr_ptr;
//	uint8_t firo_rd_ptr;
//	uint8_t number_tp_read;
//	//Get the FIFO_WR_PTR
//	fifo_wr_ptr = max30102_Bus_Read(REG_FIFO_WR_PTR);
//	//Get the FIFO_RD_PTR
//	firo_rd_ptr = max30102_Bus_Read(REG_FIFO_RD_PTR);
//	
//	number_tp_read = fifo_wr_ptr - firo_rd_ptr;
//	
//	//for(i=0;i<number_tp_read;i++){
//	if(number_tp_read>0){
//		IIC_ReadBytes(max30102_WR_address,REG_FIFO_DATA,Data,6);
//	}
	
	//max30102_Bus_Write(REG_FIFO_RD_PTR,fifo_wr_ptr);
}

void max30102_init(void)
{
//	GPIO_InitTypeDef GPIO_InitStructure;

// 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);	
//	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_9;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
// 	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
//	IIC_Init();
	
	max30102_reset();
	
//	max30102_Bus_Write(REG_MODE_CONFIG, 0x0b);  //mode configuration : temp_en[3]      MODE[2:0]=010 HR only enabled    011 SP02 enabled
//	max30102_Bus_Write(REG_INTR_STATUS_2, 0xF0); //open all of interrupt
//	max30102_Bus_Write(REG_INTR_STATUS_1, 0x00); //all interrupt clear
//	max30102_Bus_Write(REG_INTR_ENABLE_2, 0x02); //DIE_TEMP_RDY_EN
//	max30102_Bus_Write(REG_TEMP_CONFIG, 0x01); //SET   TEMP_EN

//	max30102_Bus_Write(REG_SPO2_CONFIG, 0x47); //SPO2_SR[4:2]=001  100 per second    LED_PW[1:0]=11  16BITS

//	max30102_Bus_Write(REG_LED1_PA, 0x47); 
//	max30102_Bus_Write(REG_LED2_PA, 0x47); 
	
	
	
	max30102_Bus_Write(REG_INTR_ENABLE_1,0xc0);	// INTR setting
	max30102_Bus_Write(REG_INTR_ENABLE_2,0x00);
	max30102_Bus_Write(REG_FIFO_WR_PTR,0x00);  	//FIFO_WR_PTR[4:0]
	max30102_Bus_Write(REG_OVF_COUNTER,0x00);  	//OVF_COUNTER[4:0]
	max30102_Bus_Write(REG_FIFO_RD_PTR,0x00);  	//FIFO_RD_PTR[4:0]
	max30102_Bus_Write(REG_FIFO_CONFIG,0x0f);  	//sample avg = 1, fifo rollover=false, fifo almost full = 17
	max30102_Bus_Write(REG_MODE_CONFIG,0x03);  	//0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED
	max30102_Bus_Write(REG_SPO2_CONFIG,0x27);  	// SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (400uS)  
	max30102_Bus_Write(REG_LED1_PA,0x24);   	//Choose value for ~ 7mA for LED1
	max30102_Bus_Write(REG_LED2_PA,0x24);   	// Choose value for ~ 7mA for LED2
	max30102_Bus_Write(REG_PILOT_PA,0x7f);   	// Choose value for ~ 25mA for Pilot LED


	
//	// Interrupt Enable 1 Register. Set PPG_RDY_EN (data available in FIFO)
//	max30102_Bus_Write(0x2, 1<<6);

//	// FIFO configuration register
//	// SMP_AVE: 16 samples averaged per FIFO sample
//	// FIFO_ROLLOVER_EN=1
//	//max30102_Bus_Write(0x8,  1<<4);
//	max30102_Bus_Write(0x8, (0<<5) | 1<<4);

//	// Mode Configuration Register
//	// SPO2 mode
//	max30102_Bus_Write(0x9, 3);

//	// SPO2 Configuration Register
//	max30102_Bus_Write(0xa,
//			(3<<5)  // SPO2_ADC_RGE 2 = full scale 8192 nA (LSB size 31.25pA); 3 = 16384nA
//			| (1<<2) // sample rate: 0 = 50sps; 1 = 100sps; 2 = 200sps
//			| (3<<0) // LED_PW 3 = 411��s, ADC resolution 18 bits
//	);

//	// LED1 (red) power (0 = 0mA; 255 = 50mA)
//	max30102_Bus_Write(0xc, 0xb0);

//	// LED (IR) power
//	max30102_Bus_Write(0xd, 0xa0);
											
}

void max30102_reset(void)
{
	max30102_Bus_Write(REG_MODE_CONFIG,0x40);
	max30102_Bus_Write(REG_MODE_CONFIG,0x40);
}






void maxim_max30102_write_reg(uint8_t uch_addr, uint8_t uch_data)
{
//  char ach_i2c_data[2];
//  ach_i2c_data[0]=uch_addr;
//  ach_i2c_data[1]=uch_data;
//	
//  IIC_WriteBytes(I2C_WRITE_ADDR, ach_i2c_data, 2);
	IIC_Write_One_Byte(I2C_WRITE_ADDR,uch_addr,uch_data);
}

void maxim_max30102_read_reg(uint8_t uch_addr, uint8_t *puch_data)
{
//  char ch_i2c_data;
//  ch_i2c_data=uch_addr;
//  IIC_WriteBytes(I2C_WRITE_ADDR, &ch_i2c_data, 1);
//	
//  i2c.read(I2C_READ_ADDR, &ch_i2c_data, 1);
//  
//   *puch_data=(uint8_t) ch_i2c_data;
	IIC_Read_One_Byte(I2C_WRITE_ADDR,uch_addr,puch_data);
}

void maxim_max30102_read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led)
{
	uint32_t un_temp;
	unsigned char uch_temp;
	char ach_i2c_data[6];
	*pun_red_led=0;
	*pun_ir_led=0;

  
  //read and clear status register
  maxim_max30102_read_reg(REG_INTR_STATUS_1, &uch_temp);
  maxim_max30102_read_reg(REG_INTR_STATUS_2, &uch_temp);
  
  IIC_ReadBytes(I2C_WRITE_ADDR,REG_FIFO_DATA,(uint8_t *)ach_i2c_data,6);
  
  un_temp=(unsigned char) ach_i2c_data[0];
  un_temp<<=16;
  *pun_red_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[1];
  un_temp<<=8;
  *pun_red_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[2];
  *pun_red_led+=un_temp;
  
  un_temp=(unsigned char) ach_i2c_data[3];
  un_temp<<=16;
  *pun_ir_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[4];
  un_temp<<=8;
  *pun_ir_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[5];
  *pun_ir_led+=un_temp;
  *pun_red_led&=0x03FFFF;  //Mask MSB [23:18]
  *pun_ir_led&=0x03FFFF;  //Mask MSB [23:18]
}


// Ѫ�����
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

int32_t HeartRate_Value = 0;
int32_t SP02_Value = 0;
void dis_DrawCurve(uint32_t* data,uint8_t x, uint16_t color);

void Max30102_Measure(void)
{
	HAL_TIM_Base_Stop_IT(&htim1);
	
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
			HeartRate_Value = n_heart_rate;
			SP02_Value = n_sp02;
				printf("B%i\r\n", n_heart_rate); 
				//printf("HRvalid=%i, ", ch_hr_valid);
				printf("S%i\r\n", n_sp02);
				//printf("SPO2Valid=%i\r\n", ch_spo2_valid);
		}
        maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
		
		//��ʾˢ��
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
	
//	////////////////////////////////////
//	uint32_t un_prev_data;  
//	int i;
//	int32_t n_brightness;
//	float f_temp;
////	uint8_t temp_num=0;
//	uint8_t temp[6];
////	uint8_t str[100];
////	uint8_t dis_hr=0,dis_spo2=0;
////	uint32_t draw_red_max = 0;  // ��ʾ��
////	uint32_t draw_red_min = 0;
////	uint32_t draw_ir_max = 0;
////	uint32_t draw_ir_min = 0;
//	
//	max30102_init();
//	red_min=0x3FFFF;
//	red_max=0;
//	
//	n_ir_buffer_length=500; //buffer length of 100 stores 5 seconds of samples running at 100sps
//	
//	//read the first 500 samples, and determine the signal range
//    for(i=0;i<n_ir_buffer_length;i++)
//    {
//        while(HAL_GPIO_ReadPin(INT_GPIO_Port, INT_Pin)==1);   //wait until the interrupt pin asserts
//        
//		/*********************��ȡѪ��ԭʼ����****************************/
//		max30102_FIFO_ReadBytes(REG_FIFO_DATA, temp); // ��ȡ����
//		/*************************************************/
//		
//		aun_red_buffer[i] = (long)((long)((long)temp[0]&0x03)<<16) | (long)temp[1]<<8 | (long)temp[2];    // Combine values to get the actual number
//		aun_ir_buffer[i]  = (long)((long)((long)temp[3] & 0x03)<<16) |(long)temp[4]<<8 | (long)temp[5];   // Combine values to get the actual number
//            
//        if(red_min>aun_red_buffer[i])
//            red_min=aun_red_buffer[i];
//        if(red_max<aun_red_buffer[i])
//            red_max=aun_red_buffer[i];
//		
//		if(ir_min>aun_ir_buffer[i])
//            ir_min=aun_ir_buffer[i];
//        if(ir_max<aun_ir_buffer[i])
//            ir_max=aun_ir_buffer[i];
//		
////		draw_red_max = red_max;
////		draw_red_min = red_min;
////		draw_ir_max = ir_max;
////		draw_ir_min = ir_min;
//    }
//	un_prev_data=aun_red_buffer[i];
//	//calculate heart rate and SpO2 after first 500 samples (first 5 seconds of samples)
//	/*********************��Ѫ��ԭʼ�������ʵ�ʵ�Ѫ��Ũ�Ⱥ�����****************************/
//    maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid); 
//	/*************************************************/
//	
//	//////////////////////////////////////
////	while (1) // ��һ�λ���
//	{
//	/* USER CODE END WHILE */

//	/* USER CODE BEGIN 3 */
////	sprintf(str, "%3d %3d", CTP.ctpxy.ctp_x, CTP.ctpxy.ctp_y);
////	LCD_ShowStr(0, 1, WHITE, BLACK, str, 16);

//		i=0;
//        red_min=0x3FFFF;
//        red_max=0;
//		
//		//dumping the first 100 sets of samples in the memory and shift the last 400 sets of samples to the top
//        for(i=100;i<500;i++)
//        {
//            aun_red_buffer[i-100]=aun_red_buffer[i];
//            aun_ir_buffer[i-100]=aun_ir_buffer[i];
//            

//            if(red_min>aun_red_buffer[i])
//				red_min=aun_red_buffer[i];
//            if(red_max<aun_red_buffer[i])
//				red_max=aun_red_buffer[i];
//			

//        }
//		//take 100 sets of samples before calculating the heart rate.
//        for(i=400;i<500;i++)
//        {
//            un_prev_data=aun_red_buffer[i-1];
//            while(HAL_GPIO_ReadPin(INT_GPIO_Port, INT_Pin)==1);
//			
//            max30102_FIFO_ReadBytes(REG_FIFO_DATA, temp);
//			aun_red_buffer[i] =  (long)((long)((long)temp[0]&0x03)<<16) | (long)temp[1]<<8 | (long)temp[2];
//			aun_ir_buffer[i] = (long)((long)((long)temp[3] & 0x03)<<16) |(long)temp[4]<<8 | (long)temp[5];
//       
//			
//            if(aun_red_buffer[i]>un_prev_data)
//            {
//                f_temp=aun_red_buffer[i]-un_prev_data;
//                f_temp/=(red_max-red_min);
//                f_temp*=MAX_BRIGHTNESS;
//                n_brightness-=(int)f_temp;
//                if(n_brightness<0)
//                    n_brightness=0;
//            }
//            else
//            {
//                f_temp=un_prev_data-aun_red_buffer[i];
//                f_temp/=(red_max-red_min);
//                f_temp*=MAX_BRIGHTNESS;
//                n_brightness+=(int)f_temp;
//                if(n_brightness>MAX_BRIGHTNESS)
//                    n_brightness=MAX_BRIGHTNESS;
//            }
//			//send samples and calculation result to terminal program through UART
////			if(ch_hr_valid == 1 && ch_spo2_valid ==1 && n_heart_rate<120 && n_sp02<110)
////			{
////				dis_hr = n_heart_rate;
////				dis_spo2 = n_sp02;
////			}
////			else
////			{
////				dis_hr = 0;
////				dis_spo2 = n_sp02;
////			}
//			printf("%6d  %6d\r\n", aun_red_buffer[i], aun_ir_buffer[i]); 
//		}
//        maxim_heart_rate_and_oxygen_saturation(aun_ir_buffer, n_ir_buffer_length, aun_red_buffer, &n_sp02, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid);
//		printf("B%4d  S%4d\r\n", n_heart_rate, n_sp02); 
//		
//		dis_DrawCurve(aun_red_buffer, 20, RED);
//		dis_DrawCurve(aun_ir_buffer,   0, BLACK);
//	}

	HAL_TIM_Base_Start_IT(&htim1);
}

void dis_DrawCurve(uint32_t* data,uint8_t x, uint16_t color)
{
	uint16_t i;
	uint32_t max=0, min=262144;
	uint32_t temp;
//	uint32_t compress;
	
	for(i=20;i<500;i++)
	{
		if(data[i]>max)
		{
			max = data[i];
		}
		if(data[i]<min)
		{
			min = data[i];
		}
	}
	
	for(i=10;i<250;i++)
	{
		temp  = (data[i*2] + data[i*2+1])/2 - min;
		temp /= (max-min)/30;
		if(temp>60)
			temp=60;
		LCD_DrawPoint(i, 83-x-temp, color);
	}
}

#include "iic.h"
#include "tim.h"


void delay_us(uint16_t us)
{
	uint16_t differ=0xffff-us-5;					//�趨��ʱ����������ʼֵ

	__HAL_TIM_SET_COUNTER(&htim3,differ);

	HAL_TIM_Base_Start(&htim3);					//������ʱ��

  while(differ<0xffff-6)							//�������ж�

  {

    differ=__HAL_TIM_GET_COUNTER(&htim3);			//��ѯ�������ļ���ֵ

  }

  HAL_TIM_Base_Stop(&htim3);
}

void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SDA_H();	  	  
	IIC_SCL_H();
	delay_us(4);
 	IIC_SDA_L();//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL_L();//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL_L();
	IIC_SDA_L();//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	IIC_SCL_H(); 
	IIC_SDA_H();//����I2C���߽����ź�
	delay_us(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	IIC_SDA_H();delay_us(1);	   
	IIC_SCL_H();delay_us(1);	 
	while(READ_SDA())
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL_L();//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void IIC_Ack(void)
{
	IIC_SCL_L();
	SDA_OUT();
	IIC_SDA_L();
	delay_us(2);
	IIC_SCL_H();
	delay_us(2);
	IIC_SCL_L();
}
//������ACKӦ��		    
void IIC_NAck(void)
{
	IIC_SCL_L();
	SDA_OUT();
	IIC_SDA_H();
	delay_us(2);
	IIC_SCL_H();
	delay_us(2);
	IIC_SCL_L();
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	SDA_OUT(); 	    
    IIC_SCL_L();//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {              
        //IIC_SDA=(txd&0x80)>>7;
		if((txd&0x80)>>7)
			IIC_SDA_H();
		else
			IIC_SDA_L();
		txd<<=1; 	  
		delay_us(2);   //��TEA5767��������ʱ���Ǳ����
		IIC_SCL_H();
		delay_us(2); 
		IIC_SCL_L();	
		delay_us(2);

    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
    for(i=0;i<8;i++ )
	{
        IIC_SCL_L(); 
        delay_us(2);
		IIC_SCL_H();
        receive<<=1;
        if(READ_SDA())
			receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}



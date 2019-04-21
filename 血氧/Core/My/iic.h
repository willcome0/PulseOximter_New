#ifndef __IIC_H
#define __IIC_H


#include "main.h"
#include "stm32f1xx_hal.h"

#define IIC_SDA_H()	SDA_GPIO_Port->BSRR = SDA_Pin
#define IIC_SDA_L()	SDA_GPIO_Port->BSRR = (uint32_t)SDA_Pin << 16U
#define IIC_SCL_H()	SCL_GPIO_Port->BSRR = SCL_Pin
#define IIC_SCL_L()	SCL_GPIO_Port->BSRR = (uint32_t)SCL_Pin << 16U

#define READ_SDA()	HAL_GPIO_ReadPin(SDA_GPIO_Port, SDA_Pin)
#define SDA_IN()	SDA_GPIO_Port->CRH&=0XFFFFFFF0,SDA_GPIO_Port->CRH|=0x00000008;
#define SDA_OUT()	SDA_GPIO_Port->CRH&=0XFFFFFFF0,SDA_GPIO_Port->CRH|=0x00000003;



void delay_us(uint16_t us);
void IIC_Start(void);
void IIC_Stop(void);
uint8_t IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);
void IIC_Send_Byte(uint8_t txd);
uint8_t IIC_Read_Byte(unsigned char ack);





#endif

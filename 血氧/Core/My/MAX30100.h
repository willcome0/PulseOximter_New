#ifndef __MAX30100_H
#define __MAX30100_H

#include "main.h"
#include "stm32f1xx_hal.h"

#define MAX30100_ADDR 0xAE

//register addresses
#define REG_INTR_STATUS_1 0x00
#define REG_INTR_STATUS_2 0x01
#define REG_INTR_ENABLE_1 0x02
#define REG_INTR_ENABLE_2 0x03
#define REG_FIFO_WR_PTR 0x04
#define REG_OVF_COUNTER 0x05
#define REG_FIFO_RD_PTR 0x06
#define REG_FIFO_DATA 0x07
#define REG_FIFO_CONFIG 0x08
#define REG_MODE_CONFIG 0x09
#define REG_SPO2_CONFIG 0x0A
#define REG_LED1_PA 0x0C
#define REG_LED2_PA 0x0D
#define REG_PILOT_PA 0x10
#define REG_MULTI_LED_CTRL1 0x11
#define REG_MULTI_LED_CTRL2 0x12
#define REG_TEMP_INTR 0x1F
#define REG_TEMP_FRAC 0x20
#define REG_TEMP_CONFIG 0x21
#define REG_PROX_INT_THRESH 0x30
#define REG_REV_ID 0xFE
#define REG_PART_ID 0xFF

#define INTERRUPT_REG  					0X00
#define INTERRUPT_REG_A_FULL  			(0X01<<7)
#define INTERRUPT_REG_TEMP_RDY  		(0X01<<6)
#define INTERRUPT_REG_HR_RDY  			(0X01<<5)
#define INTERRUPT_REG_SPO2_RDY  		(0X01<<4)
#define INTERRUPT_REG_PWR_RDY  			(0X01<<0)

#define I2C_WR	0		/* 写控制bit */
#define I2C_RD	1		/* 读控制bit */

#define PI 3.1415926535897932384626433832795028841971               //定义圆周率值
#define FFT_N 1024                                                  //定义福利叶变换的点数
struct compx     //定义一个复数结构
{
	float real;
	float imag;
};


extern struct compx s1[FFT_N+16];           //FFT输入和输出：从S[1]开始存放，根据大小自己定义
extern struct compx s2[FFT_N+16];           //FFT输入和输出：从S[1]开始存放，根据大小自己定义

uint8_t Max10300_Init(void);
uint8_t Max10300_Reset(void);

uint8_t maxim_max30102_read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led);

uint8_t Max30100_WriteByte(int saddress,int w_data);
uint8_t Max30100_ReadByte(int saddress);
void Max10300_ReadFIFO(uint8_t Register_Address, char Word_Data[], uint8_t count);

void FFT(struct compx *xin);
uint16_t find_max_num_index(struct compx *data, uint16_t count);
#endif

#ifndef __FT6206_H
#define __FT6206_H

#include "main.h"
#include "stm32f1xx_hal.h"

#define     FT6206_ADDR         0x70


typedef  enum{
    NO_TOUCH   = 0,    // ������û�а���
    JUST_TOUCH    ,    // �������հ���
    TOUCHED       ,    // �������Ѿ���������
    JUST_POP      ,    // �������յ���
}TouchMainStatus;

typedef  enum{  
    Y_NO_MOVING=0 ,
    UP_MOVING     ,   // �������Ѿ����£����������ƶ�
    DOWN_MOVING   ,   // �������Ѿ����£����������ƶ�
}YSubStatus;

typedef  enum{ 
    X_NO_MOVING=0 ,
    LEFT_MOVING   ,   // �������Ѿ����£����������ƶ�
    RIGHT_MOVING  ,   // �������Ѿ����£����������ƶ�
}XSubStatus;

typedef struct 
{
    uint16_t ctp_x;                  // ��������X����ֵ
    uint16_t ctp_y;                  // ��������Y����ֵ
}struCTPxy;

typedef  struct
{
    struCTPxy       ctpxy;           // ������������XY
    TouchMainStatus ctpmainstatus;   // ����������Ҫ״̬
    XSubStatus      xmove;           // X������ƶ�״̬
    YSubStatus      ymove;           // Y������ƶ�״̬
    int16_t         dx;              // X�����ƶ������� +ֵ:��ʾ�����ƶ�; -ֵ:��ʾ�����ƶ�
    int16_t         dy;              // Y�����ƶ������� +ֵ:��ʾ�����ƶ�; -ֵ:��ʾ�����ƶ�
}struTouch;


extern struTouch CTP;

uint8_t FT6206_Read_Reg(uint8_t *pbuf,uint32_t len);
void  ReadCTP(struTouch *nowctp);
void  Get_Test_Val(struTouch  nowctp, uint16_t *val_x, uint16_t *val_y);







#endif

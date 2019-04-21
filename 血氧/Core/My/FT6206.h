#ifndef __FT6206_H
#define __FT6206_H

#include "main.h"
#include "stm32f1xx_hal.h"

#define     FT6206_ADDR         0x70


typedef  enum{
    NO_TOUCH   = 0,    // 触摸屏没有按下
    JUST_TOUCH    ,    // 触摸屏刚按下
    TOUCHED       ,    // 触摸屏已经触摸到了
    JUST_POP      ,    // 触摸屏刚弹出
}TouchMainStatus;

typedef  enum{  
    Y_NO_MOVING=0 ,
    UP_MOVING     ,   // 触摸屏已经按下，并且向上移动
    DOWN_MOVING   ,   // 触摸屏已经按下，并且向下移动
}YSubStatus;

typedef  enum{ 
    X_NO_MOVING=0 ,
    LEFT_MOVING   ,   // 触摸屏已经按下，并且向左移动
    RIGHT_MOVING  ,   // 触摸屏已经按下，并且向右移动
}XSubStatus;

typedef struct 
{
    uint16_t ctp_x;                  // 触摸屏的X坐标值
    uint16_t ctp_y;                  // 触摸屏的Y坐标值
}struCTPxy;

typedef  struct
{
    struCTPxy       ctpxy;           // 触摸屏的坐标XY
    TouchMainStatus ctpmainstatus;   // 触摸屏的主要状态
    XSubStatus      xmove;           // X方向的移动状态
    YSubStatus      ymove;           // Y方向的移动状态
    int16_t         dx;              // X方向移动的像素 +值:表示向右移动; -值:表示向左移动
    int16_t         dy;              // Y方向移动的像素 +值:表示向下移动; -值:表示向上移动
}struTouch;


extern struTouch CTP;

uint8_t FT6206_Read_Reg(uint8_t *pbuf,uint32_t len);
void  ReadCTP(struTouch *nowctp);
void  Get_Test_Val(struTouch  nowctp, uint16_t *val_x, uint16_t *val_y);







#endif

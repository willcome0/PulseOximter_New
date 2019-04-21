#include "FT6206.h"
#include "myiic.h"


struTouch CTP;

uint8_t FT6206_Read_Reg(uint8_t *pbuf,uint32_t len)
{
    int8_t i=0;

    IIC_Start();
    IIC_Send_Byte(FT6206_ADDR);
    IIC_Wait_Ack(); 
    
    IIC_Send_Byte(0x00);
    IIC_Wait_Ack(); 
    IIC_Stop();
  
    IIC_Start();
    IIC_Send_Byte(FT6206_ADDR+1);
    IIC_Wait_Ack(); 
    
    for(i=0;i<len;i++)
    {
        if(i==(len-1))  *(pbuf+i)=IIC_Read_Byte(0);
        else            *(pbuf+i)=IIC_Read_Byte(1);
    }   
    IIC_Stop();
    
    return 0;
}
#define X_MAX_PIXEL         240
#define Y_MAX_PIXEL         320
//static struTouch  ctplast={0,0,NO_TOUCH,X_NO_MOVING,Y_NO_MOVING,};
void  ReadCTP(struTouch *nowctp)
{
    uint8_t    ctpbuf[10];

//    ctplast.ctpxy.ctp_x   = nowctp->ctpxy.ctp_x;
//    ctplast.ctpxy.ctp_y   = nowctp->ctpxy.ctp_y;
//    ctplast.ctpmainstatus = nowctp->ctpmainstatus;
//    ctplast.dx            = nowctp->dx;
//    ctplast.dy            = nowctp->dy;

    // ��ȡ���ڵ�����ֵ
    FT6206_Read_Reg(ctpbuf, 7);

    if ((ctpbuf[2]&0x0f) == 1)
    {
        //����������λ480*800
        nowctp->ctpxy.ctp_x   = (ctpbuf[3] & 0x0f);
        nowctp->ctpxy.ctp_x <<= 8;
        nowctp->ctpxy.ctp_x  += ctpbuf[4];
        if(nowctp->ctpxy.ctp_x < X_MAX_PIXEL ) 
//			nowctp->ctpxy.ctp_x = X_MAX_PIXEL - nowctp->ctpxy.ctp_x - 1; 
			nowctp->ctpxy.ctp_x = nowctp->ctpxy.ctp_x; 
        else
			nowctp->ctpxy.ctp_x = 0;

        nowctp->ctpxy.ctp_y   = (ctpbuf[5] & 0x0f);
        nowctp->ctpxy.ctp_y <<= 8;
        nowctp->ctpxy.ctp_y  += ctpbuf[6];
        if(nowctp->ctpxy.ctp_y < Y_MAX_PIXEL )
//			nowctp->ctpxy.ctp_y = Y_MAX_PIXEL - nowctp->ctpxy.ctp_y - 1;
			nowctp->ctpxy.ctp_y = nowctp->ctpxy.ctp_y;
        else
			nowctp->ctpxy.ctp_y = 0;
    }
    else
    {
        nowctp->ctpxy.ctp_x = 999;
        nowctp->ctpxy.ctp_y = 999;
    }
//    
//    // ������һ�ε�״̬�����ڵ�����ֵ�ж����ڵ���Ҫ״̬
//    if(ctplast.ctpmainstatus == NO_TOUCH)        // ��һ��û�д���
//    {
//        if( (nowctp->ctpxy.ctp_x == 0xFFF) && (nowctp->ctpxy.ctp_y == 0xFFF)) { nowctp->ctpmainstatus = NO_TOUCH;  }// ����Ҳû�д��� 
//        else                                                                  { nowctp->ctpmainstatus = JUST_TOUCH;}// ���ڸմ���  
//    }
//    else if((ctplast.ctpmainstatus == JUST_TOUCH) || (ctplast.ctpmainstatus == TOUCHED))   // ��һ�θմ��� �� �Ѿ���������
//    {
//        if( (nowctp->ctpxy.ctp_x == 0xFFF) && (nowctp->ctpxy.ctp_y == 0xFFF)) { nowctp->ctpmainstatus = JUST_POP;}  // �յ��� 
//        else                                                                  { nowctp->ctpmainstatus = TOUCHED; }  // �Ѿ����� 
//    }
//    else if(ctplast.ctpmainstatus == JUST_POP)
//    {
//        if( (nowctp->ctpxy.ctp_x == 0xFFF) && (nowctp->ctpxy.ctp_y == 0xFFF)) { nowctp->ctpmainstatus = NO_TOUCH;  }// û�а���  
//        else                                                                  { nowctp->ctpmainstatus = JUST_TOUCH;}// 
//    }
//    
//    // �����ڵ���Ҫ״̬���ڰ���ʱ�����ж��ƶ����
//    if(nowctp->ctpmainstatus == TOUCHED)
//    {
//        nowctp->dx = (int16_t)nowctp->ctpxy.ctp_x - (int16_t)ctplast.ctpxy.ctp_x;
//        if(nowctp->dx == 0)     { nowctp->xmove = X_NO_MOVING;  }
//        else if(nowctp->dx > 0) { nowctp->xmove = RIGHT_MOVING; }
//        else if(nowctp->dx < 0) { nowctp->xmove = LEFT_MOVING;  }
//        
//        nowctp->dy = (int16_t)nowctp->ctpxy.ctp_y - (int16_t)ctplast.ctpxy.ctp_y;
//        if(nowctp->dy == 0)     { nowctp->ymove = Y_NO_MOVING;  }
//        else if(nowctp->dy > 0) { nowctp->ymove = DOWN_MOVING;  }
//        else if(nowctp->dy < 0) { nowctp->ymove = UP_MOVING;    }
//    }
//    else
//    {
//        nowctp->xmove = X_NO_MOVING;
//        nowctp->ymove = Y_NO_MOVING;
//        nowctp->dx = 0;
//        nowctp->dy = 0;
//    }
}
void  Get_Test_Val(struTouch  nowctp, uint16_t *val_x, uint16_t *val_y)
{
    int    tmp;

    if(nowctp.ctpmainstatus == TOUCHED)
    {    
        tmp  = (int)(*val_x);
        tmp += nowctp.dx;          // �����ƶ�����/�ƶ���ֵ/ ; �����ƶ�,��/�ƶ���ֵ/
        if(tmp < 0) tmp = 65535;
        if(tmp > 65535) tmp = 0;
        *val_x = (uint16_t)tmp;
        
        tmp  = (int)(*val_y);
        tmp -= nowctp.dy;          // �����ƶ�����/�ƶ���ֵ/ ; �����ƶ�,��/�ƶ���ֵ/
        if(tmp < 0) tmp = 65535;
        if(tmp > 65535) tmp = 0;
        *val_y = (uint16_t)tmp;
    }
}
void Touch_Test(struTouch  nowctp)
{
	if(nowctp.ctpmainstatus == JUST_POP)
    {    
        
    }
}

typedef  enum{
    FALSE = 0,
    TRUE  = !FALSE,
}BOOL;

BOOL is_touch_area(struTouch  nowctp,uint16_t x,uint16_t y,uint16_t w,uint16_t h,uint16_t margin)
{
	return (BOOL)(nowctp.ctpmainstatus == TOUCHED 
		&& nowctp.ctpxy.ctp_x >= x - margin 
		&& nowctp.ctpxy.ctp_x <= x + w + margin 
		&& nowctp.ctpxy.ctp_y >= y - margin 
		&& nowctp.ctpxy.ctp_y <= y + h + margin);
}

#include "LCD.h"
#include "font.h"
#include <stdlib.h>

struct  
{										    
	uint16_t Width;			//x轴像素
	uint16_t Height;		//y轴像素
	uint16_t ID;			//LCD ID
	uint8_t  Dir;			//屏幕方向
	uint16_t W_GRAMCmd;		//写GRAM指令
	uint16_t SetXCmd;		//设置x轴指令
	uint16_t SetYCmd;		//设置y轴指令 
}Param;

#define WriteGRAM_Prepare()	LCD_Write_Para(CMD, Param.W_GRAMCmd)

void LCD_Write_Para(enum DataType type, uint16_t para)
{
	Write_CS_Pin(L);
	Write_RD_Pin(H);
	Write_RS_Pin((enum LCD_PinState)type);
	
	Write_Data_Pin(para);
	
	Write_RW_Pin(L);
	Write_RW_Pin(H);
	Write_CS_Pin(H);
}

void LCD_Write_Reg(uint16_t reg, uint16_t value)
{
	LCD_Write_Para(CMD,  reg);
	LCD_Write_Para(DATA, value);
}

void LCD_Reset(void)
{
	Write_RST_Pin(H);
	HAL_Delay(1);
	Write_RST_Pin(L);
	HAL_Delay(10);
	Write_RST_Pin(H);
	HAL_Delay(120);
}

void LCD_Init(void)
{
	LCD_Reset();
	
	LCD_Write_Para(CMD, 0x11);
	HAL_Delay(120);
	
	/*
		
		4K-Colors   0x03
		65K-Colors  0x05
		256K-Colors  0x06
	*/
	LCD_Write_Para(CMD,  0x3A);
	LCD_Write_Para(DATA, 0x05);
	LCD_Write_Para(CMD,  0x36);
	LCD_Write_Para(DATA, 0x00);
	
	//--------------------------------ST7789S Frame rate setting----------------------------------//
	LCD_Write_Para(CMD,  0xB2);
	LCD_Write_Para(DATA, 0x00);
	LCD_Write_Para(DATA, 0x00);
	LCD_Write_Para(DATA, 0x00);
	LCD_Write_Para(DATA, 0x33);
	LCD_Write_Para(DATA, 0x33);

	LCD_Write_Para(CMD,  0xB7);
	LCD_Write_Para(DATA, 0x35);
	//---------------------------------ST7789S Power setting--------------------------------------//
	LCD_Write_Para(CMD,  0xB8);
	LCD_Write_Para(DATA, 0x2F);
	LCD_Write_Para(DATA, 0x2B);
	LCD_Write_Para(DATA, 0x2F);

	LCD_Write_Para(CMD,  0xBB);
	LCD_Write_Para(DATA, 0x24);//vcom

	LCD_Write_Para(CMD,  0xC0);
	LCD_Write_Para(DATA, 0x2C);

	LCD_Write_Para(CMD,  0xC3);
	LCD_Write_Para(DATA, 0x10);//0B调深浅

	LCD_Write_Para(CMD,  0xC4);
	LCD_Write_Para(DATA, 0x20);

	LCD_Write_Para(CMD,  0xC6);
	LCD_Write_Para(DATA, 0x11);

	LCD_Write_Para(CMD,  0xD0);
	LCD_Write_Para(DATA, 0xA4);
	LCD_Write_Para(DATA, 0xA1);

	LCD_Write_Para(CMD,  0xE8);
	LCD_Write_Para(DATA, 0x03);

	LCD_Write_Para(CMD,  0xE9);
	LCD_Write_Para(DATA, 0x0D);
	LCD_Write_Para(DATA, 0x12);
	LCD_Write_Para(DATA, 0x00);
	//--------------------------------ST7789S gamma setting---------------------------------------//
	LCD_Write_Para(CMD,  0xE0);
	LCD_Write_Para(DATA, 0xD0);
	LCD_Write_Para(DATA, 0x06);
	LCD_Write_Para(DATA, 0x0B);
	LCD_Write_Para(DATA, 0x0A);
	LCD_Write_Para(DATA, 0x09);
	LCD_Write_Para(DATA, 0x05);
	LCD_Write_Para(DATA, 0x2E);
	LCD_Write_Para(DATA, 0x43);
	LCD_Write_Para(DATA, 0x44);
	LCD_Write_Para(DATA, 0x09);
	LCD_Write_Para(DATA, 0x16);
	LCD_Write_Para(DATA, 0x15);
	LCD_Write_Para(DATA, 0x23);
	LCD_Write_Para(DATA, 0x27);

	LCD_Write_Para(CMD,  0xE1);
	LCD_Write_Para(DATA, 0xD0);
	LCD_Write_Para(DATA, 0x06);
	LCD_Write_Para(DATA, 0x0B);
	LCD_Write_Para(DATA, 0x09);
	LCD_Write_Para(DATA, 0x08);
	LCD_Write_Para(DATA, 0x06);
	LCD_Write_Para(DATA, 0x2E);
	LCD_Write_Para(DATA, 0x44);
	LCD_Write_Para(DATA, 0x44);
	LCD_Write_Para(DATA, 0x3A);
	LCD_Write_Para(DATA, 0x15);
	LCD_Write_Para(DATA, 0x15);
	LCD_Write_Para(DATA, 0x23);
	LCD_Write_Para(DATA, 0x26);

	LCD_Write_Para(CMD,  0x21); //反显

	LCD_Write_Para(CMD,  0x2A); //Frame rate control
	LCD_Write_Para(DATA, 0x00);
	LCD_Write_Para(DATA, 0x00);
	LCD_Write_Para(DATA, 0x00);
	LCD_Write_Para(DATA, 0xEF);

	LCD_Write_Para(CMD,  0x2B); //Display function control
	LCD_Write_Para(DATA, 0x00);
	LCD_Write_Para(DATA, 0x00);
	LCD_Write_Para(DATA, 0x01);
	LCD_Write_Para(DATA, 0x3F);

	LCD_Write_Para(CMD, 0x29); //display on
	LCD_Write_Para(CMD, 0x2c);
	
	LCD_SetPara();
	LCD_Clear(WHITE);
}

void LCD_SetWindows(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend)
{
    //set the X coordinates
    LCD_Write_Para(CMD, Param.SetXCmd);
    LCD_Write_Para(DATA, (Xstart >> 8) & 0xFF);
    LCD_Write_Para(DATA, Xstart & 0xFF);
    LCD_Write_Para(DATA, ((Xend) >> 8) & 0xFF);
    LCD_Write_Para(DATA, (Xend) & 0xFF);

    //set the Y coordinates
    LCD_Write_Para(CMD, Param.SetYCmd);
    LCD_Write_Para(DATA, (Ystart >> 8) & 0xFF);
    LCD_Write_Para(DATA, Ystart & 0xFF);
    LCD_Write_Para(DATA, ((Yend) >> 8) & 0xFF);
    LCD_Write_Para(DATA, (Yend) & 0xFF);

    WriteGRAM_Prepare();
}



void LCD_SetPara(void)
{
	Param.W_GRAMCmd = 0x2C;
	#if USE_HORIZONTAL == 1 // 使用横屏
		Param.Dir = 1;
		Param.SetXCmd = 0x2B;
		Param.SetYCmd = 0x2A;				 	 		
		Param.Width = 320;
		Param.Height = 240;
	#else
		Param.Dir = 0;
		Param.SetXCmd = 0x2A;
		Param.SetYCmd = 0x2B;				 	 		
		Param.Width = 240;
		Param.Height = 320;
	#endif
}

void LCD_Clear(uint16_t color)
{
	LCD_SetWindows(0, 0, Param.Width-1, Param.Height-1);
	for (uint32_t i=0; i<Param.Width*Param.Height; i++)
	{
		LCD_Write_Para(DATA, color);
	}
}

void LCD_Fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t color)
{
	uint16_t width  = ex-sx+1; 			//得到填充的宽度
	uint16_t height = ey-sy+1;			//得到填充的高度
	LCD_SetWindows(sx, sy, ex-1, ey-1);	//设置显示窗口
	for (uint16_t i=0; i<height; i++)
	{
		for (uint16_t j=0; j<width; j++)
		{
			LCD_Write_Para(DATA, color);
		}
	}
	LCD_SetWindows(0, 0, Param.Width-1, Param.Height-1);
}

void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color)
{
	LCD_SetWindows(x, y, x, y);
	LCD_Write_Para(DATA, color);
}

void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	int16_t xerr=0, yerr=0, delta_x, delta_y, distance; 
	int16_t incx, incy, uRow, uCol; 

	delta_x = x2 - x1;	//计算坐标增量 
	delta_y = y2 - y1; 
	uRow = x1;
	uCol = y1;
	
	// X轴方向状态
	if (delta_x > 0)
		incx = 1;		// 线向右
	else if (delta_x == 0)
		incx = 0;		// 垂直线 
	else
	{
		incx = -1;		// 线向左
		delta_x = -delta_x;
	}
	
	// Y轴方向状态
	if (delta_y > 0)
		incy = 1; 		// 线向下
	else if (delta_y == 0)
		incy = 0;		// 水平线 
	else
	{
		incy = -1;		// 线向上
		delta_y = -delta_y;
	}
	
	if( delta_x > delta_y)
		distance = delta_x; //选取基本增量坐标轴 
	else
		distance = delta_y;
	
	for(uint16_t t=0; t<=distance+1; t++)//画线输出 
	{  
		LCD_DrawPoint(uRow, uCol, color);//画点 
		xerr += delta_x; 
		yerr += delta_y; 
		if (xerr > distance) 
		{
			xerr -= distance; 
			uRow += incx; 
		} 
		if (yerr > distance) 
		{ 
			yerr -= distance; 
			uCol += incy; 
		}
	}
}

void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
{
	LCD_DrawLine(x1, y1, x2, y1, color);
	LCD_DrawLine(x1, y1, x1, y2, color);
	LCD_DrawLine(x1, y2, x2, y2, color);
	LCD_DrawLine(x2, y1, x2, y2, color);
}

void LCD_DrawCircle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color)
{
    int a, b;
    int di;
    a = 0;
    b = r;
    di = 3 - (r << 1);

    while(a <= b)
    {
        LCD_DrawPoint(x0 - b, y0 - a, color);
        LCD_DrawPoint(x0 + b, y0 - a, color);
        LCD_DrawPoint(x0 - a, y0 + b, color);
		
        LCD_DrawPoint(x0 - b, y0 - a, color);
        LCD_DrawPoint(x0 - a, y0 - b, color);
        LCD_DrawPoint(x0 + b, y0 + a, color);
		
        LCD_DrawPoint(x0 + a, y0 - b, color);
        LCD_DrawPoint(x0 + a, y0 + b, color);
        LCD_DrawPoint(x0 - b, y0 + a, color);
        a++;

        if(di < 0)
			di += 4 * a + 6;
        else
        {
            di += 10 + 4 * (a - b);
            b--;
        }

        LCD_DrawPoint(x0 + a, y0 + b, color);
    }
}

uint8_t CoverFlag = 1;
void LCD_ShowFontEN(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, char num, uint8_t size)
{
    uint8_t data;
	
	num = num - ' ';//得到偏移后的值
	LCD_SetWindows(x, y, x+size/2-1, y+size-1);//设置单个文字显示窗口

	uint8_t RowNum = size;
	uint8_t ColNum = size/2/8 + ((size/2)%8!=0?1:0);
	
	for (uint8_t j=0; j<RowNum; j++)
	{
		for (uint8_t i=0; i<ColNum; i++)
		{
            switch (size)
            {
                case 12:
					data = asc2_1206[num][j*ColNum + i];
                    break;
				
                case 16:
					data = asc2_1608[num][j*ColNum + i];
                    break;
				
                case 24:
					data = asc2_2412[num][j*ColNum + i];
					break;
				
                default:
                    return;
            }

			for (uint8_t k=0; k<((i==ColNum-1)?((size/2)%8!=0?(size/2)%8:8):8); k++)
            {
				if (CoverFlag)
				{
					if (data&0x01)
						LCD_Write_Para(DATA, fc);
					else
						LCD_Write_Para(DATA, bc);
					data >>= 1; 
				}
				else
				{
					if (data&0x01)
						LCD_DrawPoint(x+i*8+k, y+j, fc);
					data >>= 1; 
				}
            }
        }
    }
	LCD_SetWindows(0, 0, Param.Width-1, Param.Height-1);  	   	 	  
}

void LCD_ShowFontZH(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, char *s, uint8_t size)
{
	uint16_t HZnum = sizeof(tfont16)/sizeof(typFNT_GB16);	//自动统计汉字数目
	uint8_t data;
	uint8_t index[2];
	uint8_t RowNum = size;
	uint8_t ColNum = size/8 + (size%8!=0?1:0);
	
	for (uint16_t k=0; k<HZnum; k++) 
	{
		switch (size)
		{
			case 16:
				index[0] = tfont16[k].Index[0];
				index[1] = tfont16[k].Index[1];
				break;
			
			case 24:
				index[0] = tfont24[k].Index[0];
				index[1] = tfont24[k].Index[1];
				break;
			default:
				return;
		}
		if ((index[0]==*(s)) && (index[1]==*(s+1)))
		{
			LCD_SetWindows(x, y, x+size-1, y+size-1);
			
			for (uint8_t j=0; j<RowNum; j++)
			{
				for (uint8_t i=0; i<ColNum; i++)
				{
					switch (size)
					{
						case 16:
							data = tfont16[k].Msk[j*ColNum + i];
							break;
						case 24:
							data = tfont24[k].Msk[j*ColNum + i];
							break;
						
						default:
							return;
					}

					for (uint8_t a=0; a<((i==ColNum-1)?((size)%8!=0?(size)%8:8):8); a++)
					{
						if (CoverFlag)
						{
							if (data&0x01)
								LCD_Write_Para(DATA, fc);
							else
								LCD_Write_Para(DATA, bc);
							data >>= 1; 
						}
						else
						{
							if (data&0x01)
								LCD_DrawPoint(x+i*8+a, y+j, fc);
							data >>= 1; 
						}
					}
				}
			}
		}	  	
		continue;
	}
	LCD_SetWindows(0, 0, Param.Width-1, Param.Height-1); 
}

void LCD_ShowPic(uint16_t x, uint16_t y, uint16_t length, uint16_t width, uint16_t fc, uint16_t bc, const uint8_t *pic)
{
	uint8_t RowNum = width; // 行
	uint8_t ColNum = length/8 + (length%8!=0?1:0);
	uint8_t data;
	
	LCD_SetWindows(x, y, x+length-1, y+width-1);
	
	for (uint8_t j=0; j<RowNum; j++)
	{
		for (uint8_t i=0; i<ColNum; i++)
		{

			data = pic[j*ColNum + i];
			for (uint8_t a=0; a<((i==ColNum-1)?((length)%8!=0?(length)%8:8):8); a++)
			{
				if (CoverFlag)
				{
					if (data&0x01)
						LCD_Write_Para(DATA, fc);
					else
						LCD_Write_Para(DATA, bc);
					data >>= 1; 
				}
				else
				{
					if (data&0x01)
						LCD_DrawPoint(x+i*8+a, y+j, fc);
					data >>= 1; 
				}
			}
		}
	}
}

void LCD_ShowPic_Heart(uint8_t show_flag)
{
	if (show_flag)
		LCD_ShowPic(205, 30, 32, 32, RED, WHITE, heart);
	else
		LCD_Fill(205, 30, 237, 62, WHITE);
}

void LCD_ShowStr(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, char *str, uint8_t size)
{
  	uint8_t bHz=0;     //字符或者中文 

    while(*str!=0)//数据未结束
    {
        if (!bHz)
        {
			if (x>(Param.Width-size/2) || y>(Param.Height-size)) 
				return; 
	        if ((uint8_t)(*str)>0x80)	// 中文
				bHz=1;
	        else
	        {
				switch (size)
				{
					case 12: case 16: case 24:
						LCD_ShowFontEN(x, y, fc, bc, *str, size);	
						x += size/2;
					break;
				}
				str++; 
	        }
        }
		else//中文 
        {
			if (x>(Param.Width-size) || y>(Param.Height-size)) 
				return;
            bHz = 0;		//有汉字库 
			LCD_ShowFontZH(x, y, fc, bc, str, size);

	        str += 2;
	        x += size;		//下一个汉字偏移	    
        }						 
    }
}

void LCD_ShowBat(uint16_t x, uint16_t y, uint8_t state)
{
	LCD_DrawRectangle(x-2, y-2, x+25, y+10, WHITE);
	LCD_Fill(x-3,  y+1, x-2,  y+7, WHITE);
	switch (state)
	{
		case 5: LCD_Fill(x+5*4, y, x+5*4+4, y+9, WHITE);
		case 4: LCD_Fill(x+5*3, y, x+5*3+4, y+9, WHITE);
		case 3: LCD_Fill(x+5*2, y, x+5*2+4, y+9, WHITE);
		case 2: LCD_Fill(x+5*1, y, x+5*1+4, y+9, WHITE);
		case 1: LCD_Fill(x+5*0, y, x+5*0+4, y+9, WHITE);break;
		case 0: break;
	}
	switch (5-state)
	{
		case 5: LCD_Fill(x+5*0, y, x+5*0+4, y+9, BLACK);
		case 4: LCD_Fill(x+5*1, y, x+5*1+4, y+9, BLACK);
		case 3: LCD_Fill(x+5*2, y, x+5*2+4, y+9, BLACK);
		case 2: LCD_Fill(x+5*3, y, x+5*3+4, y+9, BLACK);
		case 1: LCD_Fill(x+5*4, y, x+5*4+4, y+9, BLACK);
		case 0: break;
	}
}

void arc_chabu_area1(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t fc)
{
	
	uint16_t x,y,xi,yi,color;
	uint8_t e;
	static int fi1 = 0;
//	color=POINT_COLOR;
//	POINT_COLOR=point_color;
	x = xi=x2;
	y = yi=y2;
	e = abs(x2-x1)+abs(y2-y1);
	while(e!=0)
	{
		if(fi1>=0)
		{
		 	x=xi-1;
		 	fi1=fi1+2*(x0-xi)+1;
			LCD_DrawLine(xi,yi,x,y,fc);
		 	xi=x;
		 	e--;
		} 
		else
		{
			y=yi-1;
			fi1=fi1+2*(y0-yi)+1;
			LCD_DrawLine(xi,yi,x,y,fc);
			yi=y;
			e--;
		}	
	}
}


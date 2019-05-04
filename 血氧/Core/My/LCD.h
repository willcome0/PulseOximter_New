#ifndef __LCD_H
#define __LCD_H

#include "main.h"
#include "stm32f1xx_hal.h"


enum LCD_PinState{L=0, H};
enum DataType{CMD=0, DATA};

#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE         	 0x001F  
#define BRED             0XF81F
#define GRED 			 0XFFE0
#define GBLUE			 0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			 0XBC40 //��ɫ
#define BRRED 			 0XFC07 //�غ�ɫ
#define GRAY  			 0X8430 //��ɫ
//GUI��ɫ

#define DARKBLUE      	 0X01CF	//����ɫ
#define LIGHTBLUE      	 0X7D7C	//ǳ��ɫ  
#define GRAYBLUE       	 0X5458 //����ɫ
#define LIGHTGREEN     	 0X841F //ǳ��ɫ
#define LIGHTGRAY        0XEF5B //ǳ��ɫ(PANNEL)
#define LGRAY 			 0XC618 //ǳ��ɫ(PANNEL),���屳��ɫ

#define LGRAYBLUE        0XA651 //ǳ����ɫ(�м����ɫ)
#define LBBLUE           0X2B12 //ǳ����ɫ(ѡ����Ŀ�ķ�ɫ)

#define Write_Data_Pin(data)	GPIOB->ODR=(data)

#define Write_RST_Pin(state)	LCD_RST_GPIO_Port->BSRR=(state==H) ? (LCD_RST_Pin) : ((uint32_t)LCD_RST_Pin<<16U);
#define Write_CS_Pin(state)		LCD_CS_GPIO_Port->BSRR=(state==H) ? (LCD_CS_Pin) : ((uint32_t)LCD_CS_Pin<<16U);
#define Write_RS_Pin(state)		LCD_RS_GPIO_Port->BSRR=(state==H) ? (LCD_RS_Pin) : ((uint32_t)LCD_RS_Pin<<16U);
#define Write_RW_Pin(state)		LCD_WR_GPIO_Port->BSRR=(state==H) ? (LCD_WR_Pin) : ((uint32_t)LCD_WR_Pin<<16U);
#define Write_RD_Pin(state)		LCD_RD_GPIO_Port->BSRR=(state==H) ? (LCD_RD_Pin) : ((uint32_t)LCD_RD_Pin<<16U);

#define USE_HORIZONTAL 0	// ʹ������

void LCD_Write_Para(enum DataType type, uint16_t para);
void LCD_Reset(void);
void LCD_Init(void);
void LCD_SetWindows(uint16_t Xstart, uint16_t Ystart, uint16_t Xend, uint16_t Yend);
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos);

void LCD_SetPara(void);
void LCD_Clear(uint16_t color);
void LCD_Fill(uint16_t sx, uint16_t sy, uint16_t ex, uint16_t ey, uint16_t color);


void LCD_DrawPoint(uint16_t x, uint16_t y, uint16_t color);
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color);
void LCD_DrawCircle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t color);

void LCD_ShowNum(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, uint32_t num, uint8_t len, uint8_t size, uint8_t add);
void LCD_ShowFontEN(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, char num, uint8_t size);
void LCD_ShowFontZH(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, char *s, uint8_t size);
void LCD_ShowStr(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, char *str, uint8_t size);

void LCD_ShowBat(uint16_t x, uint16_t y, uint8_t state);
void arc_chabu_area1(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t fc);

void LCD_ShowPic_Heart(uint8_t show_flag);
#endif

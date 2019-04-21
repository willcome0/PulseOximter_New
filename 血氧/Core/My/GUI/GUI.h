#ifndef __GUI_H
#define __GUI_H


//#include "LCD_ST7789V.h"

#ifdef __cplusplus
extern "C" {
#endif
	



	
class C_GUI
{
private:
	
	
	inline void Write_Data(uint16_t data)
	{
		LCD.Write_Para(C_LCD_ST7789V::DATA, data); 
	}

	inline void SetWindows(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd)
	{
		LCD.SetWindows(xStart, yStart, xEnd, yEnd);
	}





public:
	
	C_LCD_ST7789V LCD;
	const uint16_t WIDTH;
	const uint16_t HEIGHT;

	C_GUI(void):WIDTH(LCD.Param.Width),
			  HEIGHT(LCD.Param.Height)
	{
	}

	inline void DrawPoint(uint16_t x, uint16_t y, uint16_t color)
	{
		LCD.DrawPoint(x, y, color);
	}
	
	void Fill(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
	{
		uint16_t width  = x2-x1+1; 			//�õ����Ŀ��
		uint16_t height = y2-y1+1;			//�õ����ĸ߶�
		
		SetWindows(x1, y1, x2-1, y2-1);
		for (uint16_t i=0; i<height; i++)
		{
			for (uint16_t j=0; j<width; j++)
			{
				Write_Data(color);
			}
		}
		SetWindows(0, 0, WIDTH-1, HEIGHT-1);
	}
	
	void AllFill(uint16_t color)
	{
		SetWindows(0, 0, WIDTH-1, HEIGHT-1);
		
		for (uint16_t i=0; i<HEIGHT; i++)
		{
			for (uint16_t j=0; j<WIDTH; j++)
			{
				Write_Data(color);
			}
		}
	}
	
	void DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
	{
		int16_t xerr=0, yerr=0, delta_x, delta_y, distance; 
		int16_t incx, incy, uRow, uCol; 

		delta_x = x2 - x1;	//������������ 
		delta_y = y2 - y1; 
		uRow = x1;
		uCol = y1;
		
		// X�᷽��״̬
		if (delta_x > 0)
			incx = 1;		// ������
		else if (delta_x == 0)
			incx = 0;		// ��ֱ�� 
		else
		{
			incx = -1;		// ������
			delta_x = -delta_x;
		}
		
		// Y�᷽��״̬
		if (delta_y > 0)
			incy = 1; 		// ������
		else if (delta_y == 0)
			incy = 0;		// ˮƽ�� 
		else
		{
			incy = -1;		// ������
			delta_y = -delta_y;
		}
		
		if( delta_x > delta_y)
			distance = delta_x; //ѡȡ�������������� 
		else
			distance = delta_y;
		
		for(uint16_t t=0; t<=distance+1; t++)//������� 
		{  
			DrawPoint(uRow, uCol, color);//���� 
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
	
	void DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
	{
		DrawLine(x1, y1, x2, y1, color);
		DrawLine(x1, y1, x1, y2, color);
		DrawLine(x1, y2, x2, y2, color);
		DrawLine(x2, y1, x2, y2, color);
	}
	
	inline void GUI_DrawFillRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t color)
	{
		Fill(x1, y1, x2, y2, color);
	}
	
	void GUI_ShowChar(uint16_t x, uint16_t y, uint16_t fc, uint16_t bc, char num, uint8_t size, uint8_t cover)
	{
		uint8_t data;
		
		num = num - ' ';//�õ�ƫ�ƺ��ֵ
		SetWindows(x, y, x+size/2-1, y+size-1);//���õ���������ʾ����

		uint8_t RowNum = size;
		uint8_t ColNum = size/2/8 + ((size/2)%8!=0?1:0);
		
		for (uint8_t j=0; j<RowNum; j++)
		{
			for (uint8_t i=0; i<ColNum; i++)
			{
//				switch (size)
//				{
//					case 12:
//						data = asc2_1206[num][j*ColNum + i];
//						break;
//					
//					case 16:
//						data = asc2_1608[num][j*ColNum + i];
//						break;
//					
//					case 24:
//						data = asc2_2412[num][j*ColNum + i];
//						break;
//					
//					default:
//						return;
//				}

				for (uint8_t k=0; k<((i==ColNum-1)?((size/2)%8!=0?(size/2)%8:8):8); k++)
				{
					if (cover)
					{
						if (data&0x01)
							Write_Data(fc);
						else
							Write_Data(bc);
						data >>= 1; 
					}
					else
					{
						if (data&0x01)
							DrawPoint(x+i*8+k, y+j, fc);
						data >>= 1; 
					}
				}
			}
		}
		SetWindows(0, 0, WIDTH-1, HEIGHT-1);  	   	 	  
	}
};



	
	
	
	
	
	
	
	
	
	
	
#ifdef __cplusplus
}
#endif

#endif /* __GUI_H */

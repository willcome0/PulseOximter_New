#ifndef __LCD_ST7789V_H
#define __LCD_ST7789V_H

#include "gpio.h"

#ifdef __cplusplus
extern "C" {
#endif


#define USE_HORIZONTAL 0
	
class C_LCD_ST7789V
{
private:
	
	enum LCD_PinState{L=0, H};

	
	inline void DelayMs(uint16_t ms)
	{
		HAL_Delay(ms);
	}
	
	inline void Write_Data_Pin(uint16_t data)
	{
		GPIOB->ODR = data;
	}
	
	inline void Write_RST_Pin(LCD_PinState state)
	{
		LCD_RST_GPIO_Port->BSRR = (state==H) ? (LCD_RST_Pin) : ((uint32_t)LCD_RST_Pin << 16U);
	}

	inline void Write_CS_Pin(LCD_PinState state)
	{
		LCD_CS_GPIO_Port->BSRR = (state==H) ? (LCD_CS_Pin) : ((uint32_t)LCD_CS_Pin << 16U);
	}

	inline void Write_RS_Pin(LCD_PinState state)
	{
		LCD_RS_GPIO_Port->BSRR = (state==H) ? (LCD_RS_Pin) : ((uint32_t)LCD_RS_Pin << 16U);
	}

	inline void Write_RW_Pin(LCD_PinState state)
	{
		LCD_WR_GPIO_Port->BSRR = (state==H) ? (LCD_WR_Pin) : ((uint32_t)LCD_WR_Pin << 16U);
	}

	inline void Write_RD_Pin(LCD_PinState state)
	{
		LCD_RD_GPIO_Port->BSRR = (state==H) ? (LCD_RD_Pin) : ((uint32_t)LCD_RD_Pin << 16U);
	}
	
	inline void SetCursor(uint16_t x, uint16_t y)
	{
		
	}
public:
	
	C_LCD_ST7789V(void)
	{
		Param.W_GRAMCmd = 0x2C;		// ??AM??????
		#if USE_HORIZONTAL == 1		//?????????	  
			Param.Dir = 1;
			Param.SetXCmd = 0x2B;
			Param.SetYCmd = 0x2A;				 	 		
			Param.Width = 320;
			Param.Height = 240;
		#else	//?????????
			Param.Dir = 0;
			Param.SetXCmd = 0x2A;
			Param.SetYCmd = 0x2B;				 	 		
			Param.Width = 240;
			Param.Height = 320;
		#endif
	};
	
	enum DataType{CMD=0, DATA};
	struct  
	{										    
		uint16_t Width;			//LCD ?????????
		uint16_t Height;		//LCD ??????
		uint16_t ID;			//LCD ID
		uint8_t  Dir;			//??????????????????????????????????????0??????????????????1??????????????????	
		uint16_t W_GRAMCmd;		//????????ram??????
		uint16_t SetXCmd;		//?????????x???????????????
		uint16_t SetYCmd;		//?????????y???????????????	 
	}Param;
	
	void Write_Para(DataType type, uint16_t para);
	void Reset(void);
	void Init(void);
	void SetWindows(uint8_t Xstart, uint8_t Ystart, uint16_t Xend, uint16_t Yend);
	void DrawPoint(uint16_t x, uint16_t y, uint16_t color);
};
	
	
	
	
	
	
	
	
	
#ifdef __cplusplus
}
#endif

#endif /* __LCD_ST7789V_H */

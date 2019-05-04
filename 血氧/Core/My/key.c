#include "key.h"
#include "FT6206.h"



uint8_t ScanKey_Begin(void)
{
	// 检测"开始检测"被按下
	if (65 < CTP.ctpxy.ctp_x && CTP.ctpxy.ctp_x < 175 &&
		268 < CTP.ctpxy.ctp_y && CTP.ctpxy.ctp_y < 306)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t ScanKey_Title(void)
{
	// 触摸标题
	if (70 < CTP.ctpxy.ctp_x && CTP.ctpxy.ctp_x < 170 &&
		0 < CTP.ctpxy.ctp_y && CTP.ctpxy.ctp_y < 25)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t ScanKey_Bat(void)
{
	// 触摸电池
	if (187 < CTP.ctpxy.ctp_x && CTP.ctpxy.ctp_x < 240 &&
		0 < CTP.ctpxy.ctp_y && CTP.ctpxy.ctp_y < 25)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

uint8_t ScanKey_Set(void)
{
	// 检测设置按钮按下
	if (204 < CTP.ctpxy.ctp_x && CTP.ctpxy.ctp_x < 240 &&
		284 < CTP.ctpxy.ctp_y && CTP.ctpxy.ctp_y < 320)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

// 亮度增按钮按下
uint8_t ScanKey_LightAdd(void)
{
	if ((165-5-7) < CTP.ctpxy.ctp_x && CTP.ctpxy.ctp_x < (165+8+5+7) &&
		(105-5) < CTP.ctpxy.ctp_y && CTP.ctpxy.ctp_y < (105+16+5))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

// 亮度减按钮按下
uint8_t ScanKey_LightSub(void)
{

	if ((115-5-7) < CTP.ctpxy.ctp_x && CTP.ctpxy.ctp_x < (115+8+5+7) &&
		(105-5) < CTP.ctpxy.ctp_y && CTP.ctpxy.ctp_y < (105+16+5))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

// 息屏时间增按钮按下
uint8_t ScanKey_ScreenCloseAdd(void)
{
	if ((202-5-7) < CTP.ctpxy.ctp_x && CTP.ctpxy.ctp_x < (202+8+5+7) &&
		(140-5) < CTP.ctpxy.ctp_y && CTP.ctpxy.ctp_y < (140+16+5))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

// 息屏时间减按钮按下
uint8_t ScanKey_ScreenCloseSub(void)
{
	if ((115-5-7) < CTP.ctpxy.ctp_x && CTP.ctpxy.ctp_x < (115+8+5+7) &&
		(140-5) < CTP.ctpxy.ctp_y && CTP.ctpxy.ctp_y < (140+16+5))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

// 关机时间增按钮按下
uint8_t ScanKey_TurnOffAdd(void)
{
	if ((202-5-7) < CTP.ctpxy.ctp_x && CTP.ctpxy.ctp_x < (202+8+5+7) &&
		(175-5) < CTP.ctpxy.ctp_y && CTP.ctpxy.ctp_y < (175+16+5))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

// 关机时间减按钮按下
uint8_t ScanKey_TurnOffSub(void)
{
	if ((115-5-7) < CTP.ctpxy.ctp_x && CTP.ctpxy.ctp_x < (115+8+5+7) &&
		(175-5) < CTP.ctpxy.ctp_y && CTP.ctpxy.ctp_y < (175+16+5))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}

//				LCD_Fill(115-5, 175-5, 115+8+5, 175+16+5, GRAYBLUE);
//				LCD_ShowStr(115, 175, WHITE, GRAYBLUE, "-", 16);
//				LCD_Fill(202-5, 175-5, 202+8+5, 175+16+5, GRAYBLUE);
//				LCD_ShowStr(202, 175, LIGHTGRAY, GRAYBLUE, "+", 16);





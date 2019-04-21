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
		0 < CTP.ctpxy.ctp_y && CTP.ctpxy.ctp_y < 22)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}










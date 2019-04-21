#include "LCD_ST7789V.h"


void C_LCD_ST7789V::Write_Para(DataType type, uint16_t para)
{
	Write_CS_Pin(L);
	Write_RD_Pin(H);
	Write_RS_Pin((LCD_PinState)type);
	
	Write_Data_Pin(para);
	
	Write_RW_Pin(L);
	Write_RW_Pin(H);
	Write_CS_Pin(H);
}

void C_LCD_ST7789V::Reset(void)
{
	Write_RST_Pin(H);
	DelayMs(1);
	Write_RST_Pin(L);
	DelayMs(10);
	Write_RST_Pin(H);
	DelayMs(120);
}

void C_LCD_ST7789V::Init(void)
{
	Reset();
	
	Write_Para(CMD, 0x11);
//	DelayMs(120);
	
	/*
		
		4K-Colors   0x03
		65K-Colors  0x05
		256K-Colors  0x06
	*/
	Write_Para(CMD, 0x3A);
	Write_Para(DATA, 0x05);

	Write_Para(CMD, 0x36);
	Write_Para(DATA, 0x00);
	//--------------------------------ST7789S Frame rate setting----------------------------------//
	Write_Para(CMD, 0xb2);
	Write_Para(DATA, 0x00);
	Write_Para(DATA, 0x00);
	Write_Para(DATA, 0x00);
	Write_Para(DATA, 0x33);
	Write_Para(DATA, 0x33);

	Write_Para(CMD, 0xb7);
	Write_Para(DATA, 0x35);
	//---------------------------------ST7789S Power setting--------------------------------------//
	Write_Para(CMD, 0xb8);
	Write_Para(DATA, 0x2f);
	Write_Para(DATA, 0x2b);
	Write_Para(DATA, 0x2f);

	Write_Para(CMD, 0xbb);
	Write_Para(DATA, 0x24);//vcom

	Write_Para(CMD, 0xc0);
	Write_Para(DATA, 0x2C);

	Write_Para(CMD, 0xc3);
	Write_Para(DATA, 0x10);//0B调深浅

	Write_Para(CMD, 0xc4);
	Write_Para(DATA, 0x20);

	Write_Para(CMD, 0xc6);
	Write_Para(DATA, 0x11);

	Write_Para(CMD, 0xd0);
	Write_Para(DATA, 0xa4);
	Write_Para(DATA, 0xa1);

	Write_Para(CMD, 0xe8);
	Write_Para(DATA, 0x03);

	Write_Para(CMD, 0xe9);
	Write_Para(DATA, 0x0d);
	Write_Para(DATA, 0x12);
	Write_Para(DATA, 0x00);
	//--------------------------------ST7789S gamma setting---------------------------------------//
	Write_Para(CMD, 0xe0);
	Write_Para(DATA, 0xd0);
	Write_Para(DATA, 0x06);
	Write_Para(DATA, 0x0B);
	Write_Para(DATA, 0x0A);
	Write_Para(DATA, 0x09);
	Write_Para(DATA, 0x05);
	Write_Para(DATA, 0x2E);
	Write_Para(DATA, 0x43);
	Write_Para(DATA, 0x44);
	Write_Para(DATA, 0x09);
	Write_Para(DATA, 0x16);
	Write_Para(DATA, 0x15);
	Write_Para(DATA, 0x23);
	Write_Para(DATA, 0x27);

	Write_Para(CMD, 0xe1);
	Write_Para(DATA, 0xd0);
	Write_Para(DATA, 0x06);
	Write_Para(DATA, 0x0B);
	Write_Para(DATA, 0x09);
	Write_Para(DATA, 0x08);
	Write_Para(DATA, 0x06);
	Write_Para(DATA, 0x2E);
	Write_Para(DATA, 0x44);
	Write_Para(DATA, 0x44);
	Write_Para(DATA, 0x3A);
	Write_Para(DATA, 0x15);
	Write_Para(DATA, 0x15);
	Write_Para(DATA, 0x23);
	Write_Para(DATA, 0x26);

	Write_Para(CMD, 0x21); //反显

	Write_Para(CMD, 0x2A); //Frame rate control
	Write_Para(DATA, 0x00);
	Write_Para(DATA, 0x00);
	Write_Para(DATA, 0x00);
	Write_Para(DATA, 0xEF);

	Write_Para(CMD, 0x2B); //Display function control
	Write_Para(DATA, 0x00);
	Write_Para(DATA, 0x00);
	Write_Para(DATA, 0x01);
	Write_Para(DATA, 0x3F);

	Write_Para(CMD, 0x29); //display on
	Write_Para(CMD, 0x2c);

/////Github esp代码
//		Write_Para(CMD, 0xB2);			Write_Para(DATA, 0x000C); //porch setting
//								Write_Para(DATA, 0x000C); 
//								Write_Para(DATA, 0x0000); 
//								Write_Para(DATA, 0x0033); 
//								Write_Para(DATA, 0x0033); 	

//	    Write_Para(CMD, 0xB7);			Write_Para(DATA, 0x0002); //64;VGH&VGL,15V&-10V
//										  	
//		Write_Para(CMD, 0xC2);			Write_Para(DATA, 0X0001);

//		Write_Para(CMD, 0xC3);			Write_Para(DATA, 0X0016); //1F;GVDD,5.1V  16

//		Write_Para(CMD, 0xBB);			Write_Para(DATA, 0x003C); //28;VCOM

//		Write_Para(CMD, 0xC5);			Write_Para(DATA, 0X0020); //vcom shift.0V  20
//															  
////		Write_Para(CMD, 0xC4);			Write_Para(DATA, 0X00,0x20); //VDV,0V



//        Write_Para(CMD, 0xD0);			Write_Para(DATA, 0X00A4); 
//								Write_Para(DATA, 0X00A2); //AVDD&AVCL,6.8v&-4.8v
//													  
//    	Write_Para(CMD, 0xD2);			Write_Para(DATA, 0x004C); 													  

//		Write_Para(CMD, 0xE8);			Write_Para(DATA, 0X0083); //Booster CLK Select
//					 
//	   	Write_Para(CMD, 0xE9);			Write_Para(DATA, 0x0009); //EQ
//								Write_Para(DATA, 0X0009);
//								Write_Para(DATA, 0X0008);

//		Write_Para(CMD, 0x36);			Write_Para(DATA, 0x0000); //ACCESS

//		Write_Para(CMD, 0x35);			Write_Para(DATA, 0x0000); //TE

//		Write_Para(CMD, 0x3A);			Write_Para(DATA, 0x0005); //18bit
//															

//#ifdef INV_DOT										
//		Write_Para(CMD, 0xC6);			Write_Para(DATA, 0x00,0x0F); //0x09orig [DB7-5] 0forDotInv,1forColumnInv; 
//		                                               //[DB4-0] Frame Rate,0x0F:60Hz
//#else
//		Write_Para(CMD, 0xC6);			Write_Para(DATA, 0x00EF); //0x09orig [DB7-5] 0forDotInv,1forColumnInv; 
//		                                               //[DB4-0] Frame Rate,0x0F:60Hz
//#endif
//		
//	    
//		Write_Para(CMD, 0xE0);			Write_Para(DATA, 0x00D0); //V0[7-4] & V63[3-0]
//								Write_Para(DATA, 0x0006); //V62[5-0]
//								Write_Para(DATA, 0x000C); //V61[5-0]
//								Write_Para(DATA, 0x0009); //V59[4-0]
//								Write_Para(DATA, 0x0009); //V57[4-0]
//								Write_Para(DATA, 0x0025); //J1[5-4] & V50[3-0]
//								Write_Para(DATA, 0x002E); //V43[6-0]
//								Write_Para(DATA, 0x0033); //V27[6-4] & V36[2-0]
//								Write_Para(DATA, 0x0045); //V20[6-0]
//								Write_Para(DATA, 0x0036); //J0[5-4] & V13[3-0]
//								Write_Para(DATA, 0x0012); //V6[4-0]
//								Write_Para(DATA, 0x0012); //V4[4-0]
//								Write_Para(DATA, 0x002E); //V2[5-0]
//								Write_Para(DATA, 0x0034); //V1[5-0]

//    	Write_Para(CMD, 0xE1);			Write_Para(DATA, 0x00D0); //V0[7-4] & V63[3-0]
//								Write_Para(DATA, 0x0006); //V62[5-0]
//								Write_Para(DATA, 0x000C); //V61[5-0]
//								Write_Para(DATA, 0x0009); //V59[4-0]
//								Write_Para(DATA, 0x0009); //V57[4-0]
//								Write_Para(DATA, 0x0025); //J1[5-4] & V50[3-0]
//								Write_Para(DATA, 0x002E); //V43[6-0]
//								Write_Para(DATA, 0x0033); //V27[6-4] & V36[2-0]
//								Write_Para(DATA, 0x0045); //V20[6-0]
//								Write_Para(DATA, 0x0036); //J0[5-4] & V13[3-0]
//								Write_Para(DATA, 0x0012); //V6[4-0]
//								Write_Para(DATA, 0x0012); //V4[4-0]
//								Write_Para(DATA, 0x002E); //V2[5-0]
//								Write_Para(DATA, 0x0034); //V1[5-0]
// 	   			
//    	Write_Para(CMD, 0x21);	 
//		
//		Write_Para(CMD, 0x11);		

//		Write_Para(CMD, 0x29);

}



void C_LCD_ST7789V::SetWindows(uint8_t Xstart, uint8_t Ystart, uint16_t Xend, uint16_t Yend)
{
    //set the X coordinates
    Write_Para(CMD, Param.SetXCmd);
    Write_Para(DATA, (Xstart >> 8) & 0xFF);
    Write_Para(DATA, Xstart & 0xFF);
    Write_Para(DATA, ((Xend) >> 8) & 0xFF);
    Write_Para(DATA, (Xend) & 0xFF);

    //set the Y coordinates
    Write_Para(CMD, Param.SetYCmd);
    Write_Para(DATA, (Ystart >> 8) & 0xFF);
    Write_Para(DATA, Ystart & 0xFF);
    Write_Para(DATA, ((Yend) >> 8) & 0xFF);
    Write_Para(DATA, (Yend) & 0xFF);

    Write_Para(CMD, Param.W_GRAMCmd);
}

void C_LCD_ST7789V::DrawPoint(uint16_t x, uint16_t y, uint16_t color)
{
	SetCursor(x, y);
	Write_Para(DATA, color);
}

#include "MAX30100.h"
#include "iic.h"


//-------------------------------------------------------------------------------------//
//函数:      wr_max30100_one_data()
//功能:      写一位max30100数据
//address:   芯片从地址
//saddress:  写寄存器地址
//w_data:    待写数据
//-------------------------------------------------------------------------------------//
uint8_t Max30100_WriteByte(int saddress, int w_data )
{
	/* 第1步：发起I2C总线启动信号 */
	IIC_Start();

	/* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	IIC_Send_Byte(MAX30100_ADDR | I2C_WR);	/* 此处是写指令 */

	/* 第3步：发送ACK */
	if (IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 第4步：发送字节地址 */
	IIC_Send_Byte(saddress);
	if (IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}
	
	/* 第5步：开始写入数据 */
	IIC_Send_Byte(w_data);

	/* 第6步：发送ACK */
	if (IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 发送I2C总线停止信号 */
	IIC_Stop();
	return 1;	/* 执行成功 */

cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
	/* 发送I2C总线停止信号 */
	IIC_Stop();
	return 0;
}

//-------------------------------------------------------------------------------------//
//函数:      rd_max30100_one_data()
//功能:      读一位max30100数据
//address:   芯片从地址
//saddress:  读寄存器地址
//-------------------------------------------------------------------------------------//
uint8_t Max30100_ReadByte(int saddress)
{
	uint8_t data;

	/* 第1步：发起I2C总线启动信号 */
	IIC_Start();

	/* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	IIC_Send_Byte(MAX30100_ADDR | I2C_WR);	/* 此处是写指令 */

	/* 第3步：发送ACK */
	if (IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 第4步：发送字节地址， */
	IIC_Send_Byte((uint8_t)saddress);
	if (IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}
	

	/* 第6步：重新启动I2C总线。下面开始读取数据 */
	IIC_Start();

	/* 第7步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	IIC_Send_Byte(MAX30100_ADDR | I2C_RD);	/* 此处是读指令 */

	/* 第8步：发送ACK */
	if (IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 第9步：读取数据 */
	{
		data = IIC_Read_Byte(0);	/* 读1个字节,产生NACK信号 */
	}
	/* 发送I2C总线停止信号 */
	IIC_Stop();
	return data;	/* 执行成功 返回data值 */

cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
	/* 发送I2C总线停止信号 */
	IIC_Stop();
	return 0;
}

void Max10300_ReadFIFO(uint8_t Register_Address, char Word_Data[], uint8_t count)
{
	uint8_t i=0;
	uint8_t no = count;
	uint8_t data1, data2;
	/* 第1步：发起I2C总线启动信号 */
	IIC_Start();

	/* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	IIC_Send_Byte(MAX30100_ADDR | I2C_WR);	/* 此处是写指令 */

	/* 第3步：发送ACK */
	if (IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 第4步：发送字节地址， */
	IIC_Send_Byte((uint8_t)Register_Address);
	if (IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}
	

	/* 第6步：重新启动I2C总线。下面开始读取数据 */
	IIC_Start();

	/* 第7步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	IIC_Send_Byte(MAX30100_ADDR | I2C_RD);	/* 此处是读指令 */

	/* 第8步：发送ACK */
	if (IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 第9步：读取数据 */
	while (no)
	{
		data1 = IIC_Read_Byte(no==1?0:1);	
//		data2 = IIC_Read_Byte(no==1?0:1);
//		Word_Data[i] = (((uint16_t)data1 << 8) | data2);  //
		Word_Data[i] = data1;
//		
//		data1 = IIC_Read_Byte(1);	

//		data2 = IIC_Read_Byte(no==1?0:1);

//		Word_Data[i][1] = (((uint16_t)data1 << 8) | data2); 

		no--;	
		i++;
	}
	/* 发送I2C总线停止信号 */
	IIC_Stop();

cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
	/* 发送I2C总线停止信号 */
	IIC_Stop();
}


uint8_t Max10300_Reset(void)
{
	if(!Max30100_WriteByte(REG_MODE_CONFIG,0x40))
        return 0;
    else
        return 1; 
}

uint8_t Max10300_Init(void)
{
  if(!Max30100_WriteByte(REG_INTR_ENABLE_1,0xc0)) // INTR setting
    return 0;
  if(!Max30100_WriteByte(REG_INTR_ENABLE_2,0x00))
    return 0;
  if(!Max30100_WriteByte(REG_FIFO_WR_PTR,0x00))  //FIFO_WR_PTR[4:0]
    return 0;
  if(!Max30100_WriteByte(REG_OVF_COUNTER,0x00))  //OVF_COUNTER[4:0]
    return 0;
  if(!Max30100_WriteByte(REG_FIFO_RD_PTR,0x00))  //FIFO_RD_PTR[4:0]
    return 0;
  if(!Max30100_WriteByte(REG_FIFO_CONFIG,0x0f))  //sample avg = 1, fifo rollover=false, fifo almost full = 17
    return 0;
  if(!Max30100_WriteByte(REG_MODE_CONFIG,0x03))   //0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED
    return 0;
  if(!Max30100_WriteByte(REG_SPO2_CONFIG,0x27))  // SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (400uS)
    return 0;
  
  if(!Max30100_WriteByte(REG_LED1_PA,0x24))   //Choose value for ~ 7mA for LED1
    return 0;
  if(!Max30100_WriteByte(REG_LED2_PA,0x24))   // Choose value for ~ 7mA for LED2
    return 0;
  if(!Max30100_WriteByte(REG_PILOT_PA,0x7f))   // Choose value for ~ 25mA for Pilot LED
    return 0;
  return 1;  
	
	
	
//	Max30100_WriteByte(0x06, 0x0b);  	//0000 1011 mode configuration : temp_en[3]      MODE[2:0]=010 HR only enabled    011 SP02 enabled
//	//max10300_Bus_Write(0x06, 0x0a);  	//MODE[2:0]=010 HR only enabled     when used is mode ,the red led is not used.
//	Max30100_WriteByte(0x01, 0xF0); 	//1111 0000 open all of interrupt
//	Max30100_WriteByte(INTERRUPT_REG, 0x00); //all interrupt clear
//	Max30100_WriteByte(0x09, 0x33); 	//0011 0011 r_pa=3,ir_pa=3

////#ifdef SAMPLE_50
//	Max30100_WriteByte(0x07, 0x43); 	//0100 0011 SPO2_SR[4:2]=000   50 per second    LED_PW[1:0]=11  16BITS
////#else
////	Max30100_WriteByte(0x07, 0x47); 	//0100 0111 SPO2_SR[4:2]=001  100 per second    LED_PW[1:0]=11  16BITS
////#endif
//	
//	Max30100_WriteByte(0x02, 0x00);		//set FIFO write Pointer reg = 0x00 for clear it
//	Max30100_WriteByte(0x03, 0x00);		//set Over Flow Counter  reg = 0x00 for clear it
//	Max30100_WriteByte(0x04, 0x0f);		//set FIFO Read Pointer  reg = 0x0f for   
//											//waitting  write pointer eq read pointer   to   interrupts  INTERRUPT_REG_A_FULL
}

#define I2C_WRITE_ADDR 0xAE
#define I2C_READ_ADDR 0xAF
uint8_t maxim_max30102_read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led)
{
  uint32_t un_temp;
  unsigned char uch_temp;
  *pun_red_led=0;
  *pun_ir_led=0;
  char ach_i2c_data[6];
  
  //read and clear status register
  uch_temp = Max30100_ReadByte(REG_INTR_STATUS_1);
  uch_temp = Max30100_ReadByte(REG_INTR_STATUS_2);
  
	
//  ach_i2c_data[0]=REG_FIFO_DATA;
//  if(i2c.write(I2C_WRITE_ADDR, ach_i2c_data, 1, true)!=0)
//    return 0;
//  if(i2c.read(I2C_READ_ADDR, ach_i2c_data, 6, false)!=0)
//  {
//    return 0;
//  }
  Max10300_ReadFIFO(REG_FIFO_DATA, ach_i2c_data, 6);
  un_temp=(unsigned char) ach_i2c_data[0];
  un_temp<<=16;
  *pun_red_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[1];
  un_temp<<=8;
  *pun_red_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[2];
  *pun_red_led+=un_temp;
  
  un_temp=(unsigned char) ach_i2c_data[3];
  un_temp<<=16;
  *pun_ir_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[4];
  un_temp<<=8;
  *pun_ir_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[5];
  *pun_ir_led+=un_temp;
  *pun_red_led&=0x03FFFF;  //Mask MSB [23:18]
  *pun_ir_led&=0x03FFFF;  //Mask MSB [23:18]
  
  
  return 1;
}





                  

struct compx s1[FFT_N+16];           //FFT输入和输出：从S[1]开始存放，根据大小自己定义
struct compx s2[FFT_N+16];           //FFT输入和输出：从S[1]开始存放，根据大小自己定义


#define XPI            (3.1415926535897932384626433832795)
#define XENTRY        (100)
#define XINCL        (XPI/2/XENTRY)
static const double XSinTbl[] = {
        0.00000000000000000  , 0.015707317311820675 , 0.031410759078128292 , 0.047106450709642665 , 0.062790519529313374 ,
        0.078459095727844944 , 0.094108313318514325 , 0.10973431109104528  , 0.12533323356430426  , 0.14090123193758267  ,
        0.15643446504023087  , 0.17192910027940955  , 0.18738131458572463  , 0.20278729535651249  , 0.21814324139654256  ,
        0.23344536385590542  , 0.24868988716485479  , 0.26387304996537292  , 0.27899110603922928  , 0.29404032523230400  ,
        0.30901699437494740  , 0.32391741819814940  , 0.33873792024529142  , 0.35347484377925714  , 0.36812455268467797  ,
        0.38268343236508978  , 0.39714789063478062  , 0.41151435860510882  , 0.42577929156507272  , 0.43993916985591514  ,
        0.45399049973954680  , 0.46792981426057340  , 0.48175367410171532  , 0.49545866843240760  , 0.50904141575037132  ,
        0.52249856471594880  , 0.53582679497899666  , 0.54902281799813180  , 0.56208337785213058  , 0.57500525204327857  ,
        0.58778525229247314  , 0.60042022532588402  , 0.61290705365297649  , 0.62524265633570519  , 0.63742398974868975  ,
        0.64944804833018377  , 0.66131186532365183  , 0.67301251350977331  , 0.68454710592868873  , 0.69591279659231442  ,
        0.70710678118654757  , 0.71812629776318881  , 0.72896862742141155  , 0.73963109497860968  , 0.75011106963045959  ,
        0.76040596560003104  , 0.77051324277578925  , 0.78043040733832969  , 0.79015501237569041  , 0.79968465848709058  ,
        0.80901699437494745  , 0.81814971742502351  , 0.82708057427456183  , 0.83580736136827027  , 0.84432792550201508  ,
        0.85264016435409218  , 0.86074202700394364  , 0.86863151443819120  , 0.87630668004386369  , 0.88376563008869347  ,
        0.89100652418836779  , 0.89802757576061565  , 0.90482705246601958  , 0.91140327663544529  , 0.91775462568398114  ,
        0.92387953251128674  , 0.92977648588825146  , 0.93544403082986738  , 0.94088076895422557  , 0.94608535882754530  ,
        0.95105651629515353  , 0.95579301479833012  , 0.96029368567694307  , 0.96455741845779808  , 0.96858316112863108  ,
        0.97236992039767667  , 0.97591676193874743  , 0.97922281062176575  , 0.98228725072868872  , 0.98510932615477398  ,
        0.98768834059513777  , 0.99002365771655754  , 0.99211470131447788  , 0.99396095545517971  , 0.99556196460308000  ,
        0.99691733373312796  , 0.99802672842827156  , 0.99888987496197001  , 0.99950656036573160  , 0.99987663248166059  ,
        1.00000000000000000  };

double my_floor(double x)
{
   double y=x;
    if( (*( ( (int *) &y)+1) & 0x80000000)  != 0) //或者if(x<0)
        return (float)((int)x)-1;
    else
        return (float)((int)x);
}

double my_fmod(double x, double y)
{
   double temp, ret;
  
   if (y == 0.0)
      return 0.0;
   temp = my_floor(x/y);
   ret = x - temp * y;
   if ((x < 0.0) != (y < 0.0))
      ret = ret - y;
   return ret;
}

double XSin( double x )
{
    int s = 0 , n;
    double dx , sx , cx;
    if( x < 0 )
        s = 1 , x = -x;
    x = my_fmod( x , 2 * XPI );
    if( x > XPI )
        s = !s , x -= XPI;
    if( x > XPI / 2 )
        x = XPI - x;
    n = (int)( x / XINCL );
    dx = x - n * XINCL;
    if( dx > XINCL / 2 )
        ++n , dx -= XINCL;
    sx = XSinTbl[n];
    cx = XSinTbl[XENTRY-n];
    x = sx + dx*cx - (dx*dx)*sx/2
        - (dx*dx*dx)*cx/6 
        + (dx*dx*dx*dx)*sx/24
        ;
     
    return s ? -x : x;
}
 
double XCos( double x )
{
    return XSin( x + XPI/2 );
}

/*******************************************************************
函数原型：struct compx EE(struct compx b1,struct compx b2)  
函数功能：对两个复数进行乘法运算
输入参数：两个以联合体定义的复数a,b
输出参数：a和b的乘积，以联合体的形式输出
*******************************************************************/
struct compx EE(struct compx a,struct compx b)      
{
	 struct compx c;
	 c.real=a.real*b.real-a.imag*b.imag;
	 c.imag=a.real*b.imag+a.imag*b.real;
	 return(c);
}


/*****************************************************************
函数原型：void FFT(struct compx *xin,int N)
函数功能：对输入的复数组进行快速傅里叶变换（FFT）
输入参数：*xin复数结构体组的首地址指针，struct型
*****************************************************************/
void FFT(struct compx *xin)
{
	int f,m,nv2,nm1,i,k,l,j=0;
	struct compx u,w,t;

	nv2=FFT_N/2;                  //变址运算，即把自然顺序变成倒位序，采用雷德算法
	nm1=FFT_N-1;  
	for(i=0;i<nm1;i++)        
	{
		if(i<j)                    //如果i<j,即进行变址
		{
			t=xin[j];           
			xin[j]=xin[i];
			xin[i]=t;
		}
		k=nv2;                    //求j的下一个倒位序
		
		while(k<=j)               //如果k<=j,表示j的最高位为1   
		{           
			j=j-k;                 //把最高位变成0
			k=k/2;                 //k/2，比较次高位，依次类推，逐个比较，直到某个位为0
		}
		
		j=j+k;                   //把0改为1
	}
	 
	{  //FFT运算核，使用蝶形运算完成FFT运算
		int le,lei,ip;                           
		f=FFT_N;
		for(l=1;(f=f/2)!=1;l++)                  //计算l的值，即计算蝶形级数
			;
		for(m=1;m<=l;m++)                           // 控制蝶形结级数
		{                                           //m表示第m级蝶形，l为蝶形级总数l=log（2）N
			le=2<<(m-1);                            //le蝶形结距离，即第m级蝶形的蝶形结相距le点
			lei=le/2;                               //同一蝶形结中参加运算的两点的距离
			u.real=1.0;                             //u为蝶形结运算系数，初始值为1
			u.imag=0.0;
			w.real=XCos(PI/lei);                     //w为系数商，即当前系数与前一个系数的商
			w.imag=-XSin(PI/lei);
			for(j=0;j<=lei-1;j++)                   //控制计算不同种蝶形结，即计算系数不同的蝶形结
			{
				for(i=j;i<=FFT_N-1;i=i+le)            //控制同一蝶形结运算，即计算系数相同蝶形结
				{
					ip=i+lei;                           //i，ip分别表示参加蝶形运算的两个节点
					t=EE(xin[ip],u);                    //蝶形运算，详见公式
					xin[ip].real=xin[i].real-t.real;
					xin[ip].imag=xin[i].imag-t.imag;
					xin[i].real=xin[i].real+t.real;
					xin[i].imag=xin[i].imag+t.imag;
				}
				u=EE(u,w);                           //改变系数，进行下一个蝶形运算
			}
		}
	}

}

#define START_INDEX    10   //滤出低频干扰
uint16_t find_max_num_index(struct compx *data, uint16_t count)
{
	uint16_t i=START_INDEX;
	uint16_t max_num_index = i;
	//struct compx temp=data[i];
	float temp = data[i].real;
	for(i=START_INDEX;i<count;i++)
	{
		if(temp < data[i].real)
		{
			temp = data[i].real;
			max_num_index = i;
		}
	}
	printf("max_num_index=%d\r\n",max_num_index);
	return max_num_index; 
	
}

#include "max7219.h"
#include "stm32f1xx_hal.h"
uint8_t aTxBuf[1]={0};
extern SPI_HandleTypeDef hspi2;
char dg=8;

#define cs_set() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)
#define cs_reset() HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET)

void Send_7219 (uint8_t rg, uint8_t dt)
{
	cs_set();
	aTxBuf[0]=0;
	HAL_SPI_Transmit (&hspi2, (uint8_t*)aTxBuf, 1, 5000);
	aTxBuf[0]=0;
	HAL_SPI_Transmit (&hspi2, (uint8_t*)aTxBuf, 1, 5000);
	aTxBuf[0]=rg;
	HAL_SPI_Transmit (&hspi2, (uint8_t*)aTxBuf, 1, 5000);
	aTxBuf[0]=dt;
	HAL_SPI_Transmit (&hspi2, (uint8_t*)aTxBuf, 1, 5000);
	cs_reset();
}
void Send_7219_Sec (uint8_t rg, uint8_t dt)
{
	cs_set();
	aTxBuf[0]=rg;
	HAL_SPI_Transmit (&hspi2, (uint8_t*)aTxBuf, 1, 5000);
	aTxBuf[0]=dt;
	HAL_SPI_Transmit (&hspi2, (uint8_t*)aTxBuf, 1, 5000);
	aTxBuf[0]=0;
	HAL_SPI_Transmit (&hspi2, (uint8_t*)aTxBuf, 1, 5000);
	aTxBuf[0]=0;
	HAL_SPI_Transmit (&hspi2, (uint8_t*)aTxBuf, 1, 5000);
	cs_reset();
}
//------------------------------------------------------
void Clear_7219 (void)
{
	uint8_t i=dg;
	do
	{
		Send_7219(i,0xF);//символ пустоты
		//Send_7219_Sec(i,0xF);
	} while (--i);
}
void Clear_7219_Sec (void)
{
	uint8_t i=dg;
	do
	{
		Send_7219_Sec(i,0xF);//символ пустоты
		//Send_7219_Sec(i,0xF);
	} while (--i);
}
//------------------------------------------------------
void Number_7219 (volatile long n)
{
	uint8_t ng=0;//переменная для минуса
	if(n<0)
	{
		ng=1;
		n*=-1;
	}
	uint8_t i=0;
	do
	{
		Send_7219(++i,n%10);//символ цифры
		n/=10;
	} while(n);
	if(ng)
	{
		Send_7219(i+1,0x0A);//символ -
	}
}
//-------------------------------------------------------
void Init_7219 (void)
{
	Send_7219_Sec(0x09,0x00);
		Send_7219_Sec(0x0B,dg-1);//кол-во используемых разрядов
		Send_7219_Sec(0x0A,0x02);//интенсивность свечения
		Send_7219_Sec(0x0C,0x01);//включим индикатор
		Clear_7219_Sec();
		//Send_7219(0x09,0xFF);//включим режим декодирования
		Send_7219(0x09,0x00);
		Send_7219(0x0B,dg-1);//кол-во используемых разрядов
		Send_7219(0x0A,0x02);//интенсивность свечения
		Send_7219(0x0C,0x01);//включим индикатор
		Clear_7219();
		

}





















/*
*/





#define     SPI_Speed           200000

/*
const unsigned char fnt[17] = {0x7e,0x30,0x6d,0x79,0x33,0x5b,0x5f,0x70,
						0x7f,0x7b,0x77,0x1f,0x4e,0x3d,0x4f,0x47,0x00};
						*/
/*
 *   a
 *   --
 * f|g |b
 *   --
 * e|  |c
 *   -- .
 *   d  dp
 *
 *  sbl		|a| |b| |c| |d| |e| |f| |g| |dp|       |x|b|x|x| |e|dp|f|g|
 *
 *   0       1   1   1   1   1   1   0   0    0xfa 1111 1010
 *   1       1   1   0   0   0   0   0   0    0x50 0101 0000
 *   2       1   1   0   1   1   0   1   0    0xe9 1110 1001
 *   3       1   1   0   0   0   0   0   0    0x50 0101 0000
 *   4       1   1   0   0   0   0   0   0    0x50 0101 0000
 *   5       1   1   0   0   0   0   0   0    0x50 0101 0000
 *   6       1   0   1   1   1   1   1   0    0xbb 1011 1011
 *   7       1   1   0   0   0   0   0   0    0x50 0101 0000
 *   8       1   1   0   0   0   0   0   0    0x50 0101 0000
 *   9       1   1   0   0   0   0   0   0    0x50 0101 0000
 *   A       1   1   0   0   0   0   0   0    0x50 0101 0000
 *   b       1   1   0   0   0   0   0   0    0x50 0101 0000
 *   C       1   1   0   0   0   0   0   0    0x50 0101 0000
 *   d       1   1   0   0   0   0   0   0    0x50 0101 0000
 *   E       1   1   0   0   0   0   0   0    0x50 0101 0000
 *   F       1   1   0   0   0   0   0   0    0x50 0101 0000
 *
 *
 * */
const unsigned char fnt[17] = {0xfa,0x50,0xe9,0xf1,0x53,0xB3,
						0xBB,	//6
						0x70,	//7
						0xfb,	//8
						0xf3,	//9
						0x7b,	//A
						0x9b,	//B
						0xaa,	//C
						0xd9,	//D
						0xab,	//E
						0x2b,	//F
						0x00};

uint32_t fntStep[12] = {0x1808084e,0x0848084e,0x0048484e,0x4048404e
							,0x6040404e,0x70404046,0x78404042,0x78404840
							,0x78484800,0x78084808,0x7808080c,0x3808080e};
//*****************************************************************************
//
//! \brief Write one byte to MAX7219.
//!
//! \param addr specifies the address to write.
//! \param ucdata specifies the data to write.
//!
//! \return None.
//
//*****************************************************************************
void MAX7219_Write(unsigned char addr, unsigned char ucdata)
{
	Send_7219(addr,ucdata);

}
/*
 * 	Запись в следующий чип
 * */
void MAX7219_WriteSecChip(unsigned char addr, unsigned char ucdata)
{
	
	Send_7219_Sec(addr,ucdata);

}

//*****************************************************************************
//
//! \brief Initialize  MAX7219 and SPI
//!
//! \param None
//!
//! \return None.
//
//*****************************************************************************
void MAX7219_Init(void)
{





	
	//
	// decode for all number, d0~d7
	//
	//MAX7219_Write(MAX7219_DECODE_MODE_ADDR, 0xff);

	//
	// set intensity
	//
	MAX7219_Write(MAX7219_INTENSITY_ADDR, 0x07);
	MAX7219_WriteSecChip(MAX7219_INTENSITY_ADDR, 0x07);

	//
	// set scan limit, scan all number, d0~d7
	//
	MAX7219_Write(MAX7219_SCAN_LIMIT_ADDR, 0x07);
	MAX7219_Write(MAX7219_SHUTDOWN_ADDR, 0x00);
	MAX7219_Write(MAX7219_SHUTDOWN_ADDR, 0x01);
	MAX7219_Write(MAX7219_TEST_ADDR, 0x01);
	MAX7219_Write(MAX7219_TEST_ADDR, 0x00);

	MAX7219_WriteSecChip(MAX7219_SCAN_LIMIT_ADDR, 0x07);

	//MAX7219_Write(MAX7219_SHUTDOWN_ADDR, 0x00);
	MAX7219_WriteSecChip(MAX7219_SHUTDOWN_ADDR, 0x00);

	//
	// set to normal mode
	//
	//MAX7219_Write(MAX7219_SHUTDOWN_ADDR, 0x01);
	MAX7219_WriteSecChip(MAX7219_SHUTDOWN_ADDR, 0x01);

	//
	// not test
	//
	//MAX7219_Write(MAX7219_TEST_ADDR, 0x01);
	MAX7219_WriteSecChip(MAX7219_TEST_ADDR, 0x01);
//	MAX7219_Write(MAX7219_TEST_ADDR, 0x00);
	MAX7219_WriteSecChip(MAX7219_TEST_ADDR, 0x00);
}
/*
 * 	Запись символа в указанную позицию
 * 	*/
void MAX7219_WriteDig(unsigned char pos,
		unsigned char dig)
{
	if(dig<17)
	{
		if(pos<9)
			MAX7219_Write(pos,fnt[dig]);
		if(pos>8)
			MAX7219_WriteSecChip(pos-8,fnt[dig]);
	}
}
/*-------------------------------------------------------------------
	Вывод символов в угазанный индикатор
--------------------------------------------------------------------*/
void MAX7219_WriteBuffToInd(unsigned char ind,unsigned char data[])
{
	unsigned char i;
	if(ind < 9)
	{
		for(i = 0; i< 4; i++)
		{
			MAX7219_Write(i+ind,data[i]);
		}
	}
	if((ind >8)&(ind < 16))
	{
		for(i = 0; i< 4; i++)
		{
			MAX7219_WriteSecChip(i+ind-8,data[i]);
		}
	}

}
/*-------------------------------------------------------------------
	Вывод числа на дисплей
--------------------------------------------------------------------*/
void MAX7219_WriteDec(unsigned char ndisp,int value)
{
	unsigned char buff[4] = {16,16,16,16};
	if((value < 10000)&(value >=0))
	{
		int i = 3;
		do
		{
			buff[i] = value%10;
			value /=10;
			i--;
		}while(value);

		for(i = 0; i<4;i++)
		{
			buff[i] = fnt[buff[i]];
		}

		MAX7219_WriteBuffToInd(ndisp,buff);

	}
}
/*-------------------------------------------------------------------
	Вывод float на дисплей
	Формат 000,0
--------------------------------------------------------------------*/
void MAX7219_WriteFloat(unsigned char ndisp,
									float value)
{
	unsigned char buff[4] = {16,16,0,0};
	unsigned char nDP;
	unsigned int temp;
	//int i = 3;
	nDP = 2;
	temp = (int)(value*10);


	if((temp < 10000)&( temp >= 0))
	{
		int i = 3;
		do
		{
			buff[i] = temp%10;
			temp /=10;
			i--;
		}while(temp);

		for(i = 0; i<4;i++)
		{
			buff[i] = fnt[buff[i]];
		}
		buff[nDP] |= 0x04;
		MAX7219_WriteBuffToInd(ndisp,buff);

	}
}

/*-------------------------------------------------------------------
	Вывод Времени на дисплей, предпологается что время передается в формате
	мм сс, если четное то выводим .
--------------------------------------------------------------------*/
void MAX7219_WriteTime(unsigned char ndisp,int time)
{
	unsigned char cbuff[4] = {0,0,0,0};
	int i = 3;
	do
	{
		cbuff[i] = time%10;
		time /=10;
		i--;
	}while(time);
	for(i = 0; i<4;i++)
	{
		cbuff[i] = fnt[cbuff[i]];
	}
	if(time%2 !=0)
	{
		cbuff[1] |= 0x40;
	}
	cbuff[1] |= 0x04;
	MAX7219_WriteBuffToInd(ndisp,cbuff);
}
/*-------------------------------------------------------------------
	Вывод ошибки Er00
--------------------------------------------------------------------*/
void MAX7219_WriteErr(unsigned char ndisp,int err)
{
	unsigned char cbuff[4] = {0x4f,0x05,0,0};
	int i = 1;
	if(err<100)
	{
		//do
		//{
		cbuff[3] = err%10;
		err /=10;
		cbuff[2] = err;
		//i--;
		//}while(err);
		for(i = 2; i<4;i++)
		{
			cbuff[i] = fnt[cbuff[i]];
		}
	}
	MAX7219_WriteBuffToInd(ndisp,cbuff);
}

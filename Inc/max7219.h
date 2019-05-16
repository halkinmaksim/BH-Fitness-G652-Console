#ifndef MAX7219_H_
#define MAX7219_H_


#include <stdint.h>



//*****************************************************************************
#define MAX7219_DECODE_MODE_ADDR		0x09
#define MAX7219_INTENSITY_ADDR			0x0A
#define MAX7219_SCAN_LIMIT_ADDR			0x0B
#define MAX7219_SHUTDOWN_ADDR			0x0C
#define MAX7219_TEST_ADDR				0x0F

#define MAX7219_CS1   MAX7219_SPI_PORT->BSRR = MAX7219_PIN_SPI_CS
#define MAX7219_CS0   MAX7219_SPI_PORT->BRR = MAX7219_PIN_SPI_CS

#define MAX7219_DIN1   MAX7219_SPI_PORT->BSRR = MAX7219_PIN_SPI_DIN
#define MAX7219_DIN0   MAX7219_SPI_PORT->BRR = MAX7219_PIN_SPI_DIN

#define MAX7219_SCK1   MAX7219_SPI_PORT->BSRR = MAX7219_PIN_SPI_SCK
#define MAX7219_SCK0   MAX7219_SPI_PORT->BRR = MAX7219_PIN_SPI_SCK







//*****************************************************************************



extern uint32_t fntStep[12];

extern void MAX7219_Init(void);
extern void MAX7219_Write(unsigned char ctlByte,
										unsigned char ucdata);
//*****************************************************************************
//

extern void MAX7219_WriteDig(unsigned char pos,
									unsigned char dig);

void MAX7219_WriteBuffToInd(unsigned char ind,unsigned char data[]);
void MAX7219_WriteDec(unsigned char ndisp,int value);

extern void MAX7219_WriteFloat(unsigned char ndisp,
									float value);

void MAX7219_WriteTime(unsigned char ndisp,int time);
void MAX7219_WriteErr(unsigned char ndisp,int err);


void Send_7219 (uint8_t rg, uint8_t dt);
void Send_7219_Sec (uint8_t rg, uint8_t dt);
void Clear_7219 (void);
void Clear_7219_Sec (void);
void Number_7219 (volatile long n);
void Init_7219 (void);

#endif /* MAX7219_H_ */

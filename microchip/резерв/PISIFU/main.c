#define F_CPU 8000000UL
#include <avr/io.h>
#include <math.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#define SS PORTB0
#define CLK PORTB1
#define MOSI PORTB2
#define Block PORTD6
#define PWMpin PORTD5
#define INT0pin PORTD0
#define INT1pin PORTD1
#define USART0pin PORTD4
#define MyAddress 100
#define MBD PORTD3
#define A_B 7
#define BUF 6
#define GA 5
#define SHDN 4
#define speed 51
unsigned char pwm, pwm_state, Duart=0, Dout=0,DoutH,DoutL;
volatile unsigned int  Tic_Count=0, frequency=0;
unsigned char Answer[16],Answer_size=0, temp[16],temp_size=0,uchCRCHi = 0xFF, uchCRCLo = 0xFF;
unsigned char CRCHH, CRCHL, CRCLH, CRCLL;
float Vop=5, Vnov=0;
unsigned char TEMP[16], tempsize = 0, pwm_stateH, pwm_stateL;
volatile unsigned short freq50 = 0;
short NUM;
unsigned char hidh;
unsigned char low;
unsigned char frequency1 = 0;
uint16_t CRC, CRC_check;
#define UART_SPEED 18
void SPI_MasterInit(void)
{
	DDRB|= ((1<<Block)|(1<<SS)|(1<<MOSI)|(1<<CLK));//ножки SPI на выход, B1 сигнальная
	PORTB&= ~((1<<SS)|(1<<MOSI)|(1<<CLK));		//низкий уровень
	SPCR=(1<<SPE)|(1<<MSTR)|(1<<SPR1)|(1<<SPR0);// Включаем шину объявляем ведущего, устанавливаем частоту F/128
}
unsigned char SPI_MasterTransmit(char outdata)
{
	SPDR=outdata;
	while(!(SPSR&(1<<SPIF)));
	return SPDR;
}
unsigned int MCP4921_Conver_Data(float X, float Vref)
{
	unsigned int u;
	X=X/Vref;
	u=(unsigned int)((X*4096));
	return u;
}
void MCP4921_sent_data(unsigned int h)
{
	unsigned int mask_inf=h>>8;
	mask_inf|=(1<<SHDN)|(1<<GA);
	PORTB&=~(1<<PB0);
	SPI_MasterTransmit(mask_inf);
	SPI_MasterTransmit(h);
	PORTB|=(1<<PB0);
}//Расчёт контрольной суммы
void CRC16(unsigned char *adr_buffer, unsigned char byte_cnt)
{
	static unsigned char auchCRCHi[]=
	{
		0x00,0xC0,0xC1,0x01,0xC3,0x03,0x02,0xC2,0xC6,0x06,0x07,0xC7,0x05,0xC5,0xC4,0x04,
		0xCC,0x0C,0x0D,0xCD,0x0F,0xCF,0xCE,0x0E,0x0A,0xCA,0xCB,0x0B,0xC9,0x09,0x08,0xC8,
		0xD8,0x18,0x19,0xD9,0x1B,0xDB,0xDA,0x1A,0x1E,0xDE,0xDF,0x1F,0xDD,0x1D,0x1C,0xDC,
		0x14,0xD4,0xD5,0x15,0xD7,0x17,0x16,0xD6,0xD2,0x12,0x13,0xD3,0x11,0xD1,0xD0,0x10,
		0xF0,0x30,0x31,0xF1,0x33,0xF3,0xF2,0x32,0x36,0xF6,0xF7,0x37,0xF5,0x35,0x34,0xF4,
		0x3C,0xFC,0xFD,0x3D,0xFF,0x3F,0x3E,0xFE,0xFA,0x3A,0x3B,0xFB,0x39,0xF9,0xF8,0x38,
		0x28,0xE8,0xE9,0x29,0xEB,0x2B,0x2A,0xEA,0xEE,0x2E,0x2F,0xEF,0x2D,0xED,0xEC,0x2C,
		0xE4,0x24,0x25,0xE5,0x27,0xE7,0xE6,0x26,0x22,0xE2,0xE3,0x23,0xE1,0x21,0x20,0xE0,
		0xA0,0x60,0x61,0xA1,0x63,0xA3,0xA2,0x62,0x66,0xA6,0xA7,0x67,0xA5,0x65,0x64,0xA4,
		0x6C,0xAC,0xAD,0x6D,0xAF,0x6F,0x6E,0xAE,0xAA,0x6A,0x6B,0xAB,0x69,0xA9,0xA8,0x68,
		0x78,0xB8,0xB9,0x79,0xBB,0x7B,0x7A,0xBA,0xBE,0x7E,0x7F,0xBF,0x7D,0xBD,0xBC,0x7C,
		0xB4,0x74,0x75,0xB5,0x77,0xB7,0xB6,0x76,0x72,0xB2,0xB3,0x73,0xB1,0x71,0x70,0xB0,
		0x50,0x90,0x91,0x51,0x93,0x53,0x52,0x92,0x96,0x56,0x57,0x97,0x55,0x95,0x94,0x54,
		0x9C,0x5C,0x5D,0x9D,0x5F,0x9F,0x9E,0x5E,0x5A,0x9A,0x9B,0x5B,0x99,0x59,0x58,0x98,
		0x88,0x48,0x49,0x89,0x4B,0x8B,0x8A,0x4A,0x4E,0x8E,0x8F,0x4F,0x8D,0x4D,0x4C,0x8C,
		0x44,0x84,0x85,0x45,0x87,0x47,0x46,0x86,0x82,0x42,0x43,0x83,0x41,0x81,0x80,0x40
	};

	static char auchCRCLo[]=
	{
		0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
		0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
		0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
		0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
		0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
		0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
		0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
		0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
		0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
		0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
		0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
		0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
		0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
		0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
		0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
		0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40
	};

	unsigned uIndex;
	uchCRCHi = 0xFF;
	uchCRCLo = 0xFF;
	
	while(byte_cnt--)
	{
		uIndex = uchCRCLo ^ *adr_buffer++;
		uchCRCLo = uchCRCHi ^ auchCRCLo[uIndex];
		uchCRCHi = auchCRCHi[uIndex];
	}
}
void port_ini(void)
{
	DDRD  &=~(0b00000011) ;
	PORTD = 0b00000000;
	DDRB |= (1<<PB7);                       
}


void USART_Init (void)
{
	UBRR1H = (unsigned char)(UART_SPEED >> 8);;
	UBRR1L = (unsigned char) UART_SPEED;

	UBRR0H = (unsigned char)(UART_SPEED >> 8);
	UBRR0L = (unsigned char) UART_SPEED;

	UCSR1B = (1 << TXEN1) | (1 << RXEN1)|(1<<RXCIE1);
	UCSR1C = (1 << UCSZ11) | (1 << UCSZ10);
	UCSR0B = (1 << TXEN0) | (1 << RXEN0) | (1<<RXCIE0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void USART_Transmit(unsigned char data)
{
	while(!(UCSR0A & (1 << UDRE0)));

	UDR0 = data;
}

void USART_Transmit1(unsigned char data)
{
	while(!(UCSR1A & (1 << UDRE1)));

	UDR1 = data;
}


//Обработчик внешнего прерывания INT0
ISR(INT0_vect)
{
	Tic_Count = Tic_Count+1;
}



ISR(INT1_vect)
{
	 freq50++;
	 if(freq50 == 10){
		 frequency = Tic_Count*5;
		 if(abs(frequency-8000) < 50) {
			 frequency1 = 255;
			 }else if(abs(frequency-2000) < 50) {
			 frequency1 = 0;
			 } else {
			 frequency1 = ((float)abs(Tic_Count*5 - 2000)/6000) * 255.0;
		 }
		 freq50 =0 ;
		 Tic_Count = 0;
}
}

short size0 = 0;
static unsigned char str0[4];

short powed(short x, short n) {
	if (n <= 0)
	return 1;
	else if (n == 1)
	return x;
	else if ((n % 2) == 0 )
	return powed( x * x, n/2);
	else
	return powed( x * x, n /2)*x;
}

short charToDec() {
	short i;
	short num;
	short adr;
	adr = 0;

	for(i = size0 - 1; i >= 0; i--) {
		num = (short)str0[i] - (short)48;

		adr = adr + num * powed(10, (size0 - 1 - i));
	}

	return adr;
}

unsigned int convertToDac(float x)
{
	unsigned int u;

	x = x / 5.0;
	u = (unsigned int) (x * 4096.0);

	return u;
}

void USART_transmit_str1(const char* str)
{
	int i;
	for(i=0;i<strlen(str);i++)
	{
		while(!(UCSR1A&(1<<UDRE1)));
		UDR1 = str[i];
	}
}

void USART_transmit_str(const char* str)
{
	int i;
	for(i=0;i<strlen(str);i++)
	{
		while(!(UCSR0A&(1<<UDRE0)));
		UDR0 = str[i];
	}
}
float x = 0;

ISR(USART1_RX_vect) //число с компьютера
{
	short i;
	
	str0[size0]  = UDR1;

	USART_Transmit1(str0[size0]);
	size0++;

	if(((size0 > 3) || (UDR1 == 13)) && (size0 != 0))
	{
		USART_Transmit1(7);
		size0 = size0 - 1;
		NUM = charToDec();
		
		x=acos(2*275/508.3*(1-(180-(float)NUM+4)/180)-1);
		x=x*180/3.14;
		x=x/0.018;
		x=15*(1-1/exp(x/30000));
		Vnov=x;
		if(NUM > 180)
		{
			USART_transmit_str1("Must be <180!");
			USART_Transmit1(0x0d);
		}
		else
			MCP4921_sent_data(MCP4921_Conver_Data(Vnov,5));

		size0 = 0;
		for(i = 0; i < size0; i++) {
			str0[i] = 0;
		}
	}
}


unsigned char UART_ConvertByte (unsigned char c)
{
	unsigned char t=0;
	int i;
	if (c<<0)
	{
		i=0;
	}
	if (c==0x0001)
	{
		i=1;
	}
	if (c==0x0002)
	{
		i=2;
	}
	if (c==0x0003)
	{
		i=3;
	}
	if (c==0x0004)
	{
		i=4;
	}
	if (c==0x0005)
	{
		i=5;
	}
	if (c==0x0006)
	{
		i=6;
	}
	if (c==0x0007)
	{
		i=7;
	}
	if (c==0x0008)
	{
		i=8;
	}
	if (c==0x0009)
	{
		i=9;
	}
	if (c==0x000A)
	{
		i=10;
	}
	if (c==0x000B)
	{
		i=11;
	}
	if (c==0x000C)
	{
		i=12;
	}
	if (c==0x000D)
	{
		i=13;
	}
	if (c==0x000E)
	{
		i=14;
	}
	if (c==0x000F)
	{
		i=15;
	}
	switch(i)
	{
		case 0:
		t=0x30;
		break;
		case 1:
		t=0x31;
		break;
		case 2:
		t=0x32;
		break;
		case 3:
		t=0x33;
		break;
		case 4:
		t=0x34;
		break;
		case 5:
		t=0x35;
		break;
		case 6:
		t=0x36;
		break;
		case 7:
		t=0x37;
		break;
		case 8:
		t=0x38;
		break;
		case 9:
		t=0x39;
		break;
		case 10:
		t=0x41;
		break;
		case 11:
		t=0x42;
		break;
		case 12:
		t=0x43;
		break;
		case 13:
		t=0x44;
		break;
		case 14:
		t=0x45;
		break;
		case 15:
		t=0x46;
		break;
	}
	return t;
}

ISR(USART0_RX_vect) {
	TEMP[tempsize] = UDR0;
	unsigned char Doutx;
	unsigned char j;
	USART_Transmit(TEMP[tempsize]);
	tempsize++;
	if (tempsize==12)
	CRC16(TEMP,tempsize);
	if(tempsize == 16)
	{
		CRCHH=(uchCRCHi&0x00F0)>>4;
		CRCHH=UART_ConvertByte(CRCHH);
		CRCHL=(uchCRCHi&0x000F);
		CRCHL=UART_ConvertByte(CRCHL);
		CRCLH=(uchCRCLo&0x00F0)>>4;
		CRCLH=UART_ConvertByte(CRCLH);
		CRCLL=(uchCRCLo&0x000F);
		CRCLL=UART_ConvertByte(CRCLL);
		CRC_check=(CRCHH<<12 | CRCHL<<8 | CRCLH<<4| CRCLL);
		CRC=(TEMP[12]<<12 |TEMP[13]<<8 | TEMP[14]<<4| TEMP[15]);
		if(CRC==CRC_check)
		{
			if ((TEMP[0]==0x30)&&(TEMP[1]==0x31))
			{
				if ((TEMP[2]==0x30)&&(TEMP[3]==0x33))
				{
					if ((TEMP[4]==0x31)&&(TEMP[5]==0x34)&&(TEMP[6]==0x31)&&(TEMP[7]==0x34)&&(TEMP[8]==0x32)&&(TEMP[9]==0x30)&&(TEMP[10]==0x32)&&(TEMP[11]==0x31))
					{
						
					USART_Transmit(0x0d);
					
					pwm = (frequency1*23.53+2000)/100;
					pwm_stateH = (pwm)/16;
					pwm_stateL = (pwm)%16;
					//Doutx = x*100;
					Doutx = NUM;
					DoutH = (Doutx)/16;
					DoutL = (Doutx)%16;

					TEMP[6]=UART_ConvertByte(pwm_stateH);
					TEMP[7]=UART_ConvertByte(pwm_stateL);

					
					TEMP[10]=UART_ConvertByte(DoutH);
					TEMP[11]=UART_ConvertByte(DoutL);
					CRC16(TEMP,12);
					CRCHH=(uchCRCHi&0x00F0)>>4;
					CRCHH=UART_ConvertByte(CRCHH);
					CRCHL=(uchCRCHi&0x000F);
					CRCHL=UART_ConvertByte(CRCHL);
					CRCLH=(uchCRCLo&0x00F0)>>4;
					CRCLH=UART_ConvertByte(CRCLH);
					CRCLL=(uchCRCLo&0x000F);
					CRCLL=UART_ConvertByte(CRCLL);
					CRC_check=(CRCHH<<12 | CRCHL<<8 | CRCLH<<4| CRCLL);
					CRC=(TEMP[12]<<12 |TEMP[13]<<8 | TEMP[14]<<4| TEMP[15]);
					
					for(j = 0; j < 12; j++)
					{
						USART_Transmit(TEMP[j]);
					}
					//USART_Transmit(0x0d);
					//USART_Transmit(0x0d);
					//USART_Transmit(0x0a);
					USART_Transmit(CRCHH);
					USART_Transmit(CRCHL);
					USART_Transmit(CRCLH);
					USART_Transmit(CRCLL);
					//USART_Transmit(0x0d);
					//USART_Transmit(0x0d);
					//USART_Transmit(0x0a);
					//USART_Transmit(0x0d);
					//USART_transmit_str("CRC is correct.");
					USART_Transmit(0x0d);
					//USART_transmit_str("Meander Frequency = ");
					//USART_Transmit(TEMP[6]);
					//USART_Transmit(TEMP[7]);
					
					//USART_transmit_str(" HEX * 100 (Hz)");
					USART_Transmit(0x0d);
					//USART_transmit_str("IFM amplitude  = ");
					//USART_Transmit(TEMP[10]);
					//USART_Transmit(TEMP[11]);
					
					//USART_transmit_str(" HEX / 100 (V)");
					USART_Transmit(0x0d);
					tempsize = 0;
					}
					else
					{
						USART_Transmit(0x0d);
						//USART_transmit_str("Incorrect Request!");
						//USART_Transmit(0x0d);
						for(j = 4; j < 12; j++)
						{
							TEMP[j]=UART_ConvertByte(0);
						}
						for(j = 0; j < 12; j++)
						{
							USART_Transmit(TEMP[j]);
						}
						CRC16(TEMP,12);
						CRCHH=(uchCRCHi&0x00F0)>>4;
						CRCHH=UART_ConvertByte(CRCHH);
						CRCHL=(uchCRCHi&0x000F);
						CRCHL=UART_ConvertByte(CRCHL);
						CRCLH=(uchCRCLo&0x00F0)>>4;
						CRCLH=UART_ConvertByte(CRCLH);
						CRCLL=(uchCRCLo&0x000F);
						CRCLL=UART_ConvertByte(CRCLL);
						CRC_check=(CRCHH<<12 | CRCHL<<8 | CRCLH<<4| CRCLL);
						CRC=(TEMP[12]<<12 |TEMP[13]<<8 | TEMP[14]<<4| TEMP[15]);
						USART_Transmit(CRCHH);
						USART_Transmit(CRCHL);
						USART_Transmit(CRCLH);
						USART_Transmit(CRCLL);
						USART_Transmit(0x0d);
						tempsize=0;	
					}
				}
				else
				{
					USART_Transmit(0x0d);
					//USART_transmit_str("Incorrect ModBus Function ID!");
					//USART_Transmit(0x0d);
					for(j = 4; j < 12; j++)
					{
						TEMP[j]=UART_ConvertByte(1);
					}
					for(j = 0; j < 12; j++)
					{
						USART_Transmit(TEMP[j]);
					}
					CRC16(TEMP,12);
					CRCHH=(uchCRCHi&0x00F0)>>4;
					CRCHH=UART_ConvertByte(CRCHH);
					CRCHL=(uchCRCHi&0x000F);
					CRCHL=UART_ConvertByte(CRCHL);
					CRCLH=(uchCRCLo&0x00F0)>>4;
					CRCLH=UART_ConvertByte(CRCLH);
					CRCLL=(uchCRCLo&0x000F);
					CRCLL=UART_ConvertByte(CRCLL);
					CRC_check=(CRCHH<<12 | CRCHL<<8 | CRCLH<<4| CRCLL);
					CRC=(TEMP[12]<<12 |TEMP[13]<<8 | TEMP[14]<<4| TEMP[15]);
					USART_Transmit(CRCHH);
					USART_Transmit(CRCHL);
					USART_Transmit(CRCLH);
					USART_Transmit(CRCLL);	
					USART_Transmit(0x0d);
				tempsize=0;	
				}
				
			}
			else
			{
				USART_Transmit(0x0d);
				//USART_transmit_str("Incorrect ModBus Device ID!");
				//USART_Transmit(0x0d);
				for(j = 4; j < 12; j++)
				{
					TEMP[j]=UART_ConvertByte(2);
				}
				for(j = 0; j < 12; j++)
				{
					USART_Transmit(TEMP[j]);
				}
				CRC16(TEMP,12);
				CRCHH=(uchCRCHi&0x00F0)>>4;
				CRCHH=UART_ConvertByte(CRCHH);
				CRCHL=(uchCRCHi&0x000F);
				CRCHL=UART_ConvertByte(CRCHL);
				CRCLH=(uchCRCLo&0x00F0)>>4;
				CRCLH=UART_ConvertByte(CRCLH);
				CRCLL=(uchCRCLo&0x000F);
				CRCLL=UART_ConvertByte(CRCLL);
				CRC_check=(CRCHH<<12 | CRCHL<<8 | CRCLH<<4| CRCLL);
				CRC=(TEMP[12]<<12 |TEMP[13]<<8 | TEMP[14]<<4| TEMP[15]);
				USART_Transmit(CRCHH);
				USART_Transmit(CRCHL);
				USART_Transmit(CRCLH);
				USART_Transmit(CRCLL);
				USART_Transmit(0x0d);
			tempsize=0;
			}
		}
		else
		{
			//USART_transmit_str("CRC is not correct.");
			for(j = 4; j < 8; j++)
			{
				TEMP[j]=UART_ConvertByte(4);
			}
			TEMP[8]=CRCHH;
			TEMP[9]=CRCHL;
			TEMP[10]=CRCLH;
			TEMP[11]=CRCLL;
			USART_Transmit(0x0d);
			for(j = 0; j < 12; j++)
			{
				USART_Transmit(TEMP[j]);
			}
			CRC16(TEMP,12);
			CRCHH=(uchCRCHi&0x00F0)>>4;
			CRCHH=UART_ConvertByte(CRCHH);
			CRCHL=(uchCRCHi&0x000F);
			CRCHL=UART_ConvertByte(CRCHL);
			CRCLH=(uchCRCLo&0x00F0)>>4;
			CRCLH=UART_ConvertByte(CRCLH);
			CRCLL=(uchCRCLo&0x000F);
			CRCLL=UART_ConvertByte(CRCLL);
			CRC_check=(CRCHH<<12 | CRCHL<<8 | CRCLH<<4| CRCLL);
			CRC=(TEMP[12]<<12 |TEMP[13]<<8 | TEMP[14]<<4| TEMP[15]);
			USART_Transmit(CRCHH);
			USART_Transmit(CRCHL);
			USART_Transmit(CRCLH);
			USART_Transmit(CRCLL);
			USART_Transmit(0x0d);
			tempsize=0;
		}
	}
}

void init_PWM_timer(void)
{
	TCCR2 |= (1<<WGM20)|(1<<WGM21);                //select Fast PWM mode by setting bits
	TCCR2 |=(1<<COM20)|(1<<COM21)|(1<<CS20)|(0<<CS21)|(0<<CS22);   //clear OC2 on compare match
}

void INT0_initial (void)
{
	EICRA |= (1<<ISC00)|(1<<ISC01); //Восходящий фронт сигнала
	EIMSK |= (1<<INT0); //Включение входа прерывания
	sei(); //Разрешаем прерывания
}

void INT1_initial (void)
{
	//EICRB |= (1<<ISC40)|(1<<ISC71);
	EICRA |= (1<<ISC00)|(1<<ISC01); //Восходящий фронт сигнала
	EIMSK |= (1<<INT1); //Включение входа прерывания
	sei(); //Разрешаем прерывания
}

int main(void)
{
	port_ini();
	init_PWM_timer();
	INT0_initial();
	INT1_initial();
	SPI_MasterInit();
	USART_Init();
	sei();
	while(1)
	{

		cli(); //Запрещаем прерывания
		//pwm_state = Tic_Count/10.35;
		//if(abs(pwm_state - 45) < e)
			//pwm_state = 0;
		//OCR2 = pwm_state;
		OCR2 = frequency1;
		sei(); //Разрешаем прерывания
		//_delay_ms(100);
	}
}

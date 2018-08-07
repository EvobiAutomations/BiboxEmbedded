#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "max7219.h"
#include "soft_spi.h"

extern const unsigned char FONT[95][5];


void Max_7219_Initial(void)
{
			Soft_Spi_Initial();
			Max_Shutdown(false,5);
			Max_Decode_Mode(NO_DECODE,5);
			Max_Intensity(0xff,5);
			Max_Scan_Limit(0x7,5);
			Max_Display_test(false,5);
			Max_Display_Clear(5);
	
	
}
/*
void Max_Write_Byte(unsigned char num )
{
	unsigned char t;  
	
 
	 nrf_gpio_pin_clear(MAX_CS);
  for (t=0; t<8; t++) 
    { 
      if ((num & 0x80) == 0) 
			{
         nrf_gpio_pin_clear(MAX_SDA);
			}				
      else 
			{
        nrf_gpio_pin_set(MAX_SDA); 
			}
      nrf_gpio_pin_set(MAX_SCK);
			//app_uart_put('o');
			//while(true);
						
      num <<= 1;    //shift left one bit 
       
      
			//nrf_delay_us(1);
			nrf_gpio_pin_clear(MAX_SCK);
    } 
		
	}	
	
*/	
	void Max_Display_test(bool flag, char len)
	{
		
		if(flag)
		{ 
			Max_Display_Data(0x0f,0x01,len);
			
		}
		else{
			
		  Max_Display_Data(0x0f,0x0,len);
			
		}
		
		
	}
	
	
	void Max_Shutdown(bool flag,char len)
	{
			
		if(flag)
		{
			
			
			Max_Display_Data(0x0c,0x00,len);
			
			
		}
		else
		{			
			
			Max_Display_Data(0x0c,0x01,len);
			
		}
	
		
	}
	
	
	void Max_Decode_Mode(unsigned char val,char len )
	
	{
			
			Max_Display_Data(0x09,val,len);
		
	}
		
	
void Max_Intensity(unsigned char val,char len)
{
		
		
		Max_Display_Data(0x0a,val,len);


}	

void Max_Scan_Limit(unsigned char val,char len)
	
{
	
		
		Max_Display_Data(0x0b,val,len);
	
	
}
void Max_Display_Data(unsigned char addr,unsigned char val,char len)
{
	
		nrf_gpio_pin_clear(MAX_CS);
			for(char i=0;i<len;i++)
			{
				Soft_Spi_Write_Read(addr);
				Soft_Spi_Write_Read(val);
			}
		
		nrf_gpio_pin_set(MAX_CS);
		
}
	



void Max_Display_Clear(char len)
{
	for(unsigned char i=1;i<0x09;i++)
	{
		Max_Display_Data(i,0,len);
	}
	
}

void Max_Display_Data_2X(unsigned char addr,unsigned char val1,unsigned char val2)
{
	
		nrf_gpio_pin_clear(MAX_CS);
		Soft_Spi_Write_Read(addr);
		Soft_Spi_Write_Read(val1);
		Soft_Spi_Write_Read(addr);
		Soft_Spi_Write_Read(val2);
		nrf_gpio_pin_set(MAX_CS);
		
}


/*

void Put_Char_OLED_1X( unsigned char X)
{

	unsigned int TEMP;
	const unsigned char *PTR;
        TEMP=X-0X20;
        PTR=&(FONT[TEMP][0]);
        for(int I=0;I<5;I++)
        {
        Max_Display_Data(1+I,*(PTR));
        PTR++;
				
        }
        Max_Display_Data(0x06,0);
				Max_Display_Data(0x07,0);
				Max_Display_Data(0x08,0);
}


void Put_Char_OLED_2X( unsigned char X,unsigned char Y)
{

	unsigned int TEMP,TEMP1;
	const unsigned char *PTR;
	const unsigned char *PTR1;
        TEMP=X-0X20;
				TEMP1=Y-0x20;
        PTR=&(FONT[TEMP][0]);
				PTR1=&(FONT[TEMP1][0]);
        for(int I=0;I<5;I++)
        {
        Max_Display_Data_2X(1+I,*(PTR),*(PTR1));
        PTR++;
				PTR1++;
        }
        Max_Display_Data_2X(0x06,0,0);
				Max_Display_Data_2X(0x07,0,0);
				Max_Display_Data_2X(0x08,0,0);
}



void Max_Display_Data_X(unsigned char addr,unsigned char *data,unsigned char len)
{
	nrf_gpio_pin_clear(MAX_CS);
	
	for(char i=0;i<len;i++)
	{	
		Soft_Spi_Write_Read(addr);
		Soft_Spi_Write_Read(*data);
		data++;
	}
	nrf_gpio_pin_set(MAX_CS);
	
}

*/
void Max_Display_Char_X(unsigned char *data,unsigned char len)
{
	unsigned int TEMP;
	const unsigned char *PTR;
	
	
	
	for(char j=0;j<5;j++)
	{
		nrf_gpio_pin_clear(MAX_CS);
		for(char i=0;i<len;i++)
		{
			TEMP=*(data+i)-0X20;
			PTR=&(FONT[TEMP][0]);
			PTR+=j;
			Soft_Spi_Write_Read(1+j);
			Soft_Spi_Write_Read(*(PTR));
			
			
		}
		nrf_gpio_pin_set(MAX_CS);
		
	}
	
	
	
}



#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "lcd.h"



void LCD_Pin_Write(unsigned char data)
{
	nrf_gpio_pin_write(LCD_D4,data&0x01);
	nrf_gpio_pin_write(LCD_D5,(data>>1)&0x01);
	nrf_gpio_pin_write(LCD_D6,(data>>2)&0x01);
	nrf_gpio_pin_write(LCD_D7,(data>>3)&0x01);
	
}

void LCD_Cmd(unsigned char cmd )
{
	
		
		LCD_Pin_Write(cmd>>4);
		nrf_gpio_pin_clear(LCD_RS);
	
		nrf_gpio_pin_set(LCD_EN);
	
		LCD_DELAY();
		nrf_gpio_pin_clear(LCD_EN);
		LCD_DELAY();
	
		LCD_Pin_Write(cmd);
	
		nrf_gpio_pin_set(LCD_EN);
		LCD_DELAY();
		nrf_gpio_pin_clear(LCD_EN);
		LCD_DELAY();
				

	
	
	
}


void LCD_Data(unsigned char data )
{

		
		LCD_Pin_Write(data>>4);
		nrf_gpio_pin_set(LCD_RS);
	
		nrf_gpio_pin_set(LCD_EN);
		LCD_DELAY();
		nrf_gpio_pin_clear(LCD_EN);
		LCD_DELAY();
	
		LCD_Pin_Write(data);
	
		nrf_gpio_pin_set(LCD_EN);
		LCD_DELAY();
		nrf_gpio_pin_clear(LCD_EN);
		LCD_DELAY();
				

	
	
	
}




void LCD_Initial(unsigned char Lines, bool cursonOn, bool blink)
{
	#ifdef _4BIT
	
	nrf_gpio_cfg_output(LCD_RS);
	nrf_gpio_cfg_output(LCD_EN);
	nrf_gpio_cfg_output(LCD_D4);
	nrf_gpio_cfg_output(LCD_D5);
	nrf_gpio_cfg_output(LCD_D6);
	nrf_gpio_cfg_output(LCD_D7);
	nrf_delay_ms(100); // Power on delay
	
		//initially 8 bit mode, change to 4bit mode
		LCD_Pin_Write(0x03);
		nrf_delay_ms(10);
		LCD_Pin_Write(0x03);
		nrf_delay_ms(10);
	
	
		LCD_Pin_Write(0x02);//4 bit , 1 line mode
		
		nrf_gpio_pin_clear(LCD_RS);
		nrf_gpio_pin_clear(LCD_EN);
		LCD_DELAY();
		nrf_gpio_pin_set(LCD_EN);
		LCD_DELAY();
		nrf_gpio_pin_clear(LCD_EN);
		
		nrf_delay_ms(5);
		if(Lines==2)
		{
			LCD_Cmd(0x28);// 4bit, 2 line,5*7
					
		
		}
		LCD_Cmd(0x0c|(0x02&blink)|(0x01&cursonOn));
	
	#endif
	
	
	
}



void LCD_PutString(const unsigned char* str)
{
	
	while(*str)LCD_Data(*str++);
	
}




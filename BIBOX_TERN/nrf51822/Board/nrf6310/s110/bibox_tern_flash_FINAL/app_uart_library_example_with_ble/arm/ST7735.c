#include "ST7735.h"
#include <nrf_delay.h>
#include "nrf_gpio.h"
#include "nrf.h"


	extern unsigned char y_loca;
	extern unsigned char x_loca;	




const unsigned char FONT[95][5] =
{
    { 0x00, 0x00, 0x00, 0x00, 0x00 }, // " " 0x20
    { 0x00, 0x00, 0x4f, 0x00, 0x00 }, // !   0x21
    { 0x00, 0x07, 0x00, 0x07, 0x00 }, // "   0x22
    { 0x14, 0x7f, 0x14, 0x7f, 0x14 }, // #   0x23
    { 0x24, 0x2a, 0x7f, 0x2a, 0x12 }, // $   0x24
    { 0x23, 0x13, 0x08, 0x64, 0x62 }, // %   0x25
    { 0x36, 0x49, 0x55, 0x22, 0x50 }, // &   0x26
    { 0x00, 0x05, 0x03, 0x00, 0x00 }, // '   0x27
    { 0x00, 0x1c, 0x22, 0x41, 0x00 }, // (   0x28
    { 0x00, 0x41, 0x22, 0x1c, 0x00 }, // )   0x29
    { 0x14, 0x08, 0x3e, 0x08, 0x14 }, // *   0x2A
    { 0x08, 0x08, 0x3e, 0x08, 0x08 }, // +   0x2B
    { 0x00, 0x50, 0x30, 0x00, 0x00 }, // ,   0x2C
    { 0x08, 0x08, 0x08, 0x08, 0x08 }, // -   0x2D
    { 0x00, 0x60, 0x60, 0x00, 0x00 }, // .   0x2E
    { 0x20, 0x10, 0x08, 0x04, 0x02 }, // /   0x2F
    { 0x3e, 0x51, 0x49, 0x45, 0x3e }, // 0   0x30
    { 0x00, 0x42, 0x7f, 0x40, 0x00 }, // 1   0x31
    { 0x42, 0x61, 0x51, 0x49, 0x46 }, // 2   0x32
    { 0x21, 0x41, 0x45, 0x4b, 0x31 }, // 3   0x33
    { 0x18, 0x14, 0x12, 0x7f, 0x10 }, // 4   0x34
    { 0x27, 0x45, 0x45, 0x45, 0x39 }, // 5   0x35
    { 0x3c, 0x4a, 0x49, 0x49, 0x30 }, // 6   0x36
    { 0x01, 0x71, 0x09, 0x05, 0x03 }, // 7   0x37
    { 0x36, 0x49, 0x49, 0x49, 0x36 }, // 8   0x38
    { 0x06, 0x49, 0x49, 0x29, 0x1e }, // 9   0x39
    { 0x00, 0x36, 0x36, 0x00, 0x00 }, // :   0x3A
    { 0x00, 0x56, 0x36, 0x00, 0x00 }, // ;   0x3B
    { 0x08, 0x14, 0x22, 0x41, 0x00 }, // <   0x3C
    { 0x14, 0x14, 0x14, 0x14, 0x14 }, // =   0x3D
    { 0x00, 0x41, 0x22, 0x14, 0x08 }, // >   0x3E
    { 0x02, 0x01, 0x51, 0x09, 0x06 }, // ?   0x3F
    { 0x32, 0x49, 0x79, 0x41, 0x3e }, // @   0x40
    { 0x7e, 0x11, 0x11, 0x11, 0x7e }, // A   0x41
    { 0x7f, 0x49, 0x49, 0x49, 0x36 }, // B   0x42
    { 0x3e, 0x41, 0x41, 0x41, 0x22 }, // C   0x43
    { 0x7f, 0x41, 0x41, 0x22, 0x1c }, // D   0x44
    { 0x7f, 0x49, 0x49, 0x49, 0x41 }, // E   0x45
    { 0x7f, 0x09, 0x09, 0x09, 0x01 }, // F   0x46
    { 0x3e, 0x41, 0x49, 0x49, 0x7a }, // G   0x47
    { 0x7f, 0x08, 0x08, 0x08, 0x7f }, // H   0x48
    { 0x00, 0x41, 0x7f, 0x41, 0x00 }, // I   0x49
    { 0x20, 0x40, 0x41, 0x3f, 0x01 }, // J   0x4A
    { 0x7f, 0x08, 0x14, 0x22, 0x41 }, // K   0x4B
    { 0x7f, 0x40, 0x40, 0x40, 0x40 }, // L   0x4C
    { 0x7f, 0x02, 0x0c, 0x02, 0x7f }, // M   0x4D
    { 0x7f, 0x04, 0x08, 0x10, 0x7f }, // N   0x4E
    { 0x3e, 0x41, 0x41, 0x41, 0x3e }, // O   0x4F
    { 0x7f, 0x09, 0x09, 0x09, 0x06 }, // P   0X50
    { 0x3e, 0x41, 0x51, 0x21, 0x5e }, // Q   0X51
    { 0x7f, 0x09, 0x19, 0x29, 0x46 }, // R   0X52
    { 0x46, 0x49, 0x49, 0x49, 0x31 }, // S   0X53
    { 0x01, 0x01, 0x7f, 0x01, 0x01 }, // T   0X54
    { 0x3f, 0x40, 0x40, 0x40, 0x3f }, // U   0X55
    { 0x1f, 0x20, 0x40, 0x20, 0x1f }, // V   0X56
    { 0x3f, 0x40, 0x38, 0x40, 0x3f }, // W   0X57
    { 0x63, 0x14, 0x08, 0x14, 0x63 }, // X   0X58
    { 0x07, 0x08, 0x70, 0x08, 0x07 }, // Y   0X59
    { 0x61, 0x51, 0x49, 0x45, 0x43 }, // Z   0X5A
    { 0x00, 0x7f, 0x41, 0x41, 0x00 }, // [   0X5B
    { 0x02, 0x04, 0x08, 0x10, 0x20 }, // "\" 0X5C
    { 0x00, 0x41, 0x41, 0x7f, 0x00 }, // ]   0X5D
    { 0x04, 0x02, 0x01, 0x02, 0x04 }, // ^   0X5E
    { 0x40, 0x40, 0x40, 0x40, 0x40 }, // _   0X5F
    { 0x00, 0x01, 0x02, 0x04, 0x00 }, // `   0X60
    { 0x20, 0x54, 0x54, 0x54, 0x78 }, // a   0X61
    { 0x7f, 0x48, 0x44, 0x44, 0x38 }, // b   0X62
    { 0x38, 0x44, 0x44, 0x44, 0x20 }, // c   0X63
    { 0x38, 0x44, 0x44, 0x48, 0x7f }, // d   0X64
    { 0x38, 0x54, 0x54, 0x54, 0x18 }, // e   0X65
    { 0x08, 0x7e, 0x09, 0x01, 0x02 }, // f   0X66
    { 0x0c, 0x52, 0x52, 0x52, 0x3e }, // g   0X67
    { 0x7f, 0x08, 0x04, 0x04, 0x78 }, // h   0X68
    { 0x00, 0x44, 0x7d, 0x40, 0x00 }, // i   0X69
    { 0x20, 0x40, 0x44, 0x3d, 0x00 }, // j   0X6A
    { 0x7f, 0x10, 0x28, 0x44, 0x00 }, // k   0X6B
    { 0x00, 0x41, 0x7f, 0x40, 0x00 }, // l   0X6C
    { 0x7c, 0x04, 0x18, 0x04, 0x78 }, // m   0X6D
    { 0x7c, 0x08, 0x04, 0x04, 0x78 }, // n   0X6E
    { 0x38, 0x44, 0x44, 0x44, 0x38 }, // o   0X6F
    { 0x7c, 0x14, 0x14, 0x14, 0x08 }, // p   0X70
    { 0x08, 0x14, 0x14, 0x18, 0x7c }, // q   0X71
    { 0x7c, 0x08, 0x04, 0x04, 0x08 }, // r   0X72
    { 0x48, 0x54, 0x54, 0x54, 0x20 }, // s   0X73
    { 0x04, 0x3f, 0x44, 0x40, 0x20 }, // t   0X74
    { 0x3c, 0x40, 0x40, 0x20, 0x7c }, // u   0X75
    { 0x1c, 0x20, 0x40, 0x20, 0x1c }, // v   0X76
    { 0x3c, 0x40, 0x30, 0x40, 0x3c }, // w   0X77
    { 0x44, 0x28, 0x10, 0x28, 0x44 }, // x   0X78
    { 0x0c, 0x50, 0x50, 0x50, 0x3c }, // y   0X79
    { 0x44, 0x64, 0x54, 0x4c, 0x44 }, // z   0X7A
    { 0x00, 0x08, 0x36, 0x41, 0x00 }, // {   0X7B
    { 0x00, 0x00, 0x7f, 0x00, 0x00 }, // |   0X7C
    { 0x00, 0x41, 0x36, 0x08, 0x00 }, // }   0X7D
    { 0x02, 0x01, 0x02, 0x04, 0x02 }, // ~   0X7E
};











void ST7735_SPI_Initial(void)
{

		nrf_gpio_cfg_output(ST7735_SCK);
		nrf_gpio_cfg_output(ST7735_SDA);
		nrf_gpio_cfg_output(ST7735_D_CX);
		//nrf_gpio_cfg_output(ST7735_RST);
	
		NRF_SPI0->ENABLE=1;
		NRF_SPI0->PSELSCK=ST7735_SCK;
		NRF_SPI0->PSELMOSI=ST7735_SDA;
		NRF_SPI0->PSELMISO=0xffffffff;		// do not connect MISO pin , slave output not required
		NRF_SPI0->FREQUENCY=0x80000000;
		NRF_SPI0->CONFIG=0x00000000;
		
	
}



void ST_SPI_Write(unsigned char num)
{
	NRF_SPI0->TXD=num;
	NRF_SPI0->EVENTS_READY=0;
	while(NRF_SPI0->EVENTS_READY==1);
	
	
	
	
  //nrf_delay_us(1);
	
///*
//	unsigned char t;  
//  unsigned char shiftin=0; 
//  nrf_gpio_pin_clear(ST7735_SCK);
//  for (t=0; t<8; t++) 
//    { 
//      if ((num & 0x80) == 0) 
//			{
//         nrf_gpio_pin_clear(ST7735_SDA);
//			}				
//      else 
//			{
//        nrf_gpio_pin_set(ST7735_SDA); 
//			}
//      nrf_gpio_pin_set(ST7735_SCK);
//			//app_uart_put('o');
//			//while(true);
//						
//      num <<= 1;    //shift left one bit 
//       
//      shiftin <<= 1; //shift left one bit 
//      /*if (nrf_gpio_pin_read(MISO) == 1) 
//			{
//        shiftin |= 1; 
//			}*/
//			//nrf_delay_us(1);
//			nrf_gpio_pin_clear(ST7735_SCK);
//    } 
//    //return shiftin; 

	
	
	
	
}



void ST7735_Write_Cmd(unsigned char Cmd)
{
	nrf_gpio_pin_clear(ST7735_D_CX);

	ST_SPI_Write(Cmd);

	
}




void ST7735_Write_Data(unsigned char data)
{
	
	

	nrf_gpio_pin_set(ST7735_D_CX);
	ST_SPI_Write(data);

	
	
}


 void ST7735_Initial(void)
 {
	 
		nrf_gpio_pin_clear(ST7735_BACKLIGHT);
		nrf_delay_ms(100);
	  nrf_gpio_pin_set(ST7735_BACKLIGHT);
	  nrf_delay_ms(100);	 
//	  ST7735_Write_Cmd(ST7735_SWRESET);//Soft Reset
//    nrf_delay_ms(150);
//	  	 
//	  ST7735_Write_Cmd(ST7735_SWRESET);//Soft Reset
//    nrf_delay_ms(150);
	 
	  ST7735_Write_Cmd(ST7735_SWRESET);//Soft Reset
    nrf_delay_ms(150);
	 
    ST7735_Write_Cmd(ST7735_SLPOUT);//Sleep exit
		nrf_delay_ms (150);
	 
		ST7735_Write_Cmd(0x38);
		ST7735_Write_Cmd(ST7735_DISPOFF);
	 
	  ST7735_Write_Cmd(ST7735_FRMCTR1);
    ST7735_Write_Data(0x01);
	  ST7735_Write_Data(0x3c); 
	  ST7735_Write_Data(0x3c);
	 	  
	 
	  ST7735_Write_Cmd(ST7735_FRMCTR2);
		ST7735_Write_Data(0x05);
	  ST7735_Write_Data(0x3C);
		ST7735_Write_Data(0x3c);
	 
	  ST7735_Write_Cmd(ST7735_DISSET5);
		ST7735_Write_Data(0x15);
	  ST7735_Write_Data(0x02);
	 
	 
    ST7735_Write_Cmd(ST7735_FRMCTR3);
		ST7735_Write_Data(0x05); 
		ST7735_Write_Data(0x3c); 
		ST7735_Write_Data(0x3c); 
		ST7735_Write_Data(0x05); 
		ST7735_Write_Data(0x3c); 
		ST7735_Write_Data(0x3c); 

		
		//ST7735_Write_Cmd(ST7735_FRMCTR3);
		
		
	 
	  ST7735_Write_Cmd(ST7735_INVCTR); //Column inversion
    ST7735_Write_Data(0x07);

		
	 
	  ST7735_Write_Cmd(ST7735_PWCTR1); 
		ST7735_Write_Data(0xb4); 
		ST7735_Write_Data(0x14); 
		ST7735_Write_Data(0x04); 
				
		
		
		ST7735_Write_Cmd(ST7735_PWCTR2); 
    ST7735_Write_Data(0xc0);	
		
		ST7735_Write_Cmd(ST7735_PWCTR3); 
    ST7735_Write_Data(0x0a);	
	  ST7735_Write_Data(0x00);	
		
		ST7735_Write_Cmd(ST7735_PWCTR4); 
    ST7735_Write_Data(0x8a);	
	  ST7735_Write_Data(0x2a);	
	  		
	  ST7735_Write_Cmd(ST7735_PWCTR5); 
    ST7735_Write_Data(0x8a);	
		ST7735_Write_Data(0xaa);	
		
		ST7735_Write_Cmd(ST7735_VMCTR1);
		ST7735_Write_Data(0x28);	
		
	  
		ST7735_Write_Cmd(ST7735_MADCTL); //MX, MY, RGB mode
    ST7735_Write_Data(0xc8);
		
		ST7735_Write_Cmd(ST7735_COLMOD); //Color Mode = 65k mode
    ST7735_Write_Data(0x05);
		
	  ST7735_Write_Cmd(ST7735_CASET); // Colum Address Set
    ST7735_Write_Data(0x00); 
		ST7735_Write_Data(0x00);
    ST7735_Write_Data(0x00); 
	  ST7735_Write_Data(0x80);
		
		ST7735_Write_Cmd(ST7735_RASET); //Row Address Set
    ST7735_Write_Data(0x00);
		ST7735_Write_Data(0x00);
    ST7735_Write_Data(0x00);
		ST7735_Write_Data(0x80);
		
		
		ST7735_Write_Cmd(ST7735_GMCTRP1);
    ST7735_Write_Data(0x02);
    ST7735_Write_Data(0x1C);
    ST7735_Write_Data(0x07);
    ST7735_Write_Data(0x12);
    ST7735_Write_Data(0x37);
    ST7735_Write_Data(0x32);
    ST7735_Write_Data(0x29);
    ST7735_Write_Data(0x2D);
    ST7735_Write_Data(0x29);
    ST7735_Write_Data(0x25);
    ST7735_Write_Data(0x2B);
    ST7735_Write_Data(0x39);
    ST7735_Write_Data(0x00);
    ST7735_Write_Data(0x01);
    ST7735_Write_Data(0x03);
    ST7735_Write_Data(0x10);
		
		ST7735_Write_Cmd(ST7735_GMCTRN1);
    ST7735_Write_Data(0x03);
    ST7735_Write_Data(0x1d);
    ST7735_Write_Data(0x07);
    ST7735_Write_Data(0x06);
    ST7735_Write_Data(0x2E);
    ST7735_Write_Data(0x2c);
    ST7735_Write_Data(0x29);
    ST7735_Write_Data(0x2d);
    ST7735_Write_Data(0x2E);
    ST7735_Write_Data(0x2E);
    ST7735_Write_Data(0x37);
    ST7735_Write_Data(0x3f);
    ST7735_Write_Data(0x00);
    ST7735_Write_Data(0x00);
    ST7735_Write_Data(0x02);
    ST7735_Write_Data(0x10);
		
		ST7735_Write_Cmd(ST7735_NORON);
		nrf_delay_ms(10);
		ST7735_Write_Cmd(ST7735_DISPON);
		nrf_delay_ms(100);
		ST7735_Write_Cmd(ST7735_RAMWR);	
	 /*
	 
ST7735_Write_Cmd(0x11);   

nrf_delay_ms(120);      

ST7735_Write_Cmd(0xB1);   
ST7735_Write_Data(0x01);   //05	  02
ST7735_Write_Data(0x3c); 
ST7735_Write_Data(0x3c); 

ST7735_Write_Cmd(0xB2);   
ST7735_Write_Data(0x05); 
ST7735_Write_Data(0x3c); 
ST7735_Write_Data(0x3c); 

ST7735_Write_Cmd(0xB3);   
ST7735_Write_Data(0x05); 
ST7735_Write_Data(0x3c); 
ST7735_Write_Data(0x3c); 
ST7735_Write_Data(0x05); 
ST7735_Write_Data(0x3c); 
ST7735_Write_Data(0x3c); 

ST7735_Write_Cmd(0xB4);   
ST7735_Write_Data(0x07);//07 

ST7735_Write_Cmd(0xb6);   
ST7735_Write_Data(0x84); 
ST7735_Write_Data(0xf0); 
ST7735_Write_Data(0x20); 

ST7735_Write_Cmd(0xC0);   
ST7735_Write_Data(0xb4); 
ST7735_Write_Data(0x14); 
ST7735_Write_Data(0x04); 

ST7735_Write_Cmd(0xC1);   
ST7735_Write_Data(0xc0); 

ST7735_Write_Cmd(0xC2);   
ST7735_Write_Data(0x0a); 
ST7735_Write_Data(0x00); 

ST7735_Write_Cmd(0xC3);   
ST7735_Write_Data(0x8a); 
ST7735_Write_Data(0x2a); 

ST7735_Write_Cmd(0xC4);   
ST7735_Write_Data(0x8a); 
ST7735_Write_Data(0xaa); 

ST7735_Write_Cmd(0xC5);   
ST7735_Write_Data(0x28); //00  0F	14  11

ST7735_Write_Cmd(0xE0);   
ST7735_Write_Data(0x03); 
ST7735_Write_Data(0x22); 
ST7735_Write_Data(0x0a); 
ST7735_Write_Data(0x12); 
ST7735_Write_Data(0x3a); 
ST7735_Write_Data(0x35); 
ST7735_Write_Data(0x2e); 
ST7735_Write_Data(0x31); 
ST7735_Write_Data(0x30); 
ST7735_Write_Data(0x2d); 
ST7735_Write_Data(0x34); 
ST7735_Write_Data(0x3d); 
ST7735_Write_Data(0x00); 
ST7735_Write_Data(0x00); 
ST7735_Write_Data(0x01); 
ST7735_Write_Data(0x03); 

ST7735_Write_Cmd(0xE1);   
ST7735_Write_Data(0x03); 
ST7735_Write_Data(0x20); 
ST7735_Write_Data(0x0a); 
ST7735_Write_Data(0x12); 
ST7735_Write_Data(0x2e); 
ST7735_Write_Data(0x2a); 
ST7735_Write_Data(0x26); 
ST7735_Write_Data(0x2b); 
ST7735_Write_Data(0x2b); 
ST7735_Write_Data(0x2a); 
ST7735_Write_Data(0x32); 
ST7735_Write_Data(0x3d); 
ST7735_Write_Data(0x00); 
ST7735_Write_Data(0x01); 
ST7735_Write_Data(0x02); 
ST7735_Write_Data(0x04); 

ST7735_Write_Cmd(0x36);   
ST7735_Write_Data(0xC8);  //c8

ST7735_Write_Cmd(0x2A);   
ST7735_Write_Data(0x00); 
ST7735_Write_Data(0x00); 
ST7735_Write_Data(0x00); 
ST7735_Write_Data(0x7F); 

ST7735_Write_Cmd(0x2B);   
ST7735_Write_Data(0x00); 
ST7735_Write_Data(0x00); 
ST7735_Write_Data(0x00); 
ST7735_Write_Data(0x7F); 

ST7735_Write_Cmd(0xf0);   
ST7735_Write_Data(0x01); 
ST7735_Write_Cmd(0xf6);   
ST7735_Write_Data(0x00); 

//ST7735_Write_Cmd(0x21);   

ST7735_Write_Cmd(0x3A);   
ST7735_Write_Data(0x05); 

ST7735_Write_Cmd(0x29);   
ST7735_Write_Cmd(0x2C);
	 */
	 
	 
 }
 
 
 
 
void SetAddrWindow(unsigned char x0 , unsigned char y0, unsigned char x1, unsigned char y1)
 {
	  ST7735_Write_Cmd(ST7735_CASET); // Column addr set
    ST7735_Write_Data(0x00);
    ST7735_Write_Data(x0);     // XSTART
    ST7735_Write_Data(0x00);
    ST7735_Write_Data(x1);     // XEND

    ST7735_Write_Cmd(ST7735_RASET); // Row addr set
    ST7735_Write_Data(0x00);
    ST7735_Write_Data(y0);     // YSTART
    ST7735_Write_Data(0x00);
    ST7735_Write_Data(y1);     // YEND
		ST7735_Write_Cmd(ST7735_RAMWR);
		x_loca=x0;
		y_loca=y0;
	 
 }





void clear_display(void)
{
	fill_display(0,0,130,132,ST7735_BLACK);
	
}


void fill_display(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned int color)
{
	SetAddrWindow(x0, y0, x1, y1);
	for(int i=0;i<((x1-x0)*(y1-y0)); i++)
	{
		
		ST7735_Write_Data(color>>8);
		ST7735_Write_Data(color);
	}
		
	
}


void ST7735_PutChar1X(unsigned char ch,unsigned int color,unsigned int background)
{
	
	const unsigned char *ptr;
	
	ptr=&(FONT[ch-0X20][0]);	
	
			
			
	for(int j=0;j<7;j++)
	{	
		SetAddrWindow(x_loca, y_loca, 129, 130);
			for(int i=0;i<5;i++)
			{
			
				if(((*ptr>>j)&0x01)!=0)
				{
					ST7735_Write_Data(color>>8);
					ST7735_Write_Data(color);
				}
				else{
				ST7735_Write_Data(background>>8);
				ST7735_Write_Data(background);
				}
			
				ptr++;
				
			}
			ptr-=5;
			y_loca++;
	
	
		}
	
	y_loca-=7;
	x_loca+=6;	
		
		if(x_loca>127)
		{
			x_loca=4;
			y_loca+=15;
		}
		
		
	
}



void ST7735_PutChar2X(unsigned char ch,unsigned int color,unsigned int background)
{

	const unsigned char *ptr;
	
	ptr=&(FONT[ch-0X20][0]);	
	
			
			
	for(int j=0;j<7;j++)
	{	
		SetAddrWindow(x_loca, y_loca, 129, 130);
			for(int i=0;i<5;i++)
			{
			
				if(((*ptr>>j)&0x01)!=0)
				{
					ST7735_Write_Data(color>>8);
					ST7735_Write_Data(color);
				}
				else{
				ST7735_Write_Data(background>>8);
				ST7735_Write_Data(background);
				}
			
				ptr++;
				
			}
			ptr-=5;
			y_loca++;
			
					SetAddrWindow(x_loca, y_loca, 129, 130);
			for(int i=0;i<5;i++)
			{
			
				if(((*ptr>>j)&0x01)!=0)
				{
					ST7735_Write_Data(color>>8);
					ST7735_Write_Data(color);
				}
				else{
				ST7735_Write_Data(background>>8);
				ST7735_Write_Data(background);
				}
			
				ptr++;
				
			}
			ptr-=5;
			y_loca++;
			
			
	
	
		}
	
	y_loca-=14;
	x_loca+=6;	
		
		if(x_loca>128)
		{
			x_loca=0;
			y_loca+=8;
		}
		
	
	
	
	
	
	
	
	
}


void ST7735_PutChar3X(unsigned char ch,unsigned int color,unsigned int background)
{
	
	const unsigned char *ptr;
	
	ptr=&(FONT[ch-0X20][0]);	
	
		for(int j=0;j<7;j++)
	{	
		SetAddrWindow(x_loca, y_loca, 129, 130);
			for(int i=0;i<5;i++)
			{
			
				if(((*ptr>>j)&0x01)!=0)
				{
					ST7735_Write_Data(color>>8);
					ST7735_Write_Data(color);
					ST7735_Write_Data(color>>8);
					ST7735_Write_Data(color);
				}
				else{
				ST7735_Write_Data(background>>8);
				ST7735_Write_Data(background);
				ST7735_Write_Data(background>>8);
				ST7735_Write_Data(background);
				}
			
				ptr++;
				
			}
			ptr-=5;
			y_loca++;
			
					SetAddrWindow(x_loca, y_loca, 129, 130);
			for(int i=0;i<5;i++)
			{
			
				if(((*ptr>>j)&0x01)!=0)
				{
					ST7735_Write_Data(color>>8);
					ST7735_Write_Data(color);
						ST7735_Write_Data(color>>8);
					ST7735_Write_Data(color);
				}
				else{
				ST7735_Write_Data(background>>8);
				ST7735_Write_Data(background);
					ST7735_Write_Data(background>>8);
				ST7735_Write_Data(background);
				}
			
				ptr++;
				
			}
			ptr-=5;
			y_loca++;
			
			
	
	
		}
	
	y_loca-=14;
	x_loca+=12;	
		
		if(x_loca>128)
		{
			x_loca=0;
			y_loca+=8;
		}	
			
	
		
		
	
}



void ST7735_PutChar4X(unsigned char ch,unsigned int color,unsigned int background)
{

	const unsigned char *ptr;
	
	ptr=&(FONT[ch-0X20][0]);	
	
		for(int j=0;j<7;j++)
	{	
		SetAddrWindow(x_loca, y_loca, 129, 130);
			for(int i=0;i<5;i++)
			{
			
				if(((*ptr>>j)&0x01)!=0)
				{
					for(int k=0;k<16;k++)
					{
					ST7735_Write_Data(color>>8);
					ST7735_Write_Data(color);
					}
					

				}
				else{
				
					for(int k=0;k<16;k++)
					{
					ST7735_Write_Data(background>>8);
					ST7735_Write_Data(background);	
					}
				}
			
				ptr++;
				
			}
			for(int l=0;l<15;l++)
			{
			ptr-=5;
			y_loca+=1;
	
			
			SetAddrWindow(x_loca, y_loca, 129, 130);
			for(int i=0;i<5;i++)
			{
			
				if(((*ptr>>j)&0x01)!=0)
				{for(int k=0;k<16;k++)
					{
					ST7735_Write_Data(color>>8);
					ST7735_Write_Data(color);
					}
					
					
				}
				else{
				
					for(int k=0;k<16;k++)
					{
					ST7735_Write_Data(background>>8);
					ST7735_Write_Data(background);	
					}
				}
			
				ptr++;
				
			
			}
		}
			
		
			
			
			
			
			
			
			ptr-=5;
			y_loca++;
			
			
			
			//	while(true);
	
	
		}
	
	y_loca-=14;
	x_loca+=12;	
		
		if(x_loca>128)
		{
			x_loca=0;
			y_loca+=8;
		}	
			
	
	
	
	
	
}




  void ST7735_Put_String1X(const char *str,unsigned int color,unsigned int background)
{                                   //OLED STRING DISPLAY
        while(*str)ST7735_PutChar1X(*str++,color,background);
}


  void ST7735_Put_String2X(const char *str,unsigned int color,unsigned int background)
{                                   //OLED STRING DISPLAY
        while(*str)ST7735_PutChar2X(*str++,color,background);
}


  void ST7735_Put_String3X(const char *str,unsigned int color,unsigned int background)
{                                   //OLED STRING DISPLAY
        while(*str)ST7735_PutChar3X(*str++,color,background);
}
 
 
	
 void ST7735_Put_String4X(const char *str,unsigned int color,unsigned int background)
{                                   //OLED STRING DISPLAY
        while(*str)ST7735_PutChar4X(*str++,color,background);
}




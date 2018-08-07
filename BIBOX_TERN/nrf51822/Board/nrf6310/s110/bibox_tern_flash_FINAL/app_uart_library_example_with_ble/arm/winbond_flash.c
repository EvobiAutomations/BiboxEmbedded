
#include "winbond_flash.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"



void Winbond_SPI_Initial(void)
{
	nrf_gpio_cfg_output(FLASH_SCK);
	nrf_gpio_cfg_output(FLASH_SDO);
	nrf_gpio_cfg_output(FLASH_CS);
	nrf_gpio_cfg_input(FLASH_SDI,NRF_GPIO_PIN_PULLUP);
	#ifdef USE_SPI_1
	
		NRF_SPI1->PSELMISO=FLASH_SDI;
		NRF_SPI1->PSELMOSI=FLASH_SDO;
		NRF_SPI1->PSELSCK=FLASH_SCK;
		NRF_SPI1->CONFIG=0;
		NRF_SPI1->FREQUENCY=0x80000000;
		NRF_SPI1->ENABLE=1;
		nrf_gpio_pin_set(FLASH_CS);
		#elif USE_SPI_0
	
	
	
	#endif
	
}




unsigned char SPI_Write_Read(unsigned char data)
{
	unsigned char val;
	NRF_SPI1->TXD=data;
  nrf_delay_us(1);
	//while(NRF_SPI0->EVENTS_READY==1);
	
	NRF_SPI1->EVENTS_READY=0;
	val=NRF_SPI1->RXD;
	
	return val;
	
	
}



void Winbond_Write_Enable(void)
{
	nrf_gpio_pin_clear(FLASH_CS);
	SPI_Write_Read(SPIFLASH_WRITEENABLE);
	nrf_gpio_pin_set(FLASH_CS);
	
	
}



bool Winbond_Flash_Busy(void)
{
	auto bool flag;
	nrf_gpio_pin_clear(FLASH_CS);
	SPI_Write_Read(SPIFLASH_STATUSREAD);
	if((SPI_Write_Read(0)&0x01)==0x00){
		
		flag=false;
	}
	else{
		
		flag=true;
	}
	
	
	nrf_gpio_pin_set(FLASH_CS);
	return flag;
	
	
}




void Winbond_Release_Powerdown(void)
{
	
	nrf_gpio_pin_clear(FLASH_CS);
	SPI_Write_Read(SPIFLASH_WAKE);
	nrf_gpio_pin_set(FLASH_CS);
	
	
	
}


void Winbond_Chip_Erase(void)
{
	
	nrf_gpio_pin_clear(FLASH_CS);
	SPI_Write_Read(SPIFLASH_CHIPERASE);
	nrf_gpio_pin_set(FLASH_CS);
	
}


void Winbond_Write(uint32_t addr ,unsigned int no_data,const uint16_t * buf )
	
{
	nrf_gpio_pin_clear(FLASH_CS);
	SPI_Write_Read(SPIFLASH_BYTEPAGEPROGRAM);
	SPI_Write_Read(addr>>16);
	SPI_Write_Read(addr>>8);
	SPI_Write_Read(addr);
	
		for(int i=0;i<no_data/2;i++)
		{
			SPI_Write_Read(*(buf+i)>>8);
			SPI_Write_Read(*(buf+i));
		}
	nrf_gpio_pin_set(FLASH_CS);
	
	
	
}



void Winbond_Read(uint32_t addr ,unsigned int no_data,uint8_t * buf )
{
	
	nrf_gpio_pin_clear(FLASH_CS);
	SPI_Write_Read(SPIFLASH_ARRAYREADLOWFREQ);
	SPI_Write_Read(addr>>16);
	SPI_Write_Read(addr>>8);
	SPI_Write_Read(addr);
	
		for(int i=0;i<no_data;i++)
		{
			buf[i]=SPI_Write_Read(0);
		}
	nrf_gpio_pin_set(FLASH_CS);
	
	
}



void Winbond_Chip_32kErase(uint32_t addr)
{
	nrf_gpio_pin_clear(FLASH_CS);
	SPI_Write_Read(SPIFLASH_BLOCKERASE_32K);
	SPI_Write_Read(addr>>16);
	SPI_Write_Read(addr>>8);
	SPI_Write_Read(addr);
	
	nrf_gpio_pin_set(FLASH_CS);
	
	
}



void Winbond_Power_Down(void)
{
	nrf_gpio_pin_clear(FLASH_CS);
	SPI_Write_Read(SPIFLASH_SLEEP);
	nrf_gpio_pin_set(FLASH_CS);
	
}



bool Winbond_Check_Communication(void)
{
	
	
	  nrf_gpio_pin_clear(FLASH_CS);
		SPI_Write_Read(0x9f);
		if(SPI_Write_Read(0)!=0xff)
		{
			return true;
		}
		else{
			return false;
		}
		
	
	
}



void Winbond_Write_8bit(uint32_t addr ,unsigned int no_data,unsigned char * buf )
	
{
	nrf_gpio_pin_clear(FLASH_CS);
	SPI_Write_Read(SPIFLASH_BYTEPAGEPROGRAM);
	SPI_Write_Read(addr>>16);
	SPI_Write_Read(addr>>8);
	SPI_Write_Read(addr);
	
		for(int i=0;i<no_data;i++)
		{			
			SPI_Write_Read(*(buf+i));
		}
	nrf_gpio_pin_set(FLASH_CS);
	
	
	
}


void Winbond_Send_Address(uint32_t addr)
{
	
	
	SPI_Write_Read(SPIFLASH_ARRAYREADLOWFREQ);
	SPI_Write_Read(addr>>16);
	SPI_Write_Read(addr>>8);
	SPI_Write_Read(0);
	
}
	
	
	
	
	



#include "soft_spi.h"
#include "nrf_gpio.h"


void Soft_Spi_Initial(void)
{
	nrf_gpio_cfg_output(SOFT_SPI_SCK);
	nrf_gpio_cfg_output(SOFT_SPI_MOSI);
	nrf_gpio_cfg_input(SOFT_SPI_MISO,NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_output(SOFT_SPI_CS);
	nrf_gpio_pin_set(SOFT_SPI_CS);
}




unsigned char Soft_Spi_Write_Read(unsigned char num)
{
	
	unsigned char t;  
  unsigned char shiftin=0; 
  nrf_gpio_pin_clear(SOFT_SPI_SCK);
  for (t=0; t<8; t++) 
    { 
      if ((num & 0x80) == 0) 
			{
         nrf_gpio_pin_clear(SOFT_SPI_MOSI);
			}				
      else 
			{
        nrf_gpio_pin_set(SOFT_SPI_MOSI); 
			}
      nrf_gpio_pin_set(SOFT_SPI_SCK);
			
						
      num <<= 1;    //shift left one bit 
       
      shiftin <<= 1; //shift left one bit 
      if (nrf_gpio_pin_read(SOFT_SPI_MISO) == 1) 
			{
        shiftin |= 1; 
			}
			
			nrf_gpio_pin_clear(SOFT_SPI_SCK);
    } 
    return shiftin; 
	

}



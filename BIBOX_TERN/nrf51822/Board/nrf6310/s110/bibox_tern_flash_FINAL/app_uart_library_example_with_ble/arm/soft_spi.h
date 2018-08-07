#ifndef SOFT_SPI
#define SOFT_SPI



#include "bibox.h"
#include "nrf_gpio.h"



#define SOFT_SPI_SCK 			B2
#define SOFT_SPI_MOSI 		B1
#define SOFT_SPI_MISO 		B3
#define SOFT_SPI_CS				B4



void Soft_Spi_Initial(void);





unsigned char Soft_Spi_Write_Read(unsigned char num);









#endif



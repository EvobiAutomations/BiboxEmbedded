
#ifndef WINBOND_FLASH
	#define WINBOND_FLASH

#include <stdbool.h>
#include <stdint.h>

#define	USE_SPI_1


#define FLASH_SCK 16
#define FLASH_SDO 18
#define FLASH_SDI 17
#define FLASH_CS  19

#define SPI_RX_BUF_SIZE		20


#define SPIFLASH_WRITEENABLE      0x06        // write enable
#define SPIFLASH_WRITEDISABLE     0x04        // write disable

#define SPIFLASH_BLOCKERASE_4K    0x20        // erase one 4K block of flash memory
#define SPIFLASH_BLOCKERASE_32K   0x52        // erase one 32K block of flash memory
#define SPIFLASH_BLOCKERASE_64K   0xD8        // erase one 64K block of flash memory
#define SPIFLASH_CHIPERASE        0x60        // chip erase (may take several seconds depending on size)
                                              // but no actual need to wait for completion (instead need to check the status register BUSY bit)
#define SPIFLASH_STATUSREAD       0x05        // read status register
#define SPIFLASH_STATUSWRITE      0x01        // write status register
#define SPIFLASH_ARRAYREAD        0x0B        // read array (fast, need to add 1 dummy byte after 3 address bytes)
#define SPIFLASH_ARRAYREADLOWFREQ 0x03        // read array (low frequency)

#define SPIFLASH_SLEEP            0xB9        // deep power down
#define SPIFLASH_WAKE             0xAB        // deep power wake up
#define SPIFLASH_BYTEPAGEPROGRAM  0x02        // write (1 to 256bytes)
#define SPIFLASH_IDREAD           0x9F        // read JEDEC manufacturer and device ID (2 bytes, specific bytes for each manufacturer and device)
                                              // Example for Atmel-Adesto 4Mbit AT25DF041A: 0x1F44 (page 27: http://www.adestotech.com/sites/default/files/datasheets/doc3668.pdf)
                                              // Example for Winbond 4Mbit W25X40CL: 0xEF30 (page 14: http://www.winbond.com/NR/rdonlyres/6E25084C-0BFE-4B25-903D-AE10221A0929/0/W25X40CL.pdf)
#define SPIFLASH_MACREAD          0x4B        // read unique ID number (MAC)



/****************************************************************** 
		Initialize the SPI module of nordic 51822 .
		o select the SPI0 or SPI1 change	#define	USE_SPI_0 		 
******************************************************************/	 

void Winbond_SPI_Initial(void);




/*******************************************************************
		SPI write read function-- 
		Writes one byte to the SDO line and returns the data on SDI line
*******************************************************************/

unsigned char SPI_Write_Read(unsigned char );




/******************************************************************
		Set the write enable bit. Should be called before every write
		and erase commands.																										
******************************************************************/

void Winbond_Write_Enable(void);



/******************************************************************
		Checks the status of flash IC. Returns 'true' if the IC is busy
			else returns false.																	
******************************************************************/

bool Winbond_Flash_Busy(void);



/******************************************************************
		Wajke up from the power down mode
******************************************************************/

void Winbond_Release_Powerdown(void);



/******************************************************************
		Erase the entire memory
******************************************************************/

void Winbond_Chip_Erase(void);



/******************************************************************
		Write data to the flash.
		@param	1.  24bits Block address 
						2.	No of bytes	to written. MAX value 256
						3.	Pointer to the data array (16 bit RGB val)
******************************************************************/

void Winbond_Write(uint32_t ,unsigned int,const uint16_t * );




/******************************************************************
		Read data from the flash
		@param	1.	24bit block address
						2		No. of bytes to be read.
						3.	Pointer to the destination buffer. Buffer should be
								big enough to fit the no. of data.
******************************************************************/

void Winbond_Read(uint32_t ,unsigned int,uint8_t * );




/*****************************************************************
		Erase the  32k bytes of block memory .
		@param	. 24bit block address.

*****************************************************************/


void Winbond_Chip_32kErase(uint32_t);



/****************************************************************
		Put flash memoy into power down mode

*****************************************************************/

void Winbond_Power_Down(void);



#endif




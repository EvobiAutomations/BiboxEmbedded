#ifndef ST7735
#define  ST7735
#define USE_NRF_SPI0

/* ================================================ */
/* ============== PIN Defnitions	================= */
/* ================================================	*/

#define ST7735_SCK  			20
#define ST7735_SDA				28
#define ST7735_D_CX 			29
#define ST7735_BACKLIGHT	30
//#define ST7735_CS					4


/* ================================================ */
/* ================================================ */
/* ================================================ */

#define ST7735_NOP     0x00
#define ST7735_SWRESET 0x01
#define ST7735_RDDID   0x04
#define ST7735_RDDST   0x09

#define ST7735_SLPIN   0x10
#define ST7735_SLPOUT  0x11
#define ST7735_PTLON   0x12
#define ST7735_NORON   0x13

#define ST7735_INVOFF  0x20
#define ST7735_INVON   0x21
#define ST7735_DISPOFF 0x28
#define ST7735_DISPON  0x29
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_RAMRD   0x2E

#define ST7735_PTLAR   0x30
#define ST7735_COLMOD  0x3A
#define ST7735_MADCTL  0x36

#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR  0xB4
#define ST7735_DISSET5 0xB6

#define ST7735_PWCTR1  0xC0
#define ST7735_PWCTR2  0xC1
#define ST7735_PWCTR3  0xC2
#define ST7735_PWCTR4  0xC3
#define ST7735_PWCTR5  0xC4
#define ST7735_VMCTR1  0xC5

#define ST7735_RDID1   0xDA
#define ST7735_RDID2   0xDB
#define ST7735_RDID3   0xDC
#define ST7735_RDID4   0xDD

#define ST7735_PWCTR6  0xFC

#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

// Color definitions
#define	ST7735_BLACK   0x0000
#define	ST7735_BLUE    0x001F
#define	ST7735_RED     0xF800
#define	ST7735_GREEN   0x07E0
#define ST7735_CYAN    0x07FF
#define ST7735_MAGENTA 0xF81F
#define ST7735_YELLOW  0xFFE0
#define ST7735_WHITE	 0xFFFF


/************************************************************************
				Wrtie one byte to the SPI
************************************************************************/

void ST_SPI_Write(unsigned char);



/************************************************************************
				Write Command to the display
************************************************************************/

void ST7735_Write_Cmd(unsigned char );



/************************************************************************
				Write one byte of data to display
************************************************************************/

void ST7735_Write_Data(unsigned char);



/************************************************************************
				Configure the SPI for Display
				SPI mode 0, 8MHz clock
************************************************************************/
void ST7735_SPI_Initial(void);



/************************************************************************
				Initializes the Display
************************************************************************/

void ST7735_Initial(void);



/************************************************************************
			Set address window
			@param 1.column start address
						 2.Row start address
						 3.Column end address
						 4.Row end address
************************************************************************/

void SetAddrWindow(unsigned char , unsigned char, unsigned char , unsigned char );




/************************************************************************
       Fill display with black color
************************************************************************/

void clear_display(void);




/***********************************************************************
			Fill display with color
			@param 1,2,3== Address window
						 4. Color value (2 byte 5-6-5 format
***********************************************************************/

			
void fill_display(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned int color);




/***********************************************************************
			Print one char with small font
			@param 1. char (ASCII value)
						 2. Character color
						 3. Character backgorund color
***********************************************************************/

void ST7735_PutChar1X(unsigned char ch,unsigned int color,unsigned int background);




/***********************************************************************
			Print one char with big font
			@param 1. char (ASCII value)
						 2. Character color
						 3. Character backgorund color
***********************************************************************/

void ST7735_PutChar2X(unsigned char ch,unsigned int color,unsigned int background);




/***********************************************************************
			Print one char with double size
			@param 1. char (ASCII value)
						 2. Character color
						 3. Character backgorund color
***********************************************************************/

void ST7735_PutChar3X(unsigned char ch,unsigned int color,unsigned int background);




/***********************************************************************
			Print string with small font
			@param 1. char (ASCII value)
						 2. Character color
						 3. Character backgorund color
***********************************************************************/

void ST7735_Put_String1X(const char *str,unsigned int color,unsigned int background);




/***********************************************************************
			Print string with big font
			@param 1. char (ASCII value)
						 2. Character color
						 3. Character backgorund color
***********************************************************************/

void ST7735_Put_String2X(const char *str,unsigned int color,unsigned int background);





/***********************************************************************
			Print string with double size
			@param 1. char (ASCII value)
						 2. Character color
						 3. Character backgorund color
***********************************************************************/

void ST7735_Put_String3X(const char *str,unsigned int color,unsigned int background);


void ST7735_Put_String4X(const char *str,unsigned int color,unsigned int background);

void ST7735_PutChar4X(unsigned char ch,unsigned int color,unsigned int background);

#endif




#ifndef MAX_7219
#define MAX_7219

#include <stdbool.h>
#include "bibox.h"


#define MAX_CS  B4


//////////////////////////////////////////////////
//////////////////////////////////////////////////


#define NO_DECODE 				0x00
#define DIGI0_DECODE 			0x01
#define DIGI3_DECODE			0x0f
#define DIGI7_DECODE			0xff


//////////////////////////////////////////////////
//////////////////////////////////////////////////




/************** initialize the SPI gpio lines ******************/
void Max_7219_Initial(void);




/************** Write one byte of data to spi ******************/
void Max_Write_Byte(unsigned char );



/************** Dispaly test ************************************
***************@param 1:true : all leds on
											false: normal mode
											2: No. of displays
****************************************************************/
void Max_Display_test(bool ,char);




/************** display shut down *******************************
*************** @param 1: true: shutdwown **************************
											    false: normal 
											 2: No. of displays		
****************************************************************/								
void Max_Shutdown(bool,char);




/*************** decode mode configuration **********************
****************************************************************/
void Max_Decode_Mode(unsigned char ,char);





/**************** intensity adjustment @param 0x00-0x0f *********
****************************************************************/
void Max_Intensity(unsigned char,char len );





/**************** Scan limit @prarm 0x00-0x07 *******************
****************************************************************/
void Max_Scan_Limit(unsigned char,char);






/**************** Display data @param 1. col address 2.val 3. No. of display *****
*********************************************************************************/
void Max_Display_Data(unsigned char,unsigned char,char len);





/**************** clear display *******************************/
void Max_Display_Clear(char);


void Max_Display_Data_2X(unsigned char addr,unsigned char val1,unsigned char val2);



void Put_Char_OLED_1X( unsigned char X);



void Put_Char_OLED_2X( unsigned char X,unsigned char Y);



/************** Send char to dispalay ************************************
***************@param data: pointer to the data buffer
								len: No. of displays connected
*************************************************************************/

void Max_Display_Char_X(unsigned char *data,unsigned char len);


#endif

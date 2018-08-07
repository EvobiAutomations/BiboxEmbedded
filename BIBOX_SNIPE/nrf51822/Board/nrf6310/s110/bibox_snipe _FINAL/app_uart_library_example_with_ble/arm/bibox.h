

#ifndef BIBOX
#define BIBOX
#include "nrf_gpio.h"
#include <stdbool.h>

#define _8BIT 0
#define _9BIT 1
#define _10BIT	2

#define RED 0
#define GREEN 1
#define BLUE 	2

#define CLOCKWISE 0
#define ANTICLOCKWISE 1

#define LOW 0
#define HIGH 1

#define NO_PULL 	0x00UL
#define PULL_DOWN 0x01UL
#define PULL_UP		0x03UL

#define FONT_1 0
#define FONT_2 1
#define FONT_3 2


#define ADC_CHANNEL_1 1UL
#define ADC_CHANNEL_2 2UL
#define ADC_CHANNEL_3 4UL	
#define ADC_CHANNEL_4 8UL	
#define ADC_CHANNEL_5 16UL	
#define ADC_CHANNEL_6 32UL	
#define ADC_CHANNEL_7 64UL
#define ADC_CHANNEL_8 128UL




/* ======================================================================================= */
/* ================     BIBOX PORT DEFINITONS   ========================================== */
/* ======================================================================================= */
	#define A1 1
  #define A2 2
	#define A3 3
  #define A4 4
 // #define A5 3	
 // #define A6 4	

	#define B1 7
  #define B2 5
	#define B3 6
  #define B4 26
 
	#define C1 8
  #define C2 9
	#define C3 10
  #define C4 11
	
	#define F1 12
  #define F2 13
	//#define F3 11
  //#define F4 12


	#define G1 14
  #define G2 15
	//#define G3 3
  //#define G4 4

	#define M1 21
  #define M2 22
	#define M3 23
  #define M4 24

	//#define TX F4
  //#define RX F3

	
	
	#define TRIGGER 		F1
	#define ECHO				F2	

	#define TRIGGER1 		G1
	#define ECHO1				G2

	#define Reg_Heat			6
	#define	Prog_Pin			25
	#define	LCD_BACKLIGHT	ST7735_BACKLIGHT
	#define OVER_HEAT	0
	#define Bat_pin	27


	#define BIBOX_RX_PIN_NUMBER  F1
	#define BIBOX_TX_PIN_NUMBER  F2
/***************************************************/
/****** TFT pins are defined in the ST7735.h *******/
/******	Flash ic pins are defined in the winbond.h */
/***************************************************/






	
/******************** digital pin operations ***********************************/
/********************************************************************************
Configure PORT as Input

@ param 
1: Pin Number	(Accepted values [A1-A4,B1-B4,C1-C4, F1-F2, G1-G2] )
2: Pull UP		(Accepted values [NO_PULL, PULL_DOWN, PULL_UP]

Returns Nothing
********************************************************************************/

void SET_AS_INPUT(unsigned char pin,unsigned char pull);



/********************************************************************************
Configure PORT as Output

@ param 
1: Pin Number	(Accepted values [A1-A4,B1-B4,C1-C4, F1-F2, G1-G2] )

Returns Nothing
********************************************************************************/

void SET_AS_OUTPUT(unsigned char pin);


/********************************************************************************
Make a Pulse 

@ param 
1: Pin Number	(Accepted values [A1-A4,B1-B4,C1-C4, F1-F2, G1-G2] )
2: Time Period of the Pulse
3: Pulse Level [ HIGH or LOW]
Returns Nothing
********************************************************************************/
void PULSE(unsigned char pin,long int time, unsigned char logic);


/********************************************************************************
Read Digital Input

@ param 
1: Pin Number	(Accepted values [A1-A4,B1-B4,C1-C4, F1-F2, G1-G2] )

Returns Port Value [ 1 or 0 ]
********************************************************************************/
unsigned char READ_PIN(unsigned char pin);



/********************************************************************************
Write Digital Value

@ param 
1: Pin Number	(Accepted values [A1-A4,B1-B4,C1-C4, F1-F2, G1-G2] )
2: Value to be written to the port [Accepted values [HIGH, LOW or 1 , 0] ]
	
Returns Nothing
********************************************************************************/

void WRITE_LOGIC(unsigned char pin,unsigned char val);


/******************** ADC operations	*****************************************/

/********************************************************************************
Disable the ADC
	
Returns Nothing
********************************************************************************/
void ADC_DISABLE(void);


/********************************************************************************
Configure ADC Resolution

@ param 
1: Pin Number	(Accepted values [A1-A4,B1-B4,C1-C4, F1-F2, G1-G2] )

	
Returns Nothing
********************************************************************************/
void ADC_RESOLUTION(unsigned char mode);


/********************************************************************************
Configure ADC Resolution

@ param 
1: Pin Number	(Accepted values [A1-A4,B2-B4] )

	
Returns ADC result
********************************************************************************/
unsigned int ADC_READ(unsigned char pin);


/******************** PWM operations	*****************************************/

/********************************************************************************
Enable PWM Channels

@ param 
1: No. of channels	(Accepted values 1 or 2 )
	Channel 1 is A2 and Channel 2 is A3

	
Returns nothing
********************************************************************************/

void ENABLE_PWM(unsigned char No_channel);



/********************************************************************************
Adjust PWM Duty Cycle

@ param 
1: PWM Channel ( Accepted values are A2 & A3 ) 
2: Duty Cycle  ( Accepted values 0% to 100 %)	

	
Returns nothing
********************************************************************************/

void WRITE_PWM (unsigned char pin,unsigned int dutycycle);


/******************* SERVO motor operations ***********************************/

/********************************************************************************
Enable Servo Motor

@ param 
1: No. of channels	(Accepted values 1 or 2 )
	 Channel 1 is A2 and Channel 2 is A3

	
Returns nothing
********************************************************************************/

void ENABLE_SERVO(unsigned char No_channel);


/********************************************************************************
Rotate Servo Motor

@ param 
1: Servo Port ( Accepted values are A2 & A3 ) 
2: Angle of Rotation  ( Accepted values 0 to 180 )	

	
Returns nothing
********************************************************************************/

void WRITE_SERVO(unsigned char pin, unsigned char direction);


/******************* MOTOR operations *****************************************/

/********************************************************************************
Enable MOTOR1

Returns nothing
********************************************************************************/

void ENABLE_MOTOR1(void);


/********************************************************************************
Enable MOTOR2

Returns nothing
********************************************************************************/

void ENABLE_MOTOR2(void);

/********************************************************************************
Rotate Motor1 

@ param 
1: Motor Value ( Accepted values -20  to 20)
	if value is less than 0, motor will rotate reverse direction
	if value is greater than 0, motor will rotate forward direction
	
Returns nothing
********************************************************************************/

void MOTOR_1(signed int );


/********************************************************************************
Rotate Motor2 

@ param 
1: Motor Value ( Accepted values -20  to 20)
	if value is less than 0, motor will rotate reverse direction
	if value is greater than 0, motor will rotate forward direction
	
Returns nothing
********************************************************************************/
void MOTOR_2(signed int );



/******************* UART opearions ********************************************/

/********************************************************************************
Rotate Motor2 

@ param 
1: Motor Value ( Accepted values -20  to 20)
	if value is less than 0, motor will rotate reverse direction
	if value is greater than 0, motor will rotate forward direction
	
Returns nothing
********************************************************************************/
void MOTOR_2(signed int );


/********************************************************************************
Configure Serial Port

@ param 
1: Baud rate (Accepted values UART_BAUDRATE_BAUDRATE_Baud1200,
	 UART_BAUDRATE_BAUDRATE_Baud2400, UART_BAUDRATE_BAUDRATE_Baud4800,
	 UART_BAUDRATE_BAUDRATE_Baud9600, UART_BAUDRATE_BAUDRATE_Baud14400,
	 UART_BAUDRATE_BAUDRATE_Baud19200, UART_BAUDRATE_BAUDRATE_Baud28800,
	 UART_BAUDRATE_BAUDRATE_Baud38400, UART_BAUDRATE_BAUDRATE_Baud57600,
	 UART_BAUDRATE_BAUDRATE_Baud76800, UART_BAUDRATE_BAUDRATE_Baud115200,
	 UART_BAUDRATE_BAUDRATE_Baud230400, UART_BAUDRATE_BAUDRATE_Baud250000,
	 UART_BAUDRATE_BAUDRATE_Baud460800, UART_BAUDRATE_BAUDRATE_Baud921600,
	 UART_BAUDRATE_BAUDRATE_Baud1M )
	
Returns nothing
********************************************************************************/
void SERIAL_CONFIG(uint32_t baud);



/********************************************************************************
Send One Byte to Serial Port

@ param 
1: Data
	
Returns nothing
********************************************************************************/
void SERIAL_PUTC(unsigned char data);


/********************************************************************************
Send String to Serial Port

@ param 
1: String value
	
Returns nothing
********************************************************************************/
void SERIAL_PUT_STRING( const uint8_t * data);


/********************************************************************************
Read Serial Port


	
Returns the data available in the serial port buffer
********************************************************************************/
unsigned char SERIAL_READ(void);


/********************************************************************************
Return true if serial data is available
********************************************************************************/
bool SERIAL_AVAILABLE(void);


/******************* delay opearions *******************************************/
/********************************************************************************
Make Delay
@param
1: Hours
2: Minutes
3: Seconds
4: Milli seconds

returns nothing
********************************************************************************/
void DELAY(unsigned int hour, unsigned int min, unsigned int sec,unsigned int ms);

/********************************************************************************
Make Delay in Microseconds
@param 
1: Micro Seconds

returns nothing

********************************************************************************/

void DELAY_US(unsigned int us);

/******************* ultrasonic operations *************************************/



/******************* TFT operations ********************************************/
void ENABLE_DISPLAY(void);

void CLEAR_DISPLAY(void);

void LOAD_IMAGE(unsigned char number);

void PUTC_1X_DISPLAY(unsigned char col,unsigned char row, unsigned char data,unsigned int color);

void PUTC_2X_DISPLAY(unsigned char col,unsigned char row, unsigned char data,unsigned int color);

void PUTC_3X_DISPLAY(unsigned char col,unsigned char row, unsigned char data,unsigned int color);

void PUTSTRING_1X_DISPLAY(unsigned char col,unsigned char row, const char *data,unsigned int color);

void PUTSTRING_2X_DISPLAY(unsigned char col,unsigned char row, const char *data,unsigned int color);

void PUTSTRING_3X_DISPLAY(unsigned char col,unsigned char row, const char *data,unsigned int color);

void DRAW_LINE(unsigned char col1,unsigned char row1, unsigned char col2, unsigned char row2,unsigned int color);


/******************** color sensor oprations **************************************/

unsigned char READ_COLOR(unsigned char color);


/******************** Bt remote ***************************************************/

unsigned char READ_BT_REMOTE(void);

unsigned int Read_UltraSonic_Sensor(void);


void COLOUR_SENSOR_ENABLE(void);


void setup(void);

void loop(void);

void Clear_Flash(void);

#endif



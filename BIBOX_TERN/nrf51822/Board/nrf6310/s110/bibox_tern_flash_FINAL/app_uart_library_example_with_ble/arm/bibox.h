

#ifndef BIBOX
#define BIBOX

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


void SET_AS_INPUT(unsigned char pin,unsigned char pull);

void SET_AS_OUTPUT(unsigned char pin);

void PULSE(unsigned char pin,long int time, unsigned char logic);

unsigned char READ_PIN(unsigned char pin);

void WRITE_PIN(unsigned char pin,unsigned char val);


/******************** ADC operations	*****************************************/

void ADC_ENABLE(void);

void ADC_DISABLE(void);

void ADC_RESOLUTION(unsigned char mode);

unsigned int ADC_READ(unsigned char pin);


/******************** PWM operations	*****************************************/

void ENABLE_PWM(unsigned char No_channel,unsigned char pin1, unsigned char pin2);

void WRITE_PWM (unsigned char pin,unsigned int dutycycle);


/******************* SERVO motor operations ***********************************/

void ENABLE_SERVO(unsigned char No_channel);

void WRITE_SERVO(unsigned char pin, unsigned char direction);


/******************* MOTOR operations *****************************************/

void MOTOR_1(unsigned char direction,unsigned int speed);

void MOTOR_2(unsigned char direction,unsigned int speed);



/******************* UART opearions ********************************************/

void CONFIG_UART(unsigned int baudrate);

void PUTC_UART(unsigned char data);

void PUTSTRING_UART(const char * data);

unsigned char READ_UART(void);

unsigned char DATA_AVAILABLE_UART(void);


/******************* delay opearions *******************************************/

void DELAY(unsigned int hour, unsigned int min, unsigned int sec,unsigned int ms);

void DELAU_US(unsigned int us);

/******************* ultrasonic operations *************************************/



/******************* TFT operations ********************************************/

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


void Initialize_Display(void);

#endif



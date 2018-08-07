

Bibox API Reference

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
void SERIAL_PUT_STRING(const char * data);







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

void DELAU_US(unsigned int us);










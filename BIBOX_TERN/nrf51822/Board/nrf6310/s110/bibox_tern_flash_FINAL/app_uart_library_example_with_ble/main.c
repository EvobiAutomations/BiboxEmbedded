/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the license.txt file.
 */

/** @file
 * @brief    UART over BLE application using the app_uart library (event driven).
 *
 * This UART example is configured with flow control enabled which is necessary when softdevice
 * is enabled, in order to prevent data loss. To connect the development kit with your PC via 
 * UART, connect the configured RXD, TXD, RTS and CTS pins to the RXD, TXD, RTS and CTS pins 
 * on header P15 on the motherboard. Then connect the RS232 port on the nRFgo motherboard to
 * your PC. Configuration for UART pins is defined in the uart_conf.h header file.
 *
 * This file contains source code for a sample application that uses the Nordic UART service.
 * Connect to the UART example via Master Control Panel and the PCA10000 USB dongle, or via 
 * nRF UART 2.0 app for Android, or nRF UART app for IOS, available on 
 * https://www.nordicsemi.com/Products/nRFready-Demo-APPS.
 *
 * This example should be operated in the same way as the UART example for the evaluation board
 * in the SDK. Follow the same guide for this example, given on:
 * https://devzone.nordicsemi.com/documentation/nrf51/6.0.0/s110/html/a00066.html#project_uart_nus_eval_test
 *
 * This example uses FIFO RX and FIFO TX buffer to operate with the UART. You can set the size
 * for the FIFO buffers by modifying the RX_BUFFER_SIZE and TX_BUFFER_SIZE constants.
 *
 * Documentation for the app_uart library is given in UART driver documentation in the SDK at:
 * https://devzone.nordicsemi.com/documentation/nrf51/6.1.0/s110/html/a00008.html
 */

/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */


#include <stdint.h>
#include <string.h>
#include <math.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf51_bitfields.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "softdevice_handler.h"
#include "app_timer.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "boards.h"
#include "ble_error_log.h"
#include "ble_debug_assert_handler.h"
#include "app_util_platform.h"
#include "nrf_pwm.h"
#include "pstorage_platform.h"
#include "pstorage.h"
#include "nrf_delay.h"
#include <string.h>
#include "ST7735.h"
#include "winbond_flash.h"
#include "bibox.h"
#include "simple_uart.h"
#include "MFRC522.h"
#include "soft_spi.h"
#include "max7219.h"
#include "image.h"
#include "twi.h"
#include "mma8452.h"
#include "mp3module.h"
#include "timeslot.h"
#include "apds9960.h"
#include "hmc5883.h"
#include "lcd.h"
#include "mpu6050.h"
#include "twi_master.h"



#define IS_SRVC_CHANGED_CHARACT_PRESENT 0                                           /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/


#define WAKEUP_BUTTON_PIN               BUTTON_0                                    /**< Button used to wake up the application. */

#define ADVERTISING_LED_PIN_NO          LED_0                                       /**< LED to indicate advertising state. */
#define CONNECTED_LED_PIN_NO            LED_1                                       /**< LED to indicate connected state. */

#define DEVICE_NAME                      "BIBOX TERN"                            /**< Name of device. Will be included in the advertising data. */

#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                       	/**< The advertising timeout (in units of seconds). */

#define APP_TIMER_PRESCALER             0                                           /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_MAX_TIMERS            3                                           /**< Maximum number of simultaneously created timers. */
#define APP_TIMER_OP_QUEUE_SIZE         4                                           /**< Size of timer operation queues. */

#define MIN_CONN_INTERVAL               7.5                                          /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               40                                          /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< slave latency. */
#define CONN_SUP_TIMEOUT                300                                         /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(1000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(3000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define BUTTON_DETECTION_DELAY          APP_TIMER_TICKS(50, APP_TIMER_PRESCALER)    /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define SEC_PARAM_TIMEOUT               30                                          /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */
                              

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define APP_GPIOTE_MAX_USERS            1

static ble_gap_sec_params_t             m_sec_params;                               /**< Security requirements for this application. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */

static bool ble_buffer_available = true;
static bool tx_complete = false;


	
	
	


	/*       look up table for hex to ASCII conversion																		*/

const char To_ASCII[16]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};	



const unsigned int bluetooth_red[]=
{ 
  0x0000, 0x0000, 0x0000, 0xF800, 0xF800, 0x0000, 0x0000,   
  0xF800, 0x0000, 0x0000, 0xF800, 0x0000, 0xF800,	0x0000, 
	0x0000, 0xF800, 0x0000, 0xF800, 0x0000, 0x0000, 0xF800, 
	0x0000, 0x0000, 0xF800, 0xF800, 0x0000, 0xF800, 0x0000, 
	0x0000, 0x0000, 0x0000, 0xF800, 0xF800, 0x0000, 0x0000,	
	0x0000, 0x0000, 0xF800, 0xF800, 0x0000, 0xF800, 0x0000,
	0x0000, 0xF800, 0x0000, 0xF800, 0x0000, 0x0000, 0xF800, 
	0xF800, 0x0000, 0x0000, 0xF800, 0x0000, 0xF800,	0x0000,
	0x0000, 0x0000, 0x0000, 0xF800, 0xF800, 0x0000, 0x0000,  
	
};







	const unsigned int bluetooth_blue[]=
{ 
  0x0000, 0x0000, 0x0000, 0x001F, 0x001F, 0x0000, 0x0000,   
  0x001F, 0x0000, 0x0000, 0x001F, 0x0000, 0x001F,	0x0000, 
	0x0000, 0x001F, 0x0000, 0x001F, 0x0000, 0x0000, 0x001F, 
	0x0000, 0x0000, 0x001F, 0x001F, 0x0000, 0x001F, 0x0000, 
	0x0000, 0x0000, 0x0000, 0x001F, 0x001F, 0x0000, 0x0000,	
	0x0000, 0x0000, 0x001F, 0x001F, 0x0000, 0x001F, 0x0000,
	0x0000, 0x001F, 0x0000, 0x001F, 0x0000, 0x0000, 0x001F, 
	0x001F, 0x0000, 0x0000, 0x001F, 0x0000, 0x001F,	0x0000,
	0x0000, 0x0000, 0x0000, 0x001F, 0x001F, 0x0000, 0x0000,
	
	
	
	
};





	
uint16_t Bi_Counter1=0, Bi_Counter2=0,Bi_Counter3=0,image_counter=0;
unsigned int Bi_Data1,Bi_Data2,Bi_Data3;
uint8_t Bi_Flag1, Bi_Flag2, Bi_Flag3;
	
	
	
static uint8_t code_array[2048],Ble_Remote,Bt_Slider;
uint8_t temp;
int array_index=5; 
uint32_t err_code,Millis=0;



int data_length=0,if_count=0;
bool new_code=false,code_complete=false,data_end=false,ULTRASONIC_Timeout=false;
bool chip_overheat=false;

uint8_t loop_depth=0;
uint8_t loop_pc[20];
uint8_t loop_count[20];


	

nrf_pwm_config_t pwm_config = PWM_DEFAULT_CONFIG;  







/************************************** variables for storing port values ****************************************/

unsigned int A1_val=0,A2_val=0,A3_val=0,A4_val=0,M1_val=0,M2_val=0,B1_val=0,B2_val=0,B3_val=0,B4_val=0;
unsigned int C1_val=0,C2_val=0,C3_val=0,C4_val=0,F1_val=0,F2_val=0,F3_val=0,F4_val=0,G1_val=0,G2_val=0,G3_val=0,G4_val=0;
unsigned int M1_Prev_val=0,M2_Prev_Val=0,Ultra1,Ultra2,Red=0,Green=0,Blue=0,Rfid_val=0,Servo_Direct1=0,Servo_Direct2=0; 

unsigned char color_sensor_index=0,Dot_Matrix1,Dot_Matrix2,Dot_Matrix3,Dot_Matrix4,Dot_Matrix5;
	
/*****************************************************************************************************************/ 
 bool Ultra1_En=false,Ultra2_En=false;
 bool Battery_low=false,Battery_low_prev=false;
 bool System_Error=false,adc_semaphore=false;
 bool Is_Connected=false,Is_COnnected_prev=false, Iot_data=false,Prog_Pin_Prev=false;
 bool Img_receiving=false,Img_block_Complete=false,Iot_Receiving=false;
 bool Corrupt_Program=false;
 bool Image_Enabled=false;
	
char char_array[20];


unsigned int SEC=0,MIN=0,HOUR=0,TICK=0,Distance,R_HOUR=0,R_MIN=0,SEC_Prev,MIN_Prev,HOUR_Prev,R_HOUR_Prev,R_MIN_Prev;


int Prog_Pin_Count=0,Double_click_Count=0;


unsigned char x_loca=2,y_loca=2,Bat_prev_val;

extern const unsigned char FONT[95][5];







/* ================================================== */

unsigned int IOT_1=0,IOT_2=0,IOT_3=0,IOT_4=0,IOT_5=0;


unsigned int IOT_6=0,IOT_7=0,IOT_8=0,IOT_9=0,IOT_10=0;

signed int BAT;
unsigned char NAME[10]={' ',' ',' ',' ',' ',' ',' ',' ',' ',' '};
unsigned char img_location;

bool loop_break=false;
bool Bi_data_flag=false;
signed int Acc_X_Val,Acc_Y_Val,Acc_Z_Val;
unsigned char Song_Number=0;

//////////////////////////////////
////////// APDS Val //////////////
//////////////////////////////////

struct {
	
	
	struct {
		uint16_t Red;
		uint16_t Green;
		uint16_t Blue;
	}Color;
	
	unsigned char Gesture;
	unsigned char Proximity;
	
		
}APDS9960_Val;
//////////////////////////////////
////////// Gyro Val //////////////
//////////////////////////////////

struct {
	
	signed short int GyroX;
	signed short int GyroY;
	signed short int GyroZ;
	
		
}MPU6050_Val;


/* ========================================================================================== */
/* ========================== Application Switching Functions	=============================== */
/* ========================================================================================== */



//__asm void isr_abort(uint32_t reset_handler)
//{
//xPSR_RESET      EQU 0x21000000  ; Default value of xPSR after System Reset.
//EXC_RETURN_CMD  EQU 0xFFFFFFF9  ; EXC_RETURN for ARM Cortex. When loaded to PC the current interrupt service routine (handler mode) willl exit and the stack will be popped. Execution will continue in thread mode.

//    LDR   R4,=MASK_ONES         ; Fill with ones before jumping to reset handling. We be popped as R12 when exiting ISR (Cleaning up the registers).
//    LDR   R5,=MASK_ONES         ; Fill with ones before jumping to reset handling. We be popped as LR when exiting ISR. Ensures no return to application.
//    MOV   R6, R0                ; Move address of reset handler to R6. Will be popped as PC when exiting ISR. Ensures the reset handler will be executed when exist ISR.
//    LDR   R7,=xPSR_RESET        ; Move reset value of xPSR to R7. Will be popped as xPSR when exiting ISR.
//    PUSH  {r4-r7}               ; Push everything to new stack to allow interrupt handler to fetch it on exiting the ISR.
//    
//    LDR   R4,=MASK_ZEROS        ; Fill with zeros before jumping to reset handling. We be popped as R0 when exiting ISR (Cleaning up of the registers).
//    LDR   R5,=MASK_ZEROS        ; Fill with zeros before jumping to reset handling. We be popped as R1 when exiting ISR (Cleaning up of the registers).
//    LDR   R6,=MASK_ZEROS        ; Fill with zeros before jumping to reset handling. We be popped as R2 when exiting ISR (Cleaning up of the registers).
//    LDR   R7,=MASK_ZEROS        ; Fill with zeros before jumping to reset handling. We be popped as R3 when exiting ISR (Cleaning up of the registers).
//    PUSH  {r4-r7}               ; Push zeros (R4-R7) to stack to prepare for exiting the interrupt routine.
//    
//    LDR   R0,=EXC_RETURN_CMD    ; Load the execution return command into register.
//    BX    R0                    ; No return - Handler mode will be exited. Stack will be popped and execution will continue in reset handler initializing other application.
//    ALIGN
//}








//__asm static void bootloader_util_reset(uint32_t start_addr)
//{
//MASK_ONES       EQU 0xFFFFFFFF  ; Ones, to be loaded into register as default value before reset.
//MASK_ZEROS      EQU 0x00000000  ; Zeros, to be loaded into register as default value before reset.

//    LDR   R1, [R0]              ; Get App initial MSP for bootloader.
//    MSR   MSP, R1               ; Set the main stack pointer to the applications MSP.
//    LDR   R0,[R0, #0x04]        ; Load Reset handler into register 0.
//    
//    LDR   R2, =MASK_ZEROS       ; Load zeros to R2
//    MRS   R3, IPSR              ; Load IPSR to R3 to check for handler or thread mode 
//    CMP   R2, R3                ; Compare, if 0 then we are in thread mode and can continue to reset handler of bootloader
//    BNE   isr_abort             ; If not zero we need to exit current ISR and jump to reset handler of bootloader

//    LDR   R4, =MASK_ONES        ; Load ones to R4 to be placed in Link Register.
//    MOV   LR, R4                ; Clear the link register and set to ones to ensure no return.
//    BX    R0                    ; Branch to reset handler of bootloader
//    ALIGN
//}


//__asm void StartApplication(uint32_t start_addr)
//{
//    LDR   R2, [R0]               ; Get App MSP.
//    MSR   MSP, R2                ; Set the main stack pointer to the applications MSP.
//    LDR   R3, [R0, #0x00000004]  ; Get application reset vector address.
//    BX    R3                     ; No return - stack code is now activated only through SVC and plain interrupts.
//    ALIGN
//	
//	
//}





//void bootloader_util_app_start(uint32_t start_addr)
//{
//	
//	 StartApplication(start_addr);
//	
//}




//static void interrupts_disable(void)
//{
//    uint32_t interrupt_setting_mask;
//    uint32_t irq = 0; // We start from first interrupt, i.e. interrupt 0.

//    // Fetch the current interrupt settings.
//    interrupt_setting_mask = NVIC->ISER[0];

//    for (; irq < 32; irq++)
//    {
//        if (interrupt_setting_mask & (0x01  << irq))
//        {
//            // The interrupt was enabled, and hence disable it.
//            NVIC_DisableIRQ((IRQn_Type)irq);
//        }
//    }
//}


// void bootloader_app_start(uint32_t app_addr)
//{
//    uint32_t err_code = sd_softdevice_disable();
//    APP_ERROR_CHECK(err_code);
//	
//    err_code = sd_softdevice_vector_table_base_set(app_addr);

//    APP_ERROR_CHECK(err_code);
//		

//    interrupts_disable();
//    bootloader_util_reset(app_addr);	
//		NVIC_SystemReset();
//}




//void Switch_Mesh(void)
//{
//	
//  bootloader_app_start(0x29000);
//	//NVIC_SystemReset();

//	
//}

















/* ========================================================================================== */
/* ========================== BIBOX functions	=============================================== */
/* ========================================================================================== */


 void __INLINE SET_AS_INPUT(unsigned char pin,unsigned char pull)	
{

	nrf_gpio_cfg_input(pin,pull);
	
}



/**************************************************************************/
/**************************************************************************/

 void __INLINE SET_AS_OUTPUT(unsigned char pin)
{
	
	nrf_gpio_cfg_output(pin);
	
}

/**************************************************************************/
/**************************************************************************/


void PULSE(unsigned char pin,long int time, unsigned char logic)
{
	if(logic)
	{
		nrf_gpio_pin_set(pin);
		nrf_delay_ms(time);
		nrf_gpio_pin_clear(pin);
	}
	else
	{
		nrf_gpio_pin_clear(pin);
		nrf_delay_ms(time);
		nrf_gpio_pin_set(pin);
	}
	
}
/**************************************************************************/
/**************************************************************************/



unsigned char READ_PIN(unsigned char pin)
{	
	return nrf_gpio_pin_read(pin);
}


/**************************************************************************/
/**************************************************************************/



void WRITE_LOGIC(unsigned char pin,unsigned char val)
{
	if(val)
	{
		nrf_gpio_pin_set(pin);
	}
	else
	{
		nrf_gpio_pin_clear(pin);
	}
	
	
}
	
/**************************************************************************/
/**************************************************************************/


	
void ADC_ENABLE(void)
{
		
	NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;	
	NRF_ADC->CONFIG	= (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos) 
															
										| (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos)							
										| (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) ;
									
	
}

/**************************************************************************/
/**************************************************************************/




 void ADC_DISABLE(void)
	 
 {
	 
	 	NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Disabled;	
 }

 
/***************************************************************************/
/***************************************************************************/
 
 
 
 
 void ADC_RESOLUTION(unsigned char mode)
 {
	 
	 	NRF_ADC->CONFIG	|= (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos);				
	 
 }

/***************************************************************************/
/***************************************************************************/
 
 
 
 unsigned int ADC_READ(unsigned char pin)
 {
		NRF_ADC->TASKS_START = 1;
	

		while (NRF_ADC->BUSY==1)
		{};
		NRF_ADC->EVENTS_END	= 0;
	
	
		return( NRF_ADC->RESULT);	
	 
 }
 
 

/***************************************************************************/
/***************************************************************************/





 void ENABLE_PWM(unsigned char No_channel,unsigned char pin1, unsigned char pin2)
 {
	 
		

		if(No_channel)
		{
			pwm_config.num_channels     = 3;		//A2 is port is pwm->> remap pwm to A2 							
			pwm_config.gpio_num[0]      = M1;								
			pwm_config.gpio_num[1]      = M3;
			pwm_config.gpio_num[2]      = pin1;    
		
			nrf_pwm_init(&pwm_config);
			nrf_pwm_set_value(0, 0);
      nrf_pwm_set_value(1, 0);
      nrf_pwm_set_value(2,0);
		}
		else if(No_channel==2)
			
		{
			pwm_config.num_channels     = 4;		//A2 is port is pwm->> remap pwm to A2 							
			pwm_config.gpio_num[0]      = M1;								
			pwm_config.gpio_num[1]      = M3;
			pwm_config.gpio_num[2]      = pin1;    
			pwm_config.gpio_num[3]      = pin2; 
		
			nrf_pwm_init(&pwm_config);
			nrf_pwm_set_value(0, 0);
      nrf_pwm_set_value(1, 0);
      nrf_pwm_set_value(2,0);
		}
		else
		{
			return ;
		}
	 
 }
 
/***************************************************************************/
/***************************************************************************/
 
 void WRITE_PWM(unsigned char pin, unsigned int val)
 {
	 
	 
	 
	 
 }
 
 
/***************************************************************************/
/***************************************************************************/
 
void ENABLE_SERVO(unsigned char No_channel)
{
		

		if(No_channel)
		{
			pwm_config.num_channels     = 3;		//A2 is port is pwm->> remap pwm to A2 							
			pwm_config.gpio_num[0]      = M1;								
			pwm_config.gpio_num[1]      = M3;
			pwm_config.gpio_num[2]      = A1;    
		
			nrf_pwm_init(&pwm_config);
			nrf_pwm_set_value(0, 0);
      nrf_pwm_set_value(1, 0);
      nrf_pwm_set_value(2,0);
		}
		else if(No_channel==2)
			
		{
			pwm_config.num_channels     = 4;		//A2 is port is pwm->> remap pwm to A2 							
			pwm_config.gpio_num[0]      = M1;								
			pwm_config.gpio_num[1]      = M3;
			pwm_config.gpio_num[2]      = A1;    
			pwm_config.gpio_num[3]      = A2; 
		
			nrf_pwm_init(&pwm_config);
			nrf_pwm_set_value(0, 0);
      nrf_pwm_set_value(1, 0);
      nrf_pwm_set_value(2,0);
		}
		else
		{
			return ;
		}
	
}
 
/***************************************************************************/
/***************************************************************************/



void WRITE_SERVO(unsigned char pin, unsigned char val)
{


					if(pin==A2)
					{						
					nrf_pwm_set_value(2, val);
					}
	
					else if(pin==A3)
					{
						
						if(	 pwm_config.num_channels ==3 	)
						{
						
						nrf_pwm_set_value(3, val);
						
						}
						else 	if(	 pwm_config.num_channels ==4 	)
						{
						
						nrf_pwm_set_value(2, val);
						
						}
					}

}	


/***************************************************************************/
/***************************************************************************/
 


void MOTOR_1(unsigned char direction,unsigned int speed)
{
	if(direction== CLOCKWISE)
	{
																							//==========  motor m1 fwd   ========//
				nrf_gpio_pin_clear(M1);								//==========     m1=0        ========//	
				pwm_config.gpio_num[0]=M2;
				nrf_pwm_init(&pwm_config);
				nrf_pwm_set_value(0, speed);	
	}
	
	else
	{
																							//==========  motor m1 rev   ========//
				nrf_gpio_pin_clear(M2);								//==========     m1=pwm      ========//	
				pwm_config.gpio_num[0]=M1 ;
				nrf_pwm_init(&pwm_config);
				nrf_pwm_set_value(0, speed);					//==========     m2=0        ========//	
				
	}
		
	
	
}
 

/***************************************************************************/
/***************************************************************************/



 void MOTOR_2(unsigned char direction,unsigned int speed)
 {
	 if(direction== CLOCKWISE)
	 {
																							//==========  motor m1 fwd   ========//
			
																							//==========  motor m1 fwd   ========//
				nrf_gpio_pin_clear(M3);								//==========     m1=0        ========//	
				pwm_config.gpio_num[1]=M4;
				nrf_pwm_init(&pwm_config);
				nrf_pwm_set_value(1, speed);					//==========   m2 pwm out    ========//	
					
	 }
	
		else
	  {
																							//==========  motor m1 rev   ========//
																							//==========  motor m1 rev   ========//
				nrf_gpio_pin_clear(M4);								//==========     m1=pwm      ========//	
				pwm_config.gpio_num[1]=M3 ;
				nrf_pwm_init(&pwm_config);
				nrf_pwm_set_value(1, speed);					//==========     m2=0        ========//	
			
   	}
	 
	 
 }
 
 
/***************************************************************************/
/***************************************************************************/
 
 
 void DELAY_US(unsigned int val)
 {
	 
	 nrf_delay_us(val);
 }
 void Initialize_Display(void)
	
{
	fill_display(0,0,130,132,ST7735_BLACK);	//clear display 
	
	SetAddrWindow(6, 17, 127, 16);
	
	for(int i=0;i<244;i++)										/* draw line */
	{
	
		ST7735_Write_Data(ST7735_WHITE>>8);
		ST7735_Write_Data(ST7735_WHITE&0xff);
	}	
	
	SetAddrWindow(6, 6, 129, 129);
	ST7735_Put_String1X("Bat:",ST7735_WHITE,ST7735_BLACK);
	SetAddrWindow(36, 120, 129, 129);	
	ST7735_PutChar1X('-',ST7735_WHITE,ST7735_BLACK);
	SetAddrWindow(91, 120, 129, 129);	
	ST7735_PutChar1X('-',ST7735_WHITE,ST7735_BLACK);
	SetAddrWindow(6, 25, 129, 132);	
  ST7735_Put_String1X("A1:",ST7735_WHITE,ST7735_BLACK);
	SetAddrWindow(6, 40, 129, 132);			
	ST7735_Put_String1X("A3:",ST7735_WHITE,ST7735_BLACK);
	SetAddrWindow(6, 55, 129, 132);			
	ST7735_Put_String1X("M1:",ST7735_WHITE,ST7735_BLACK);		
	
	

	SetAddrWindow(43, 120, 129, 129);
	if(HOUR<10)
	{
		ST7735_PutChar1X('0',ST7735_WHITE,ST7735_BLACK);
	}

	sprintf (char_array, "%d",HOUR);
	ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);
	ST7735_PutChar1X(':',ST7735_WHITE,ST7735_BLACK);
	HOUR_Prev=HOUR;
	
	SetAddrWindow(60, 120, 129, 129);
	if(MIN<10)
	{
		ST7735_PutChar1X('0',ST7735_WHITE,ST7735_BLACK);
	}
	sprintf (char_array, "%d",MIN);
	ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);
	ST7735_PutChar1X(':',ST7735_WHITE,ST7735_BLACK);
	MIN_Prev=MIN;
	SetAddrWindow(78, 120, 129, 129);
	if(SEC<10)
	{
		ST7735_PutChar1X('0',ST7735_WHITE,ST7735_BLACK);
	}
	sprintf (char_array, "%d",SEC);
	ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);

	
	

	SetAddrWindow(81, 6, 129, 129);								
	if(R_HOUR<10)
	{
		ST7735_PutChar1X('0',ST7735_WHITE,ST7735_BLACK);
	}
	
	sprintf (char_array, "%d",R_HOUR);
	ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);
	ST7735_PutChar1X(':',ST7735_WHITE,ST7735_BLACK);
	R_HOUR_Prev=R_HOUR;




	SetAddrWindow(98, 6, 129, 129);		
	if(R_MIN<10)
	{
		ST7735_PutChar1X('0',ST7735_WHITE,ST7735_BLACK);
	}
	sprintf (char_array, "%d",R_MIN);
	ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);
	ST7735_PutChar1X(':',ST7735_WHITE,ST7735_BLACK);
	R_MIN_Prev=R_MIN;

	
	SetAddrWindow(116, 6, 129, 129);
	if(SEC<10)
	{
		ST7735_PutChar1X('0',ST7735_WHITE,ST7735_BLACK);
	}
	sprintf (char_array, "%d",SEC);
	ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);
	
	
	
	
}

 
/***************************************************************************/
/***************************************************************************/
 
 
 
 void DELAY(unsigned int hour, unsigned int min, unsigned int sec,unsigned int ms)
 {
	 
	 nrf_delay_ms(hour*3600000);
	 nrf_delay_ms(min*60000);
	 nrf_delay_ms(sec*1000);
	 nrf_delay_ms(ms);
	 
	 
	 
 }


/***************************************************************************/
/***************************************************************************/

 
 unsigned char BT_REMOTE_READ(void)
 {
	 
	 return Ble_Remote;
 }

 
/***************************************************************************/
/***************************************************************************/
 
 void TFT_ENABLE(void)
 {
	 ST7735_SPI_Initial();
	 ST7735_Initial();
	 Winbond_SPI_Initial();
	 
 }
 
 
 void TFT_DISPLAY_PIXEL(unsigned char row , unsigned char col, unsigned int color)
 {
	 
	 SetAddrWindow(col,row, 129, 129);
	 ST7735_Write_Data(color>>8);
	 ST7735_Write_Data(color);
	 
	 
 }


 void TFT_DISPLAY_CHAR(unsigned char row, unsigned char col,unsigned char size, unsigned char val, unsigned char color)
 {
	
	 SetAddrWindow(col,row,129,129);
	 
	 if(size==FONT_1)
	 {
	 ST7735_PutChar1X(val,color,ST7735_BLACK);
	 }
	 else if(size==FONT_2)
	 {
		 ST7735_PutChar2X(val,color,ST7735_BLACK);
	 }
	 
	 else if( size==FONT_3)
	 {
		 ST7735_PutChar3X(val,color,ST7735_BLACK);
	 }
	 
	 
 }
 
 void TFT_DISPLAY_STRING(unsigned char row, unsigned char col, unsigned char size, const  char *val, unsigned int color)
 {
	 
		SetAddrWindow(col,row,129,129);
		 
		 if(size==FONT_1)
		 {
		 ST7735_Put_String1X(val,color,ST7735_BLACK);
		 }
		 else if(size==FONT_2)
		 {
			 ST7735_Put_String2X(val,color,ST7735_BLACK);
		 }
		 
		 else if( size==FONT_3)
		 {
			ST7735_Put_String3X(val,color,ST7735_BLACK);
		 }
	 
 
 }
 
 
 void TFT_DISPLAY_VARIABLE(unsigned char row, unsigned char col, unsigned char size, unsigned int val, unsigned int color)
 {
	 
	 sprintf (char_array, "%d",val);
	 
	 SetAddrWindow(col,row,129,129);
		 
		 if(size==FONT_1)
		 {
		 ST7735_Put_String1X(char_array,color,ST7735_BLACK);
		 }
		 else if(size==FONT_2)
		 {
			 ST7735_Put_String2X(char_array,color,ST7735_BLACK);
		 }
		 
		 else if( size==FONT_3)
		 {
			ST7735_Put_String3X(char_array,color,ST7735_BLACK);
		 }
	 
	 
	  
 }
 
 
 
 void TFT_CLEAR_DISPLAY(void)
 {
	 
	 	clear_display();
 }
 


 void TFT_DISABLE(void)
 {
	 NRF_SPI0->ENABLE=0;
	 nrf_gpio_pin_clear(ST7735_BACKLIGHT);
	 
 }
 
 
 
  
 
 
 void SERIAL_CONFIG(uint32_t baud)
 {
	 
	 simple_uart_config(RTS_PIN_NUMBER,TX_PIN_NUMBER,CTS_PIN_NUMBER,RX_PIN_NUMBER,false,baud);
 }
 
 
 void SERIAL_PUTC(unsigned char data)
 {
	 
	 simple_uart_put(data);
 }
 
 
 
 void SERIAL_PUT_STRING(const uint8_t *ptr)
 {
	 
	 simple_uart_putstring(ptr);
 }
 
 
 
 bool SERIAL_AVILABLE(void)
 {
	 return simple_uart_data_available();
 }
 
 
 unsigned char SERIAL_READ(void)
 {
	  return simple_uart_get();
	 
 }
	 
 
 
 void COLOUR_SENSOR_ENABLE(void)
 {
	 
	
	 SERIAL_CONFIG(UART_BAUDRATE_BAUDRATE_Baud9600);
	 
	 
 }
 
 void COLOUR_SENSOR_DISABLE(void)
 {
	 
	 NRF_UART0->ENABLE=0;
 }
 
 
 
 unsigned int COLOUR_SENSOR_READ(unsigned char color)
 {
			 uint8_t val;
			 
			if(color==RED)
			{
				simple_uart_put('R');
			}
			else if(color==GREEN)
			{
				simple_uart_put('G');
			}
			
			else if(color==BLUE)
			{
				simple_uart_put('B');
			}
			
			if(simple_uart_get_with_timeout(1000,&val))
			{
				return val;
			}
			else
			{
				return 0;
			}
			
	

 }
 
 
 
 
/* ======================================================================================= */
/* ======================= ADC READ ====================================================== */
/* ======================================================================================= */



/** @brief initialize the ADC and return the 10 bit adc value 
		@parm inchannel number [1ul,2ul,4ul,8ul,16ul,32ul,128ul,256ul]

*/

 
 void adc_initial(void)
 {
	 NRF_ADC->INTENSET = (ADC_INTENSET_END_Disabled << ADC_INTENSET_END_Pos);					
					
	
	NRF_ADC->CONFIG	= (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos) 
										| (ADC_CONFIG_PSEL_Disabled << ADC_CONFIG_PSEL_Pos)					
										| (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos)							
										| (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) 
										| (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos);									
	

	NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Disabled;			
 }
 
 
 
 
 
 

	uint16_t  adc_read_sample(unsigned long adc_channel)
{
	

	NRF_ADC->INTENSET = (ADC_INTENSET_END_Disabled << ADC_INTENSET_END_Pos);					
					
	
	NRF_ADC->CONFIG	= (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos) 
										| (adc_channel << ADC_CONFIG_PSEL_Pos)					
										| (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos)							
										| (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) 
										| (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos);									
	
	NRF_LPCOMP->ENABLE=0;

	NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;					  													
	
	NRF_ADC->TASKS_START = 1;
	int i=100;

	while (NRF_ADC->BUSY==1&&i>0)
	{i--;};
	NRF_ADC->EVENTS_END	= 0;
	
	
	return( NRF_ADC->RESULT);	
		
  
}



uint16_t adc_read(unsigned long adc_channel)
{
	
	return adc_read_sample(adc_channel);
	
	
}




	uint16_t  read_bat(void)
{

		
	NRF_ADC->INTENSET = (ADC_INTENSET_END_Disabled << ADC_INTENSET_END_Pos);					
					
	
	NRF_ADC->CONFIG	= (ADC_CONFIG_EXTREFSEL_None << ADC_CONFIG_EXTREFSEL_Pos) 
										| (2ul << ADC_CONFIG_PSEL_Pos)					
										| (ADC_CONFIG_REFSEL_VBG << ADC_CONFIG_REFSEL_Pos)							
										| (ADC_CONFIG_INPSEL_AnalogInputOneThirdPrescaling << ADC_CONFIG_INPSEL_Pos) 
										| (ADC_CONFIG_RES_10bit << ADC_CONFIG_RES_Pos);									
	

	NRF_ADC->ENABLE = ADC_ENABLE_ENABLE_Enabled;					  													
	
	NRF_ADC->TASKS_START = 1;
	

	while (NRF_ADC->BUSY==1)
	{};
	NRF_ADC->EVENTS_END	= 0;
	
	
	return( NRF_ADC->RESULT);	
		
  
}




void Display_BleIcon(void)
{
		SetAddrWindow(67, 5, 73, 14);
		for(int i=0;i<63;i++)
		{
			if(Is_Connected)
			{
			ST7735_Write_Data(bluetooth_blue[i]>>8);
			ST7735_Write_Data(bluetooth_blue[i]);
			
			}
			else
			{				
			ST7735_Write_Data(bluetooth_red[i]>>8);
			ST7735_Write_Data(bluetooth_red[i]);
			}
					
		}
}






/* =========================================================================== */
/* ===================== READ COLOR SENSOR VALUES ============================ */
/* =========================================================================== */



void read_color_sensor(void)
{
	unsigned char  rx_lsb=0,rx_msb=0;

	simple_uart_put('R');
	simple_uart_get_with_timeout(5,&rx_lsb);
	simple_uart_get_with_timeout(5,&rx_msb);
	

	
	Red=rx_msb;
	Red<<=8;
	Red|=rx_lsb;
	
	simple_uart_put('G');
	simple_uart_get_with_timeout(5,&rx_lsb);
	simple_uart_get_with_timeout(5,&rx_msb);
	
	
	
	Green=rx_msb;
	Green<<=8;
	Green|=rx_lsb;
	
	
	simple_uart_put('B');
	simple_uart_get_with_timeout(5,&rx_lsb);
	simple_uart_get_with_timeout(5,&rx_msb);
		
	
	Blue=rx_msb;
	Blue<<=8;
	Blue|=rx_lsb;
	

}



/* =========================================================================== */
/* ===================== Read RFID card ====================================== */
/* =========================================================================== */

unsigned int Read_Rfid(void)
{
	unsigned char status,str[16];
	unsigned int number=0;
	MFRC522_Init();
	MFRC522_Halt();
	
	status = MFRC522_Request(PICC_REQIDL, str);
   // UART1_Write(status);
    if (status == MI_OK)
    {
      //simple_uart_putstring ("Card detected\n");
     // UART1_Write(str[0]);
      //UART1_Write(" , ");
     // UART1_Write(str[1]);
      //UART1_Write(" ");

			
			
			status=MFRC522_Anticoll(str);
			if(status==MI_OK)
      {
        
				
          
					number=str[0];
					number<<=8;
					number|=str[1];
					
				
	
			}else{
				return 0;
			}
		}
		
		MFRC522_Halt();
	
	return number;
  

}



/* =========================================================================== */
/* ========================= Send data to dot matrix disply ================== */
/* =========================================================================== */
void Send_Dotmatrix(void)
{
	unsigned char Dot_Matrix[5];
	
	Dot_Matrix[0]=Dot_Matrix5;
	Dot_Matrix[1]=Dot_Matrix4;
	Dot_Matrix[2]=Dot_Matrix3;
	Dot_Matrix[3]=Dot_Matrix2;
	Dot_Matrix[4]=Dot_Matrix1;
	Max_Display_Char_X(&Dot_Matrix[0],5);	
	
	
}






	/* ========================================================================= */
	/* =================== REFRESH OLED WITH PORT VALUES ======================= */
	/* ========================================================================= */


void Refresh_OLED()
{
		
	unsigned int color=ST7735_GREEN;
	
	
	
	
/* ================== update the actual time  ======================== */		
if(HOUR!=HOUR_Prev)
{
	
	SetAddrWindow(43, 120, 129, 129);
	if(HOUR<10)
	{
		ST7735_PutChar1X('0',ST7735_WHITE,ST7735_BLACK);
	}

	sprintf (char_array, "%d",HOUR);
	ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);
	ST7735_PutChar1X(':',ST7735_WHITE,ST7735_BLACK);
	HOUR_Prev=HOUR;
}	


if(MIN!=MIN_Prev)
{
	SetAddrWindow(60, 120, 129, 129);
	if(MIN<10)
	{
		ST7735_PutChar1X('0',ST7735_WHITE,ST7735_BLACK);
	}
	sprintf (char_array, "%d",MIN);
	ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);
	ST7735_PutChar1X(':',ST7735_WHITE,ST7735_BLACK);
	MIN_Prev=MIN;
}


if(SEC!=SEC_Prev)
{
	SetAddrWindow(78, 120, 129, 129);
	if(SEC<10)
	{
		ST7735_PutChar1X('0',ST7735_WHITE,ST7735_BLACK);
	}
	sprintf (char_array, "%d",SEC);
	ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);

	
	
	
}	

/* =================== update the time =============================== */ 	
	
if(R_HOUR!=R_HOUR_Prev)
{	
	SetAddrWindow(81, 6, 129, 129);								
	if(R_HOUR<10)
	{
		ST7735_PutChar1X('0',ST7735_WHITE,ST7735_BLACK);
	}
	
	sprintf (char_array, "%d",R_HOUR);
	ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);
	ST7735_PutChar1X(':',ST7735_WHITE,ST7735_BLACK);
	R_HOUR_Prev=R_HOUR;
}


if(R_MIN!=R_MIN_Prev)
{
	SetAddrWindow(98, 6, 129, 129);		
	if(R_MIN<10)
	{
		ST7735_PutChar1X('0',ST7735_WHITE,ST7735_BLACK);
	}
	sprintf (char_array, "%d",R_MIN);
	ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);
	ST7735_PutChar1X(':',ST7735_WHITE,ST7735_BLACK);
	R_MIN_Prev=R_MIN;
}

	
if(SEC_Prev!=SEC)
{
	SetAddrWindow(116, 6, 129, 129);
	if(SEC<10)
	{
		ST7735_PutChar1X('0',ST7735_WHITE,ST7735_BLACK);
	}
	sprintf (char_array, "%d",SEC);
	ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);
	
	
	SetAddrWindow(31, 6, 129, 132);
	
	//ULTRA=adc_read(64UL);
	
		BAT=(((adc_read(ADC_CONFIG_PSEL_AnalogInput1)-470)*100)/(785-470));
	
		if(BAT>100)
		{
			BAT=100;
		}
			if(BAT<1)
		{
			BAT=0;
		}	
		
		if(BAT<25)
		{
			color=ST7735_RED;
		}
			
		if(BAT<25)
		{
			Battery_low=true;
					
		}
		else
		{
			Battery_low=false;					
		}
				
	sprintf (char_array, "%d",BAT);
	ST7735_Put_String1X(char_array,color,ST7735_BLACK);
	ST7735_Put_String1X("%  ",color,ST7735_BLACK);	
	MIN_Prev=MIN;		
	
				
				if(Battery_low_prev!=Battery_low)
				{
					SetAddrWindow(20, 110, 119, 132);	
					if(Battery_low)
					{
					
					ST7735_Put_String1X("Recharge Battery",ST7735_RED,ST7735_BLACK);
					}
					else
					{
						
					ST7735_Put_String1X("                ",ST7735_WHITE,ST7735_BLACK);
					}
					Battery_low_prev=Battery_low;
				}
				
				
		
		
		
	
}
	SEC_Prev=SEC;
	
/* ==================================================================== */			
			
	/*		
  ST7735_Put_String1X(" R:",ST7735_WHITE,ST7735_BLACK);

	sprintf (char_array, "%d",ULTRA);
	ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);	
	ST7735_Put_String1X("  ",ST7735_WHITE,ST7735_BLACK);*/
	
//	SetAddrWindow(8, 17, 127, 16);
//	//ST7735_Put_String1X("---------------------",ST7735_GREEN,ST7735_BLACK);	
//	for(int i=0;i<240;i++)
//	{
//	
//		ST7735_Write_Data(ST7735_WHITE>>8);
//		ST7735_Write_Data(ST7735_WHITE&0xff);
//	}	
	
	
	SetAddrWindow(25, 25, 129, 132);	
	sprintf (char_array, "%d",A1_val);
	ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);		
	ST7735_Put_String1X(" A2:",ST7735_WHITE,ST7735_BLACK);
	sprintf (char_array, "%d      ",A2_val);
	ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);			
	//ST7735_Put_String1X("",ST7735_WHITE,ST7735_BLACK);
		
	
	SetAddrWindow(25, 40, 129, 132);	
	sprintf (char_array, "%d",A3_val);
	ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);		
	ST7735_Put_String1X(" A4:",ST7735_WHITE,ST7735_BLACK);
	sprintf (char_array, "%d      ",A4_val);
	ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);	
	//  ST7735_Put_String1X("",ST7735_WHITE,ST7735_BLACK);
	
	SetAddrWindow(25, 55, 129, 132);				
	sprintf (char_array, "%d",M1_val);
	ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);
	ST7735_Put_String1X(" M2:",ST7735_WHITE,ST7735_BLACK);
	sprintf (char_array, "%d    ",M2_val);
	ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);		
	//ST7735_Put_String1X("",ST7735_WHITE,ST7735_BLACK);
	
	
	SetAddrWindow(4, 70, 129, 132);
	
	
	
	if(code_array[34]=='X')
	{
		
		ST7735_Put_String1X("X:",ST7735_WHITE,ST7735_BLACK);	
		sprintf (char_array, "%d",Acc_X_Val);
		ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);
		ST7735_Put_String1X(" Y:",ST7735_WHITE,ST7735_BLACK);
		sprintf (char_array, "%d",Acc_Y_Val);
		ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);
				//ST7735_Put_String1X("",ST7735_WHITE,ST7735_BLACK);			
		//SetAddrWindow(4, y_loca+15, 129, 132);
		ST7735_Put_String1X(" Z:",ST7735_WHITE,ST7735_BLACK);
		sprintf (char_array, "%d",Acc_Z_Val);
		ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);
		while(x_loca<124)
		{
			ST7735_PutChar1X(' ',ST7735_WHITE,ST7735_BLACK);
		}
	
		

		
	}
		else if(code_array[34]=='G')
	{
		
		ST7735_Put_String1X("R:",ST7735_WHITE,ST7735_BLACK);	
		sprintf (char_array, "%d",APDS9960_Val.Color.Red);
		ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);
		ST7735_Put_String1X(" G:",ST7735_WHITE,ST7735_BLACK);
		sprintf (char_array, "%d",APDS9960_Val.Color.Green);
		ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);
				//ST7735_Put_String1X("",ST7735_WHITE,ST7735_BLACK);			
		//SetAddrWindow(4, y_loca+15, 129, 132);
		ST7735_Put_String1X(" B:",ST7735_WHITE,ST7735_BLACK);
		sprintf (char_array, "%d",APDS9960_Val.Color.Blue);
		ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);
		while(x_loca<124)
		{
			ST7735_PutChar1X(' ',ST7735_WHITE,ST7735_BLACK);
		}
		SetAddrWindow(4, 85, 129, 132);
		ST7735_Put_String1X("P:",ST7735_WHITE,ST7735_BLACK);
		sprintf (char_array, "%d ",APDS9960_Val.Proximity);
		ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);
		sprintf (char_array, "Ges:",APDS9960_Val.Proximity);
		ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);
		if(APDS9960_Val.Gesture==DIR_RIGHT)
		{
			ST7735_Put_String1X("R",ST7735_WHITE,ST7735_BLACK);
		}
		else if(APDS9960_Val.Gesture==DIR_LEFT)
		{
			ST7735_Put_String1X("L",ST7735_WHITE,ST7735_BLACK);
		}
		else if(APDS9960_Val.Gesture==DIR_DOWN)
		{
			ST7735_Put_String1X("D",ST7735_WHITE,ST7735_BLACK);
		}
		else if(APDS9960_Val.Gesture==DIR_UP)
		{
			ST7735_Put_String1X("U",ST7735_WHITE,ST7735_BLACK);
		}
	}
	else if(code_array[34]=='T')
	{
		ST7735_Put_String1X("X:",ST7735_WHITE,ST7735_BLACK);	
		sprintf (char_array, "%d",MPU6050_Val.GyroX);
		ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);
		ST7735_Put_String1X(" Y:",ST7735_WHITE,ST7735_BLACK);
		sprintf (char_array, "%d",MPU6050_Val.GyroX);
		ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);
				//ST7735_Put_String1X("",ST7735_WHITE,ST7735_BLACK);			
		//SetAddrWindow(4, y_loca+15, 129, 132);
		ST7735_Put_String1X(" Z:",ST7735_WHITE,ST7735_BLACK);
		sprintf (char_array, "%d",MPU6050_Val.GyroX);
		ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);
		while(x_loca<124)
		{
			ST7735_PutChar1X(' ',ST7735_WHITE,ST7735_BLACK);
		}
		
	}
	if((code_array[15]==65)&&(code_array[16]==65)&&(code_array[17]==65))
		{
					if(code_array[34]=='X')
					{
						SetAddrWindow(4, 85, 129, 132);
					}
				ST7735_Put_String1X("B2:",ST7735_WHITE,ST7735_BLACK);					
				sprintf (char_array, "%d",B2_val);
				ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);
				ST7735_Put_String1X(" B3:",ST7735_WHITE,ST7735_BLACK);
				sprintf (char_array, "%d    ",B3_val);
				ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);
				//ST7735_Put_String1X("",ST7735_WHITE,ST7735_BLACK);			
				SetAddrWindow(4, y_loca+15, 129, 132);
				ST7735_Put_String1X("B4:",ST7735_WHITE,ST7735_BLACK);
				sprintf (char_array, "%d ",B4_val);
				ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);	
				//ST7735_Put_String1X("   ",ST7735_WHITE,ST7735_BLACK);								
			
		}	
		else
		{	
			if(code_array[34]=='X')
			{
				SetAddrWindow(4, 85, 129, 132);
			}	

			
			if(code_array[15]==65)
			{
				ST7735_Put_String1X("B2:",ST7735_WHITE,ST7735_BLACK);					
				sprintf (char_array, "%d",B2_val);
				ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);
			
			}

			 if((code_array[16]==65) && (code_array[30]!='M')&& (code_array[14]!='R') && (code_array[34]!='G')&& (code_array[34]!='T')&& (code_array[34]!='X')&& (code_array[30]!='U')&& (code_array[34]!='U'))
			 {
//				 if(code_array[34]=='G')
//				 {
//				 SetAddrWindow(4, 100, 129, 132);
//				 }
				ST7735_Put_String1X("B3:",ST7735_WHITE,ST7735_BLACK);					
				sprintf (char_array, "%d",B3_val);
				ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);
			  ST7735_Put_String1X("   ",ST7735_WHITE,ST7735_BLACK);
			 }
		   if((code_array[17]==65) && (code_array[30]!='M')&& (code_array[14]!='R') && (code_array[34]!='G')&& (code_array[34]!='T')&& (code_array[34]!='X')&& (code_array[30]!='U')&& (code_array[34]!='U'))
			 {
				ST7735_Put_String1X("B4:",ST7735_WHITE,ST7735_BLACK);
				sprintf (char_array,"%d",B4_val);
				ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);
			
			 }
			
		}
	
	//SetAddrWindow(x_loca, y_loca, 129, 132);
		
		if(code_array[30]=='U')
		{
			if(code_array[34]=='G')
				 {
				 SetAddrWindow(4, 100, 129, 132);
				 }
			
			ST7735_Put_String1X("U1:",ST7735_WHITE,ST7735_BLACK);		
			sprintf (char_array, "%d ",Ultra1);
			ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);
									
		}
		if((code_array[30]=='M') && (code_array[16]!=65) && (code_array[14]!='R') && (code_array[34]!='G')&& (code_array[34]!='T')&& (code_array[34]!='X')&& (code_array[30]!='U')&& (code_array[34]!='U'))
			 {
			ST7735_Put_String1X("MP3:",ST7735_WHITE,ST7735_BLACK);		
			sprintf (char_array, "%d ",Song_Number);
			ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);
															
		}
	
		if(code_array[34]=='U')
		{
			ST7735_Put_String1X("U2:",ST7735_WHITE,ST7735_BLACK);		
			sprintf (char_array, "%d ",Ultra2);
			ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);
									
		}
			
		
		 if((code_array[14]=='R') && (code_array[30]!='M') && (code_array[16]!=65)  && (code_array[34]!='G')&& (code_array[34]!='T')&& (code_array[34]!='X')&& (code_array[30]!='U')&& (code_array[34]!='U'))
			 {
			if(code_array[30]=='U'&&code_array[34]=='U')
			{
				
				ST7735_Put_String1X("   ",ST7735_BLUE,ST7735_BLACK);
				y_loca+=15;
				x_loca=4;
			}
			if(x_loca>120)
			{
				y_loca+=15;
				x_loca=4;
			}
			
			SetAddrWindow(x_loca, y_loca, 129, 132);
			ST7735_Put_String1X("RFID:",ST7735_WHITE,ST7735_BLACK);					
			sprintf (char_array, "%d",Rfid_val);
			ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);
			//ST7735_Put_String1X("    ",ST7735_WHITE,ST7735_BLACK);
		}else
		{
			ST7735_Put_String1X("   ",ST7735_BLUE,ST7735_BLACK);
		}
		
							
	
		if(code_array[30]=='C')
		{		
			if(x_loca!=4)
			{
				y_loca+=15;
				x_loca=4;
			}
			
				SetAddrWindow(x_loca, y_loca, 129, 132);					
				ST7735_Put_String1X("R:",ST7735_RED,ST7735_BLACK);
				sprintf (char_array, "%d ",Red);
				ST7735_Put_String1X(char_array,ST7735_RED,ST7735_BLACK);	
							
				ST7735_Put_String1X("G:",ST7735_GREEN,ST7735_BLACK);
				sprintf (char_array, "%d ",Green);
				ST7735_Put_String1X(char_array,ST7735_GREEN,ST7735_BLACK);	
				
				ST7735_Put_String1X("B:",ST7735_BLUE,ST7735_BLACK);
				sprintf (char_array, "%d",Blue);
				ST7735_Put_String1X(char_array,ST7735_BLUE,ST7735_BLACK);	
																											
		}
		
		
		ST7735_Put_String1X("    ",ST7735_BLUE,ST7735_BLACK);		
			
			/*
		SetAddrWindow(8, 70, 129, 132);
		ST7735_Put_String1X("Ultra1:",ST7735_WHITE,ST7735_BLACK);
		sprintf (char_array,"%d",ULTRA);
		ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);
		ST7735_Put_String1X("   ",ST7735_WHITE,ST7735_BLACK);	
			(8, 85, 129, 132);
		ST7735_Put_String1X("Ultra2:",ST7735_WHITE,ST7735_BLACK);
		sprintf (char_array,"%d",Ultra1);
		ST7735_Put_String1X(char_array,ST7735_WHITE,ST7735_BLACK);
		ST7735_Put_String1X("   ",ST7735_WHITE,ST7735_BLACK);	
		*/
}





/**@brief  Function for placing the application in low power state while waiting for events.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}







//***************************************************************************/
//***************************************************************************/
/****************************************************************************/




//*======================================================================================= *//	
//*========================== FLASH WRITE FUNCTIONS       ================================ *//	
//*======================================================================================= *//	


static pstorage_block_t pstorage_wait_handle = 0;	 
uint32_t retval;	
pstorage_handle_t       base_handle;
pstorage_handle_t block_handle;				
pstorage_module_param_t param;


/** @brief pstorage call back function **/


	static uint8_t pstorage_wait_flag = 0;


	static void bibox_cb_handler(pstorage_handle_t  * handle,
                               uint8_t              op_code,
                               uint32_t             result,
                               uint8_t            * p_data,
                               uint32_t             data_len)
{
	
	if(handle->block_id == pstorage_wait_handle) { pstorage_wait_flag = 0; }
	
	
    switch(op_code)
    {
       
       case PSTORAGE_LOAD_OP_CODE:
           if (result == NRF_SUCCESS)
           {
               // Store operation successful.
						 pstorage_wait_flag = 0;
						 
						 
           }
           else
           {
						
               // Store operation failed.
           }
           // Source memory can now be reused or freed.
           break;
					 
					 case PSTORAGE_UPDATE_OP_CODE:
           if (result == NRF_SUCCESS)
           {
						 
						 pstorage_wait_flag = 0;
               // Update operation successful.
           }
           else
           {
               // Update operation failed.
						 
           }
           break;
					 case PSTORAGE_CLEAR_OP_CODE:
           if (result == NRF_SUCCESS)
           {
						 pstorage_wait_flag = 0;
						 
               // Clear operation successful.
           }
           else
           {
						 
               // Clear operation failed.
           }
           break;
					 
					 case PSTORAGE_STORE_OP_CODE:
           if (result == NRF_SUCCESS)
           {
						
						 pstorage_wait_flag = 0;
               // Clear operation successful.
           }
           else
           {
						
               // Clear operation failed.
           }
           break;
					 
					 
					 
					 
					 
     
    }
}





		
	void bibox_pstorage_write(void)
	{
		
								
		
																				
		retval = pstorage_block_identifier_get(&base_handle, 0, &block_handle);
		
		if (retval == NRF_SUCCESS)
		{
			// Get Block Identifier successful.
			
		}
		else
		{
    // Failed to get block id, take corrective action.
		}	
		
		
		
		pstorage_wait_flag = 1;
		retval = pstorage_clear(&block_handle, 1024);	
		while(pstorage_wait_flag) { power_manage(); }  		//clear flash memmory
	
		
		pstorage_wait_handle = block_handle.block_id;           
		pstorage_wait_flag = 1;   
		pstorage_store(&block_handle,code_array, 1024, 0); //write to flash memory
		while(pstorage_wait_flag) { power_manage(); } 

		/* ======================== writing to second block =================== */ 
		
		retval = pstorage_block_identifier_get(&base_handle, 1, &block_handle);
		
		if (retval == NRF_SUCCESS)
		{
			// Get Block Identifier successful.
			
		}
		else
		{
    // Failed to get block id, take corrective action.
		}	
		
		
		
		pstorage_wait_flag = 1;
		retval = pstorage_clear(&block_handle, 1024);	
		while(pstorage_wait_flag) { power_manage(); }  		//clear falsh memmory
	
		
		pstorage_wait_handle = block_handle.block_id;           
		pstorage_wait_flag = 1;   
		pstorage_store(&block_handle,&code_array[1024], 1024, 0); //write to flash memory
		while(pstorage_wait_flag) { power_manage(); } 
		
		
		
		
			
		
		
	}
		

 /** @breif pstorage event handler function **/
 
		
static void sys_evt_dispatch(uint32_t sys_evt)
   {
        pstorage_sys_event_handler(sys_evt);
				nrf_evt_signal_handler(sys_evt);
    }














/* ======================================================================================== */
/*  app timer call back function for refreshing oled display, calls every 250ms,update time */
/* ======================================================================================== */




static app_timer_id_t         refresh_timer_id;
		
extern unsigned long time;
extern bool TimeSlot;



// ======================================================================================= //
// ======================== READ ULTRA SONIC SENSOR ====================================== //
// ======================================================================================= //


/** @brief returns distance in CM                **/

unsigned int ULTRASONIC_READ1(unsigned int* val)
{				
	if(TimeSlot)
	{
	

		nrf_gpio_pin_clear(TRIGGER);
		nrf_gpio_pin_set(TRIGGER);
		nrf_delay_us(20);
		nrf_gpio_pin_clear(TRIGGER);
		nrf_delay_us(10);
		int  i=10000;
		while((!nrf_gpio_pin_read(ECHO))&&(i>0)){i--;};
		int k=0;
			while(nrf_gpio_pin_read(ECHO)&&k<4706)
			{
				k++;
				nrf_delay_us(5);	
			}
			if(TimeSlot==true)
			{		
				
				
				
			*val=(((k*34*5)/1000)/2)+4;
			TimeSlot=false;	
				nrf_delay_ms(20);
			//configure_next_event_normal();
			//SEGGER_RTT_printf(0, "%d\n ", time, RTT_CTRL_RESET);	
			
			return true;
			}
	
	}
return false;
	
	
	
}



unsigned int ULTRASONIC_READ2(unsigned int* val)
{
	if(TimeSlot)
	{
	

		nrf_gpio_pin_clear(TRIGGER1);
		nrf_gpio_pin_set(TRIGGER1);
		nrf_delay_us(20);
		nrf_gpio_pin_clear(TRIGGER1);
		nrf_delay_us(10);
		int  i=10000;
		while((!nrf_gpio_pin_read(ECHO1))&&(i>0)){i--;};
		unsigned int k=0;
		while(nrf_gpio_pin_read(ECHO1)&&k<4706)
		{
			k++;
			nrf_delay_us(5);	
		}
			if(TimeSlot==true)
			{		
			*val=(((k*34*5)/1000)/2)+4;
			TimeSlot=false;	
			nrf_delay_ms(20);
			return true;
			//configure_next_event_normal();
			//SEGGER_RTT_printf(0, "%d\n ", time, RTT_CTRL_RESET);	
			
			
			}
	
}
return false;
	
	
	
}











/* ======================================================================================== */
/* ======================================================================================== */
/* ======================================================================================== */





















/* ======================================================================================== */
/* ================ update time and refresh oled 			===================================== */
/* ======================================================================================== */	


static void refresh(void * p_context)
{
	
	
	TICK++;
	Millis+=250;
		if(TICK>=4)
		{
			TICK=0;
			SEC++;
			
			
				color_sensor_index++;
				if(color_sensor_index>2)
				{
					color_sensor_index=0;
				}

			
		
/* ======================================================================================= */
/* ==================== Reading the silicon tempearature ================================= */
/* ======================================================================================= */			
			
			
			//	NRF_TEMP->TASKS_START = 1; /** Start the temperature measurement. */

        /* Busy wait while temperature measurement is not finished, you can skip waiting if you enable interrupt for DATARDY event and read the result in the interrupt. */
        /*lint -e{845} // A zero has been given as right argument to operator '|'" */
			/* unsigned int timeout=10000;
       while (NRF_TEMP->EVENTS_DATARDY == 0&&timeout)          
       {		
					timeout--;
					nrf_delay_us(1);
       }
        NRF_TEMP->EVENTS_DATARDY = 0;  
     */   
        /**@note Workaround for PAN_028 rev2.0A anomaly 29 - TEMP: Stop task clears the TEMP register. */       
        //temp = (nrf_temp_read()/4);
        
        /**@note Workaround for PAN_028 rev2.0A anomaly 30 - TEMP: Temp module analog front end does not power down when DATARDY event occurs. */
     //   NRF_TEMP->TASKS_STOP = 1; /** Stop the temperature measurement. */

       // nrf_gpio_port_write(NRF_GPIO_PORT_SELECT_PORT1, (uint8_t)(temp));
      //  nrf_delay_ms(500);
			 
				//simple_uart_put(NRF_TEMP->TEMP/4);
			/*
			 
			 
			 if((NRF_TEMP->TEMP/4)>50)
			 {
				 err_code=app_timer_stop(refresh_timer_id);
				 fill_display(0,0,129,130,ST7735_BLACK);												
				 SetAddrWindow(2, 35, 129, 132);	
				 ST7735_Put_String2X("   SYSTEM OVERHEAT ! ",ST7735_RED,ST7735_BLACK);	
				 chip_overheat=true;
				 
				 return;
			 }
			
			*/
			
			if(nrf_gpio_pin_read(OVER_HEAT))
			{
				 err_code=app_timer_stop(refresh_timer_id);
				 fill_display(0,0,129,130,ST7735_BLACK);												
				 SetAddrWindow(2, 50, 129, 132);	
				 ST7735_Put_String2X("   SYSTEM OVERHEAT ! ",ST7735_RED,ST7735_BLACK);	
				 chip_overheat=true;
				 
				 return;
			}
	

/* ======================================================================================== */
/* ======================================================================================== */
/* ======================================================================================== */			
			
			
				if(SEC>59)
				{
					SEC=0;
					MIN++;
					R_MIN++;
					
						if(MIN>59)
						{
							MIN=0;
							HOUR++;
							
						}
					
						if(R_MIN>59)
						{
							R_MIN=0;
							R_HOUR++;
								if(R_HOUR>23)
								{
									R_HOUR=0;
								}
							
							
						}	
						
						
							
						
						
					
				}
			
			
		}
			
		
		
		
	if(nrf_gpio_pin_read(Prog_Pin)==0)
	{
		Prog_Pin_Count++;
		
		if(Prog_Pin_Prev==1)
		{
			
			Double_click_Count++;
			if(Double_click_Count>1)
			{
				//Switch_Mesh();
				
			}
			
		}
		
			
	}
	else
	{
		Prog_Pin_Count=0;
	}
	
	
	Prog_Pin_Prev=nrf_gpio_pin_read(Prog_Pin);
	
	
	
	
	
	
	/* ============================================================================================== */
	/* ============================	 READ input port values ========================================= */
	/* ============================================================================================== */
			adc_semaphore=true;
	
			if(code_array[8]==73)
				{
				
					A1_val=	nrf_gpio_pin_read(A1);
					
					
				}
				else if (code_array[8]==65)
				{
					A1_val=adc_read(4ul);
					
					
				}
	
				if(code_array[9]==73)
				{
				
					A2_val=	nrf_gpio_pin_read(A2);
									
				}
				else if (code_array[9]==65)
				{
					A2_val=adc_read(8ul);
					
				}
								
				
				if(code_array[10]==73)
				{
				
					A3_val=	nrf_gpio_pin_read(A3);
									
				}		
				else if (code_array[10]==65)
				{
					A3_val=adc_read(16ul);
					
				}

				
	
				if(code_array[11]==73)
				{
				
					A4_val=	nrf_gpio_pin_read(A4);
					
					
				}
				else if (code_array[11]==65)
				{
					A4_val=adc_read(32ul);
					
				}
				
				if(code_array[14]==73)
				{
				
					B1_val=	nrf_gpio_pin_read(B1);
					
					
				}			
				
				
	
				if(code_array[15]==73)
				{
				
					B2_val=	nrf_gpio_pin_read(B2);
					
					
				}		
			 else if (code_array[15]==65)
				{
					B2_val=adc_read(64ul);
					
				}

				if(code_array[16]==73)
				{
				
					B3_val=	nrf_gpio_pin_read(B3);
					
					
				}
				else if (code_array[16]==65)
				{
					B3_val=adc_read(128ul);
					
				}
				
				if(code_array[17]==73)
				{
				
					B4_val=	nrf_gpio_pin_read(B4);
					
					
				}	
				else if (code_array[17]==65)
				{
					B4_val=adc_read(1ul);
					
				}


				
				if(code_array[18]==73)
				{
				
					C1_val=	nrf_gpio_pin_read(C1);
					
					
				}
				if(code_array[19]==73)
				{
				
					C2_val=	nrf_gpio_pin_read(C2);
					
					
				}
				if(code_array[20]==73)
				{
				
					C3_val=	nrf_gpio_pin_read(C3);
					
					
				}
				if(code_array[21]==73)
				{
				
					C4_val=	nrf_gpio_pin_read(C4);
					
					
				}
				/*
				if(code_array[30]=='U')
				{
					Ultra1_En=true;
					Ultra1=ULTRASONIC_READ1();
					
				}	
				if(code_array[34]=='U')
				{
					Ultra2_En=true;
					Ultra2=ULTRASONIC_READ2();
					
				}	
			*/
				if(Is_Connected!=Is_COnnected_prev)
				{
					Display_BleIcon();
					Is_COnnected_prev=Is_Connected;
				}
		
		
/* ===================================================================================== */
/* ===================================================================================== */			
/* ===================================================================================== */				
				
				
	if(Image_Enabled==false)
	{
	
	Refresh_OLED();
	}
	
}









/**@brief     Error handler function, which is called when an error has occurred.
 *
 * @warning   This handler is an example only and does not fit a final product. You need to analyze
 *            how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name. 
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    // This call can be used for debug purposes during application development.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    //ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover with a reset.
   // NVIC_SystemReset();
}


/**@brief       Assert macro callback function.
 *
 * @details     This function will be called in case of an assert in the SoftDevice.
 *
 * @warning     This handler is an example only and does not fit a final product. You need to
 *              analyze how your product is supposed to react in case of Assert.
 * @warning     On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief   Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by this application.
 */
/*
static void leds_init(void)
{
    nrf_gpio_cfg_output(ADVERTISING_LED_PIN_NO);
    nrf_gpio_cfg_output(CONNECTED_LED_PIN_NO);
}


*/
/**@brief   Function for Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void timers_init(void)
{
    // Initialize timer module
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_MAX_TIMERS, APP_TIMER_OP_QUEUE_SIZE, false);
}


/**@brief   Function for the GAP initialization.
 *
 * @details This function will setup all the necessary GAP (Generic Access Profile)
 *          parameters of the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief   Function for the Advertising functionality initialization.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t scanrsp;
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    
    ble_uuid_t adv_uuids[] = {{BLE_UUID_NUS_SERVICE, m_nus.uuid_type}};

    memset(&advdata, 0, sizeof(advdata));
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = false;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;

    memset(&scanrsp, 0, sizeof(scanrsp));
    scanrsp.uuids_complete.uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]);
    scanrsp.uuids_complete.p_uuids  = adv_uuids;
    
    err_code = ble_advdata_set(&advdata, &scanrsp);
    APP_ERROR_CHECK(err_code);
}






bool ble_attempt_to_send(uint8_t * data, uint8_t length)
{
    uint32_t err_code;
    
    err_code = ble_nus_send_string(&m_nus, data,length);
    
    if(err_code == BLE_ERROR_NO_TX_BUFFERS)
    {
        /* ble tx buffer full*/
        return false;
    }                   
    else if (err_code != NRF_ERROR_INVALID_STATE)
	{
        APP_ERROR_CHECK(err_code);   
    }
    
    return true;
    
    
}




// *====================================================================================== *//
// *========================Send the acknowledgement ===================================== *//
// *====================================================================================== *//


void Send_Ack(void)
{
		uint8_t buf[20];
		buf[0]='K';			
		ble_attempt_to_send(&buf[0],1);
}









// ================================================================================================ //
// ============================ HANDLE DATA FROM BLE ============================================== //
// ================================================================================================ //


/**@brief    Function for handling the data from the Nordic UART Service.
 *
 * @details  This function will process the data received from the Nordic UART BLE Service and send
 *           it to the UART module.
 */
/**@snippet [Handling the data received over BLE] */
void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
  
	
//	
//			 if((p_data[0]=='N')&&(p_data[1]=='A')&&(p_data[2]=='M'))			
//			{				
//				
//				for(char i=3;i<length;i++)
//				{			
//						NAME[i-3]=p_data[i];	
//																				
//				}
//				uint32_t m=0x010000;												
//				Winbond_Write_Enable();												
//				Winbond_Chip_32kErase(m);
//				while(Winbond_Flash_Busy()==true);
//				Winbond_Write_Enable();
//				Winbond_Write_8bit(m,10,&NAME[0]);
//				while(Winbond_Flash_Busy()==true);		
//				
//				
//				Img_receiving=false;
//			
//			}
	
//				for(char i=0;i<length;i++)
//				{
//				SEGGER_RTT_printf(0, "%d ", p_data[i], RTT_CTRL_RESET);
//					
//				}
	
	
			if(Img_receiving)
			{
				//Count+=length;
				for(int i=0;i<length;i++)
				{
					code_array[data_length]=p_data[i];
					//SEGGER_RTT_printf(0, "%d ", p_data[i], RTT_CTRL_RESET);
					data_length++;
					if(data_length>255)
					{
						data_length=0;
						Img_block_Complete=true;
					}
					//ST7735_Write_Data(p_data[i]);
					
						if((p_data[i]=='N')&&(p_data[i+1]=='A')&&(p_data[i+2]=='M'))
						{
							//SEGGER_RTT_printf(0, "%s ", "ok", RTT_CTRL_RESET);
							Img_receiving=false;
							int j;
							for( j=3;j<length;j++)
							{
								NAME[j-3]=p_data[j];
								//SEGGER_RTT_printf(0, "%c ", NAME[j-3], RTT_CTRL_RESET);
							}
							
							for(int k=j-3;k<10;k++)
							{
								NAME[k]=' ';
								
							}
							
							
						
						}
				
				}
				
				
				
				
			}
			else if(new_code)
			{
				for(int i=0;i<length;i++)
				{			
				code_array[data_length]=p_data[i];
				data_length++;	
				//SEGGER_RTT_printf(0, "%d ", p_data[i], RTT_CTRL_RESET)	;
					
				}
				
			}
					
			
			
			else if((p_data[0]=='I')&&(p_data[1]=='M')&&(p_data[2]=='G'))
			{
				image_counter++;
				Img_receiving=true;
				data_length=0;
				//img_location=p_data[3];
				for(int i=3;i<length;i++)
				{
				//ST7735_Write_Data(p_data[i]);
				code_array[data_length]=p_data[i];
					//SEGGER_RTT_printf(0, "%d ", p_data[i], RTT_CTRL_RESET);
					//img_location=code_array[3];
				data_length++;	
				//Count+=length-3;	
				}
			}
			else if((p_data[0]=='R')&&(p_data[1]=='T')&&(p_data[2]=='5'))			
			{
				new_code=true;
				data_length=0;
				err_code=app_timer_stop(refresh_timer_id);
				for(int i=0;i<length;i++)
				{			
				code_array[data_length]=p_data[i];
				data_length++;	
				//SEGGER_RTT_printf(0, "%d ", p_data[i], RTT_CTRL_RESET);	
				}
				
				

			}
			else if((p_data[0]=='D')&&(p_data[1]=='F')&&(p_data[2]=='U'))
			{
				sd_power_gpregret_set(POWER_GPREGRET_GPREGRET_Msk);
				sd_nvic_SystemReset();
				
			}
			else if(p_data[0]=='B')
			{
				
			
				Ble_Remote=p_data[1];
				
		
				
			}
			
			else if(p_data[0]=='S')
			{
				
			
				Bt_Slider=p_data[1];
				
				
				
			}	

		else if((p_data[0]=='P')&&(p_data[1]=='B'))
		{
			
		
			IOT_1=p_data[2];
			IOT_1<<=8;
			IOT_1|=p_data[3];
			
			IOT_2=p_data[4];
			IOT_2<<=8;
			IOT_2|=p_data[5];
			
			IOT_3=p_data[6];
			IOT_3<<=8;
			IOT_3|=p_data[7];
			
			IOT_4=p_data[8];
			IOT_4<<=8;
			IOT_4|=p_data[9];
			
			IOT_5=p_data[10];
			IOT_5<<=8;
			IOT_5|=p_data[11];
			
			IOT_6=p_data[12];
			IOT_6<<=8;
			IOT_6|=p_data[13];
			
			IOT_7=p_data[14];
			IOT_7<<=8;
			IOT_7|=p_data[15];
			
			IOT_8=p_data[16];
			IOT_8<<=8;
			IOT_8|=p_data[17];
			
			IOT_9=p_data[18];
			IOT_9<<=8;
			IOT_9|=p_data[19];
			
//				for(int k=0;k<length;k++)
//				{
//				SEGGER_RTT_printf(0, "%d ", p_data[k], RTT_CTRL_RESET);
//				}
			
			
			
//			unsigned int temp_val;	
//			
//			A1_val=p_data[2];
//			A1_val<<=8;
//			A1_val|=p_data[3];
//			A2_val=p_data[4];
//			A2_val<<=8;
//			A2_val|=p_data[5];
//			A3_val=p_data[6];
//			A3_val<<=8;
//			A3_val|=p_data[7];
//			A4_val=p_data[8];
//			A4_val<<=8;
//			A4_val|=p_data[9];
//			M1_temp=p_data[10];
//			M1_temp<<=8;
//			M1_temp|=p_data[11];
//			M2_temp=p_data[12];
//			M2_temp<<=8;
//			M2_temp|=p_data[13];
//			
//			if(code_array[8]==79)								// the port is not pwm
//				{
//					if(A1_val!=0)
//					{					
//						nrf_gpio_pin_set(A1);	
//					
//					}
//					else
//					{
//						nrf_gpio_pin_clear(A1);	
//						
//					}
//					
//				}
//			
//			
//			
//			if(code_array[9]==79)								// the port is not pwm
//				{
//					if(A2_val!=0)
//					{					
//						nrf_gpio_pin_set(A2);	
//					
//					}
//					else
//					{
//						nrf_gpio_pin_clear(A2);	
//						
//					}
//					
//				}
//				
//				else if (code_array[9]==80)		// port is pwm 
//				{
//										
//								/*  remapping servo value(1800-5400) to pwm pulse width for						
//									servo (9 to 40). For changing pulsewidth from 1ms to 2ms 	*/		
//								/*	
//								value=(((value-1800)*31)/3600)+9;				
//								nrf_pwm_set_value(2, value);
//								nrf_delay_ms(200);
//								
//								*/
//						
//					
//						temp_val=(((A2_val)*312)/65536);							
//						nrf_pwm_set_value(2, temp_val);
//						
//						
//					}
//			
//				
//			else if(code_array[9]=='S')
//			{
//				
//				
//				temp_val=(((A2_val)*1000)/65536);	// remapping value for the pwm channel
//				A2_val=((A2_val-1800)*180)/3600;		//remapping value to 0-180 for the display			
//				nrf_pwm_set_value(2, temp_val);
//				
//				
//				
//				
//			}
//				
//			if(code_array[10]==79)								// the port is not pwm
//			{
//					if(A3_val!=0)
//					{					
//						nrf_gpio_pin_set(A3);	
//					
//					}
//					else
//					{
//						nrf_gpio_pin_clear(A3);	
//						
//					}
//					
//			}
//				
//				else if (code_array[10]==80)			// port is pwm 
//				{
//					
//	/* servo value is remapped from 1800-5400 to  to 180 for the display */				
//					
//					
//							
//					
//		
//					
//		/*  remapping servo value(1800-5400) to pwm pulse width for						
//			servo (9 to 40). For changing pulsewidth from 1ms to 2ms 	*/		
//				/*	
//					value=(((value-1800)*31)/3600)+9;				
//					nrf_pwm_set_value(2, value);
//					nrf_delay_ms(200);
//					
//					*/
//									
//							
//							temp_val=(((A3_val)*312)/65536);				
//							if(((code_array[9]=='P')||(code_array[9]=='S'))&&((code_array[10]=='P')||(code_array[10]=='S')))
//							{
//								
//								nrf_pwm_set_value(3, temp_val);
//								
//							}
//							else 
//							{
//									
//								nrf_pwm_set_value(2, temp_val);
//								
//							}
//					
//			}
//				
//			
//			
//			else if(code_array[10]=='S')
//			{
//				temp_val=(((A3_val)*1000)/65536);										//remapping value to 0-180 forthe display
//				A3_val=((A3_val-1800)*180)/3600;											//remapping value for the pwm channel
//					
//				
//				if(((code_array[9]=='P')||(code_array[9]=='S'))&&((code_array[10]=='P')||(code_array[10]=='S')))	//if A2&A3 is pwm, then A3 pwm channel is 3,else 2
//					{
//								
//						nrf_pwm_set_value(3, temp_val);
//								
//					}
//					else 
//					{
//							
//						nrf_pwm_set_value(2, temp_val);
//				
//					}
//				
//				
//				
//			}
//					
//		
//			if(A4_val!=0)
//				{					
//						
//					nrf_gpio_pin_set(A4);	
//				}
//				else
//				{
//							
//				nrf_gpio_pin_clear(A4);
//				}
//			
//			
//	
//		if (M1_temp==20)
//		{		
//			
//			  nrf_gpio_pin_clear(M1);								//===================================//
//				nrf_gpio_pin_clear(M2);								//======stops motor 1================//
//				nrf_pwm_set_value(0,0);
//				M1_val=0;
//			
//		}
//		
//		else if(M1_temp>20)
//		{		
//			
//			
//				M1_val=M1_temp-20;
//				temp_val=(M1_temp-20)*15.6;						//==========  motor m1 fwd   ========//
//				nrf_gpio_pin_clear(M2);								//==========     m1=0        ========//					
//				nrf_pwm_set_value(0, temp_val);				//==========   m2 pwm out    ========//	
//																							// for moving forward M2 os 0 =======//
//		}
//		else if(M1_temp<20)
//		{
//				M1_val=M1_temp-20;
//				temp_val=(20-M1_temp)*15.6;							//==========  motor m1 rev   ========//
//				nrf_gpio_pin_set(M2);									//==========     m1=pwm      ========//					
//				nrf_pwm_set_value(0,( 312-temp_val));	//==========     m2=0        ========//	
//																							//== for moving backward M2 is 1 &
//																							// pwm range is reversed             //
//		}
//		
//	
////===============================================================================================================//		

//		
//		
////===============================================================================================================/
////===============   motor m2 control            =================================================================/
////===============================================================================================================/		
//	
//		
//				if (M2_temp==20)
//				{		
//						M2_val=0;
//						nrf_gpio_pin_clear(M3);														//===================================//
//						nrf_gpio_pin_clear(M4);														//======stops motor 1================//
//						nrf_pwm_set_value(1,0);																									//===================================//
//						
//				}
//				
//				else if(M2_temp>20)
//				{
//						M2_val=M2_temp-20;
//						temp_val=(M2_temp-20)*15.6;													//==========  motor m1 fwd   ========//
//						nrf_gpio_pin_clear(M4);														//==========     m1=0        ========//	
//						nrf_pwm_set_value(1, temp_val);										//==========   m2 pwm out    ========//	
//								
//				}
//				else if(M2_temp<20)
//				{
//						M2_val=M2_temp-20;
//						temp_val=(20-M2_temp)*15.6;													//==========  motor m1 rev   ========//
//						nrf_gpio_pin_set(M4);															//==========     m1=pwm      ========//	
//						nrf_pwm_set_value(1,(312- temp_val));							//==========     m2=0        ========//	
//							
//				}
				
					
		
		
			}
			else if((p_data[0]=='P')&&(p_data[1]=='T'))
			{		
			
				IOT_10=p_data[2];
				IOT_10<<=8;
				IOT_10|=p_data[3];
//				for(int k=0;k<length;k++)
//				{
//				SEGGER_RTT_printf(0, "%d ", p_data[k], RTT_CTRL_RESET);
//				}
			
				
		 }				
		
			else if((p_data[0]=='P')&&(p_data[1]=='A'))
			{
				
				uint8_t buf[20];
				
				buf[0]=IOT_1;
				buf[1]=IOT_1>>8;
				buf[2]=IOT_2;
				buf[3]=IOT_2>>8;		
				buf[4]=IOT_3;
				buf[5]=IOT_3>>8;
				buf[6]=IOT_4;
				buf[7]=IOT_4>>8;	
				buf[8]=IOT_5;
				buf[9]=IOT_5>>8;	

				buf[10]=IOT_6;
				buf[11]=IOT_6>>8;
				buf[12]=IOT_7;
				buf[13]=IOT_7>>8;		
				buf[14]=IOT_8;
				buf[15]=IOT_8>>8;
				buf[16]=IOT_9;
				buf[17]=IOT_9>>8;	
				buf[18]=IOT_10;
				buf[19]=IOT_10>>8;
				
				ble_attempt_to_send(&buf[0],20);
//				for(int k=0;k<length;k++)
//				{
//				SEGGER_RTT_printf(0, "%d ", p_data[k], RTT_CTRL_RESET);
//				}
//				
								
				
			}
			else if(p_data[0]=='P'&&p_data[1]=='R')
			{
				uint8_t buf[20];												
					buf[0]=A1_val;
					buf[1]=A1_val>>8;
					buf[2]=A2_val;
					buf[3]=A2_val>>8;
					buf[4]=A3_val;
					buf[5]=A3_val>>8;
					buf[6]=A4_val;
					buf[7]=A4_val>>8;
					buf[8]=Bi_Data1;
					buf[9]=Bi_Data1>>8;
					buf[10]=Bi_Data2;
					buf[11]=Bi_Data2>>8;
					buf[12]=Bi_Data3;
					buf[13]=Bi_Data3>>8;
					buf[14]=0;
					buf[15]=0;
					buf[16]=0;
					buf[17]=0;
					buf[18]=0;
					buf[19]=0;					
					ble_attempt_to_send(&buf[0],14);	// send back the port value
				
				
			}
			else if(p_data[0]=='P')
			{	 
				uint8_t buf[20];					
							
					buf[0]=A1_val;
					buf[1]=A1_val>>8;
					buf[2]=A2_val;
					buf[3]=A2_val>>8;
					buf[4]=A3_val;
					buf[5]=A3_val>>8;
					buf[6]=A4_val;
					buf[7]=A4_val>>8;
					buf[8]=B1_val;
					buf[9]=B1_val>>8;
					buf[10]=B2_val;
					buf[11]=B2_val>>8;
					buf[12]=B3_val;
					buf[13]=B3_val>>8;
					buf[14]=B4_val;
					buf[15]=B4_val>>8;
					buf[16]=0;
					buf[17]=0;
					buf[18]=0;
					buf[19]=0;
					
					ble_attempt_to_send(&buf[0],20);	// send back the port value
					
			}
			
			
			
				else if(p_data[0]=='D')
			{
		 
				uint8_t buf[2];		
				data_length=0;
					
				if(p_data[1]=='A')
				{
					switch(p_data[2])
					{
						case 1: 
							buf[0]=A1_val;
							buf[1]=A1_val>>8;
						break;
						case 2:
							buf[0]=A2_val;
							buf[1]=A2_val>>8;
						break;
						case 3:
							buf[0]=A3_val;
							buf[1]=A3_val>>8;						
						break;
						case 4:
							buf[0]=A4_val;
							buf[1]=A4_val>>8;
						break;	
						
					}
					
					
				} else if(p_data[1]=='B')
				{
					switch(p_data[2])
					{
						case 1: 
							buf[0]=B1_val;
							buf[1]=B1_val>>8;
						break;
						case 2:
							buf[0]=B2_val;
							buf[1]=B2_val>>8;
						break;
						case 3:
							buf[0]=B3_val;
							buf[1]=B3_val>>8;						
						break;
						case 4:
							buf[0]=B4_val;
							buf[1]=B4_val>>8;
						break;	
						
					}
				}
																
					ble_attempt_to_send(&buf[0],2);	// send back the port value
					
			}
			else if(p_data[0]=='C')
			{
				uint8_t buf[2];		
				data_length=0;
					
				if(p_data[1]=='D')
				{
					switch(p_data[2])
					{
						case 1: 
							buf[0]=Bi_Data1;
							buf[1]=Bi_Data1>>8;
						break;
						case 2:
							buf[0]=Bi_Data2;
							buf[1]=Bi_Data2>>8;
						break;
						case 3:
							buf[0]=Bi_Data3;
							buf[1]=Bi_Data3>>8;						
						break;					
						
					}
					
					
				} 
				
					ble_attempt_to_send(&buf[0],2);	// send back the port value
			}
	
			
	
	
	
	
}
	
	
	
	
	
	
	
	
	
//    for (uint32_t i = 0; i < length; i++)
//    {
//			
//			SEGGER_RTT_printf(0, "%d ", p_data[i], RTT_CTRL_RESET);
//			ST7735_Write_Data(p_data[i]);
//			//while(app_uart_put(p_data[i]) != NRF_SUCCESS);
//			if(Img_receiving)
//			{
//				//code_array[data_length]=p_data[i];
//				
//				if(data_length>255)
//				{
//					Img_block_Complete=true;
//				}
//				
//				
//				
//			}		
			
			
			
			
//			else if(new_code==false)
//				{
//					Ble_data[data_length]=p_data[i];
//						if(data_length>49)
//						{
//							System_Error=true;
//							break;
//						}
//					
//				}
//				else
//				{
//					code_array[data_length]=p_data[i];
//				}
//				
//													
//		
//			
//			if((Ble_data[data_length]=='T')&&(Ble_data[data_length-1]=='E')&&(Ble_data[data_length-2]=='S'))
//			{
//				
//				new_code=true;
//				code_array[0]='R';
//				code_array[1]='T';
//				code_array[2]='5';
//				code_array[3]=1;
//				code_array[4]=1;
//				code_array[5]='S';
//				code_array[6]='E';
//				code_array[7]='T';
//				
//				
//					err_code=app_timer_stop(refresh_timer_id);	
//					data_length=7;			
//					
//			}
//																										
//			
//				if((code_array[data_length]==84&&code_array[data_length-1]==83&&code_array[data_length-2]==82)||(code_array[data_length]==68&&code_array[data_length-1]==78&&code_array[data_length-2]==69))
//				{
//					data_end=true;
//													
//					
//					uint8_t	buf[20];
//					buf[0]='K';				
//					ble_attempt_to_send(&buf[0],1);	// send back the ack
//					
////					
////					//break;	
//				}
//				
//				
//				
//				
//			
//			data_length++;
			  
		
//	 }
		
	 
//	 /* ===================== Ble remote data handling ============================= */
//	 
//	 
//	 
//	 
//	 
//			if((Ble_data[0]=='I')&&(Ble_data[1]=='M')&&(Ble_data[2]=='G'))
//			{
//				err_code=app_timer_stop(refresh_timer_id);		
//				Img_receiving=true;
//							
//	 
//			}
//	 
//		  else if(Ble_data[0]=='B')
//			{
//				
//			
//				Ble_Remote=Ble_data[1];
//				data_length=0;
//		
//				
//			}
//			
//			else if(Ble_data[0]=='S')
//			{
//				
//			
//				Bt_Slider=Ble_data[1];
//				data_length=0;
//				
//				
//			}		
//			
//			
//	/*================ Biprofiler data handling ==================================== */	
//			
//			
//			else if(Ble_data[0]=='P')
//			{
//		 
//				uint8_t buf[20];		
//				data_length=0;
//							
//					buf[0]=A1_val;
//					buf[1]=A1_val>>8;
//					buf[2]=A2_val;
//					buf[3]=A2_val>>8;
//					buf[4]=A3_val;
//					buf[5]=A3_val>>8;
//					buf[6]=A4_val;
//					buf[7]=A4_val>>8;
//					buf[8]=B1_val;
//					buf[9]=B1_val>>8;
//					buf[10]=B2_val;
//					buf[11]=B2_val>>8;
//					buf[12]=B3_val;
//					buf[13]=B3_val>>8;
//					buf[14]=B4_val;
//					buf[15]=B4_val>>8;
//					buf[16]=0;
//					buf[17]=0;
//					buf[18]=0;
//					buf[19]=0;
//					
//					ble_attempt_to_send(&buf[0],20);	// send back the port value
//					
//			}
//			else if(Ble_data[0]=='D')							// display port data handling
//			{
//		 
//				uint8_t buf[2];		
//				data_length=0;
//					
//				if(Ble_data[1]=='A')
//				{
//					switch(Ble_data[2])
//					{
//						case 1: 
//							buf[0]=A1_val;
//							buf[1]=A1_val>>8;
//						break;
//						case 2:
//							buf[0]=A2_val;
//							buf[1]=A2_val>>8;
//						break;
//						case 3:
//							buf[0]=A3_val;
//							buf[1]=A3_val>>8;						
//						break;
//						case 4:
//							buf[0]=A4_val;
//							buf[1]=A4_val>>8;
//						break;	
//						
//					}
//					
//					
//				} else if(Ble_data[1]=='B')
//				{
//					switch(Ble_data[2])
//					{
//						case 1: 
//							buf[0]=B1_val;
//							buf[1]=B1_val>>8;
//						break;
//						case 2:
//							buf[0]=B2_val;
//							buf[1]=B2_val>>8;
//						break;
//						case 3:
//							buf[0]=B3_val;
//							buf[1]=B3_val>>8;						
//						break;
//						case 4:
//							buf[0]=B4_val;
//							buf[1]=B4_val>>8;
//						break;	
//						
//					}
//				}
//																
//					ble_attempt_to_send(&buf[0],2);	// send back the port value
//					
//			}
//			else if(Ble_data[0]=='C')
//			{
//				uint8_t buf[2];		
//				data_length=0;
//					
//				if(Ble_data[1]=='D')
//				{
//					switch(Ble_data[2])
//					{
//						case 1: 
//							buf[0]=Bi_Data1;
//							buf[1]=Bi_Data1>>8;
//						break;
//						case 2:
//							buf[0]=Bi_Data2;
//							buf[1]=Bi_Data2>>8;
//						break;
//						case 3:
//							buf[0]=Bi_Data3;
//							buf[1]=Bi_Data3>>8;						
//						break;					
//						
//					}
//					
//					
//				} 
//				
//					ble_attempt_to_send(&buf[0],2);	// send back the port value
//			}
//			









			
//			else if((Ble_data[0]=='I')&&(Ble_data[1]=='O')&&(Ble_data[2]=='T')&&(Ble_data[3]=='I'))
//			{
//				
//				uint8_t buf[20];		
//				data_length=0;
//							
//					buf[0]=A1_val;
//					buf[1]=A1_val>>8;
//					buf[2]=A2_val;
//					buf[3]=A2_val>>8;
//					buf[4]=A3_val;
//					buf[5]=A3_val>>8;
//					buf[6]=A4_val;
//					buf[7]=A4_val>>8;
//					buf[8]=Bi_Data1;
//					buf[9]=Bi_Data1>>8;
//					buf[10]=Bi_Data2;
//					buf[11]=Bi_Data2>>8;
//					buf[12]=Bi_Data3;
//					buf[13]=Bi_Data3>>8;
//					buf[14]=0;
//					buf[15]=0;
//					buf[16]=0;
//					buf[17]=0;
//					buf[18]=0;
//					buf[19]=0;
//					
//					ble_attempt_to_send(&buf[0],20);	// send back the port value
//			
//		}
//		else 	
		
			
	/* ==================================================================================== */		
			/*
			if(data_end)
			{
				uint8_t	buf[2];
				buf[0]='O';
				buf[1]='K';
				ble_attempt_to_send(&buf[0],2);	// send back the port value

			}
			*/
			
		
			
			
	
	

/**@snippet [Handling the data received over BLE] */





// ===================================================================================================== //
// ===================================================================================================== //
// ===================================================================================================== //









/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t         err_code;
    ble_nus_init_t   nus_init;
    
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;
    
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing security parameters.
 */
static void sec_params_init(void)
{
    m_sec_params.timeout      = SEC_PARAM_TIMEOUT;
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;  
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}


/**@brief       Function for handling an event from the Connection Parameters Module.
 *
 * @details     This function will be called for all events in the Connection Parameters Module
 *              which are passed to the application.
 *
 * @note        All this function does is to disconnect. This could have been done by simply setting
 *              the disconnect_on_fail config parameter, but instead we use the event handler
 *              mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;
    
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief       Function for handling errors from the Connection Parameters module.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    
    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t             err_code;
    ble_gap_adv_params_t adv_params;
    
    // Start advertising
    memset(&adv_params, 0, sizeof(adv_params));
    
    adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    adv_params.p_peer_addr = NULL;
    adv_params.fp          = BLE_GAP_ADV_FP_ANY;
    adv_params.interval    = APP_ADV_INTERVAL;
    adv_params.timeout     = APP_ADV_TIMEOUT_IN_SECONDS;

    err_code = sd_ble_gap_adv_start(&adv_params);
    APP_ERROR_CHECK(err_code);

    //nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);
}




// =================================================================================================== //
// ============================ BLE EVENT HANDLER ==================================================== //
// =================================================================================================== //




/**@brief       Function for the Application's S110 SoftDevice event handler.
 *
 * @param[in]   p_ble_evt   S110 SoftDevice event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t                         err_code;
    static ble_gap_evt_auth_status_t m_auth_status;
    ble_gap_enc_info_t *             p_enc_info;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
           // nrf_gpio_pin_set(CONNECTED_LED_PIN_NO);
           // nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
						data_length=0;
						data_end=false;
					//	app_uart_flush();
				
						Is_Connected=true;
						
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
					
           // nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            advertising_start();
						Is_Connected=false;
						if(new_code)
						{
							
							code_complete=true;
						}

            break;
            
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, 
                                                   BLE_GAP_SEC_STATUS_SUCCESS, 
                                                   &m_sec_params);
            APP_ERROR_CHECK(err_code);
            break;
            
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_AUTH_STATUS:
            m_auth_status = p_ble_evt->evt.gap_evt.params.auth_status;
            break;
            
        case BLE_GAP_EVT_SEC_INFO_REQUEST:
            p_enc_info = &m_auth_status.periph_keys.enc_info;
            if (p_enc_info->div == p_ble_evt->evt.gap_evt.params.sec_info_request.div)
            {
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, p_enc_info, NULL);
                APP_ERROR_CHECK(err_code);
            }
            else
            {
                // No keys found for this device
                err_code = sd_ble_gap_sec_info_reply(m_conn_handle, NULL, NULL);
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_GAP_EVT_TIMEOUT:
					
/* ========================================================================================================== */				
/* COMMENT TO DISABLE THE TIMEOUT EVENT FOR AVOIDING THE TURN OFF OF SYSTEM AFTER APP_ADV_TIMEOUT_IN_SECONDS  */				
 /* ========================================================================================================= */		                    			
				
				
				
					/*
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT)
            { 
                nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);

                // Configure buttons with sense level low as wakeup source.
                nrf_gpio_cfg_sense_input(WAKEUP_BUTTON_PIN,
                                         BUTTON_PULL,
                                         NRF_GPIO_PIN_SENSE_LOW);
                
                // Go to system-off mode (this function will not return; wakeup will cause a reset)
                err_code = sd_power_system_off();    
                APP_ERROR_CHECK(err_code);
            }
				*/
				advertising_start();
				
/* ========================================================================================================= */						
/* ========================================================================================================= */						
/* ========================================================================================================= */						
            break;
        case BLE_EVT_TX_COMPLETE:
            if(!ble_buffer_available) tx_complete = true;
            break;

        default:
            // No implementation needed.
            break;
    }
}


// ==================================================================================================== //
// ==================================================================================================== //
// ==================================================================================================== //







/**@brief       Function for dispatching a S110 SoftDevice event to all modules with a S110
 *              SoftDevice event handler.
 *
 * @details     This function is called from the S110 SoftDevice event interrupt handler after a
 *              S110 SoftDevice event has been received.
 *
 * @param[in]   p_ble_evt   S110 SoftDevice event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_conn_params_on_ble_evt(p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);
    on_ble_evt(p_ble_evt);
}


/**@brief   Function for the S110 SoftDevice initialization.
 *
 * @details This function initializes the S110 SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;
    
    // Initialize SoftDevice.
    SOFTDEVICE_HANDLER_INIT( NRF_CLOCK_LFCLKSRC_RC_250_PPM_250MS_CALIBRATION, NULL);

    // Enable BLE stack 
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
    ble_enable_params.gatts_enable_params.service_changed = IS_SRVC_CHANGED_CHARACT_PRESENT;
    err_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    // Subscribe for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}

/**@brief  Function for configuring the buttons.
 */

/*
static void buttons_init(void)
{
    nrf_gpio_cfg_sense_input(WAKEUP_BUTTON_PIN,
                             BUTTON_PULL, 
                             NRF_GPIO_PIN_SENSE_LOW);    
}

*/







/**@brief   Function for handling UART interrupts.
 *
 * @details This function will receive a single character from the UART and append it to a string.
 *          The string will be be sent over BLE when the last character received was a 'new line'
 *          i.e '\n' (hex 0x0D) or if the string has reached a length of @ref NUS_MAX_DATA_LENGTH.
 */

void uart_evt_callback(app_uart_evt_t * uart_evt)
{
    //uint32_t err_code;
	
    switch (uart_evt->evt_type)
    {
        case APP_UART_DATA:	
						//Data is ready on the UART					
            break;
						
		case APP_UART_DATA_READY:
            //Data is ready on the UART FIFO		
            break;
						
        case APP_UART_TX_EMPTY:
			//Data has been successfully transmitted on the UART
            break;
						
        default:
            break;
    }
    
}
/**@brief  Function for initializing the UART module.
 */



/*
static void uart_init(void)
{
    uint32_t err_code;
    
    APP_UART_FIFO_INIT(&comm_params,
                        RX_BUF_SIZE,
                        TX_BUF_SIZE,
                        uart_evt_callback,
                        UART_IRQ_PRIORITY,
                        err_code);
    
    APP_ERROR_CHECK(err_code);
}


*/






/* ======================================================================================= */
/* ================   PORTSET FUNCTION        =========================================== */
/* ======================================================================================= */

void port_set(uint8_t value,uint32_t port)
{
	
	switch(value)
	{
		case 73:
			nrf_gpio_cfg_input(port,NRF_GPIO_PIN_PULLDOWN);
			break;
		case 65:
			nrf_gpio_cfg_input(port,NRF_GPIO_PIN_PULLDOWN);
			break;
		case 79:
			nrf_gpio_cfg_output(port);
		break;
		case 80:
			nrf_gpio_cfg_output(port);
		break;
		case 'S':
			nrf_gpio_cfg_output(port);
		break;
		
	}
}


/* ======================================================================================= */




/* ======================================================================================= */
/* ================  BIBOX PORT CONFIGURATION    ========================================= */
/* ======================================================================================= */
	 

void port_configuration(void)
{
	
		array_index=8;
	port_set(code_array[array_index],A1);
		array_index++;
	port_set(code_array[array_index],A2);
		array_index++;
	port_set(code_array[array_index],A3);
		array_index++;
	port_set(code_array[array_index],A4);
		array_index++;
//	port_set(code_array[array_index],A5);
		array_index++;
//	port_set(code_array[array_index],A6);
		array_index++;
	port_set(code_array[array_index],B1);
		array_index++;
	port_set(code_array[array_index],B2);
		array_index++;
	port_set(code_array[array_index],B3);
		array_index++;
	port_set(code_array[array_index],B4);
		array_index++;
	port_set(code_array[array_index],C1);
		array_index++;
	port_set(code_array[array_index],C2);
		array_index++;
	port_set(code_array[array_index],C3);
		array_index++;
	port_set(code_array[array_index],C4);
		array_index=array_index+9;					//skipping 8 values(d1-d4,e1-e4)
	port_set(code_array[array_index],F1);
		array_index++;
	port_set(code_array[array_index],F2);
			array_index++;
//	port_set(code_array[array_index],F3);
			array_index++;
//	port_set(code_array[array_index],F4);
			array_index++;
	port_set(code_array[array_index],G1);
				array_index++;
	port_set(code_array[array_index],G2);
				array_index++;
//	port_set(code_array[array_index],G3);
				array_index++;
//	port_set(code_array[array_index],G4);
	
	
//============initialise pwm pins m1-m4=====================//

	       
    
		nrf_gpio_cfg_output(M1);
		nrf_gpio_cfg_output(M2);
		nrf_gpio_cfg_output(M3);
		nrf_gpio_cfg_output(M4);
		nrf_gpio_pin_clear(M1);
		nrf_gpio_pin_clear(M2);
		nrf_gpio_pin_clear(M3);
		nrf_gpio_pin_clear(M4);
		
		pwm_config.mode             = PWM_MODE_LED_100;		//pwm frequency is changed to 50hz by changing configurations
																											// in nrf_pwm.c file (pwm_max_value=312).
		if(((code_array[9]=='I')||(code_array[9]=='O')||(code_array[9]=='A'))&&((code_array[10]=='I')||(code_array[10]=='O')||(code_array[10]=='A')))	
		{
																											//Enable two channel pwm
		pwm_config.num_channels     = 2;									
    pwm_config.gpio_num[0]      = M1;									
    pwm_config.gpio_num[1]      = M3;      		
		nrf_pwm_init(&pwm_config);
		nrf_pwm_set_value(0, 0);
    nrf_pwm_set_value(1, 0);
		nrf_delay_ms(50);
       
		}
		
			else if(((code_array[9]=='P')||(code_array[9]=='S'))&&((code_array[10]=='P')||(code_array[10]=='S')))
		{
			pwm_config.num_channels     = 4;							//A2& A3 is port is pwm->> remap pwm to A2 & A4 							
			pwm_config.gpio_num[0]      = M1;							//Enable the four channel pwm	
			pwm_config.gpio_num[1]      = M3;
			pwm_config.gpio_num[2]      = A2;    
			pwm_config.gpio_num[3]      = A3;    
			nrf_pwm_init(&pwm_config);
			nrf_pwm_set_value(0, 0);
      nrf_pwm_set_value(1, 0);
      nrf_pwm_set_value(2,0);
			nrf_pwm_set_value(3,0);
				nrf_delay_ms(50);

		}
		
		
		
		
		else if(((code_array[9]=='P')||(code_array[9]=='S'))&&((code_array[10]!='P')&&(code_array[10]!='S')	))
		{
			pwm_config.num_channels     = 3;							 //A2 is port is pwm->> remap pwm to A2 							
			pwm_config.gpio_num[0]      = M1;							 // Enable three channel pwm	
			pwm_config.gpio_num[1]      = M3;
			pwm_config.gpio_num[2]      = A2;    
		
			nrf_pwm_init(&pwm_config);
			nrf_pwm_set_value(0, 0);
      nrf_pwm_set_value(1, 0);
      nrf_pwm_set_value(2,0);
				nrf_delay_ms(50);
		
		}
	else if(((code_array[9]!='P')&&(code_array[9]!='S'))&&((code_array[10]=='P')||(code_array[10]=='S'))	)
		{
			pwm_config.num_channels     = 3;						  //A3 is port is pwm->> remap pwm to A3 							
			pwm_config.gpio_num[0]      = M1;						  //Enable three channel pwm	
			pwm_config.gpio_num[1]      = M3;
			pwm_config.gpio_num[2]      = A3;    
		
			nrf_pwm_init(&pwm_config);
			nrf_pwm_set_value(0, 0);
      nrf_pwm_set_value(1, 0);
      nrf_pwm_set_value(2,0);
				nrf_delay_ms(50);

		}
	
		
		
		if(code_array[30]=='C'||code_array[30]=='M')	//enable uart for color sensor
		{
			
			simple_uart_config(RTS_PIN_NUMBER,BIBOX_TX_PIN_NUMBER,CTS_PIN_NUMBER,BIBOX_RX_PIN_NUMBER,false,UART_BAUDRATE_BAUDRATE_Baud9600);
			if(code_array[30]=='M')
			{
				mp3_set_vol (30);	
				mp3_source ();
				nrf_delay_ms(10);
			}
		}
		else {
			NRF_UART0->ENABLE=0;
		}
		if(code_array[30]=='U')
		{
										
				nrf_gpio_cfg_output(TRIGGER);								
				nrf_gpio_cfg_input(ECHO,NRF_GPIO_PIN_PULLDOWN);	
										
		}	
		if(code_array[34]=='U')
		{
										
				nrf_gpio_cfg_output(TRIGGER1);								
				nrf_gpio_cfg_input(ECHO1,NRF_GPIO_PIN_PULLDOWN);	
										
		}	
		if(code_array[14]=='R')	//enable software spi for the rfid
		{
			Soft_Spi_Initial();
			MFRC522_Init();
		
		}
		else if(code_array[14]=='D')
		{
			Soft_Spi_Initial();
			Max_Shutdown(false,5);
			Max_Decode_Mode(NO_DECODE,5);
			Max_Intensity(0xff,5);
			Max_Scan_Limit(0x7,5);
			Max_Display_test(false,5);
			Max_Display_Clear(5);
		}
		if(code_array[34]=='X')				//intialize the accelerometer
		{
			
			TWI_Initial();
			if(MMA8452_Check_Communication())
			{
				
				MMA8452_Initial();	
				MMA8452_Active_Mode();
				
			}

		}
		if(code_array[34]=='C')				//intialize the accelerometer
		{
			TWI_Initial1();
			if(!Hmc5883_Initial())
			{
				//failed initializing  
			}
		}
		else if(code_array[34]=='G')
		{
			//TWI_Initial1();// Second TWI initialization with interchanged i2c lines, 
			
			
			twi_master_init();
			nrf_delay_ms(50);
			
				APDS9960_Enable();
			
			
		}
		else if(code_array[34]=='T')
		{
			TWI_Initial1();// Second TWI initialization with interchanged i2c lines, 
			if(MPU6050_TestConnection())
			{
				MPU6050_Initialize();
			}
		}
		
		
		

	
	//adc_initial();			// intialize the adc, all channels are disabled by default

	
	
	
}	
///****************************************************************************************//





/* ======================================================================================= */
/* ================  OUT PUT SEQUENCE        ============================================= */
/* ======================================================================================= */


void output_sequence(void)
{
	uint32_t port;
	uint8_t buf[2];
	
	unsigned  int value=0,temp_val;
	array_index+=2;
	again:
	port=code_array[array_index];
	if(port=='}')			//************* if no output is enabled skip the parsing ******//
	{
		goto SKIP;
	}
	
	array_index++;
	value=code_array[array_index];
	value=(value<<8);
	array_index++;
	value|=code_array[array_index];
	

	//========== find the bibox port==============================================/
	
	switch(port)
	{
		case 1:							/* ============= A1 ======================= */
			port=A1;
			A1_val=value;
			if(value!=0)
			{					
					
				nrf_gpio_pin_set(port);	
			}
			else
			{
						
			nrf_gpio_pin_clear(port);
			}
		break;
		case 2:							/* ============= A2 ======================= */
			port=A2;
			
			if(code_array[9]==79)								// the port is not pwm
				{
					if(value!=0)
					{					
						nrf_gpio_pin_set(port);	
					
					}
					else
					{
						nrf_gpio_pin_clear(port);	
						
					}
					
				}
				
				else if (code_array[9]==80)		// port is pwm 
				{
										
								/*  remapping servo value(1800-5400) to pwm pulse width for						
									servo (9 to 40). For changing pulsewidth from 1ms to 2ms 	*/		
								/*	
								value=(((value-1800)*31)/3600)+9;				
								nrf_pwm_set_value(2, value);
								nrf_delay_ms(200);
								
								*/
					
						
					if(A2_val!=value)							//modify port only if the port has been changed
					{				
						
						temp_val=(((value)*312)/65536);	
						
						nrf_pwm_set_value(2, temp_val);
						
						
					}
			}
				
			else if(code_array[9]=='S')
			{
				
				
				temp_val=(((value)*1000)/65536);	// remapping value for the pwm channel
				value=((value-1800)*180)/3600;		//remapping value to 0-180 for the display
				
				if(A2_val!=value)
				{
				nrf_pwm_set_value(2, temp_val);
				}
				
				
				
			}
				
			
			
		A2_val=value;		
		
		break;
		case 3:										/* ============= A3 ======================= */
			port=A3;
			
		
		if(code_array[10]==79)								// the port is not pwm
				{
					if(value!=0)
					{					
						nrf_gpio_pin_set(port);	
					
					}
					else
					{
						nrf_gpio_pin_clear(port);	
						
					}
					
				}
				
				else if (code_array[10]==80)			// port is pwm 
				{
					
	/* servo value is remapped from 1800-5400 to  to 180 for the display */				
					
					
							
					
		
					
		/*  remapping servo value(1800-5400) to pwm pulse width for						
			servo (9 to 40). For changing pulsewidth from 1ms to 2ms 	*/		
				/*	
					value=(((value-1800)*31)/3600)+9;				
					nrf_pwm_set_value(2, value);
					nrf_delay_ms(200);
					
					*/
					if(A3_val!=value)				//modify port only if the port has been changed
					{				
							
							temp_val=(((value)*312)/65536);				
							if(((code_array[9]=='P')||(code_array[9]=='S'))&&((code_array[10]=='P')||(code_array[10]=='S')))
							{
								
								nrf_pwm_set_value(3, temp_val);
								
							}
							else 
							{
									
								nrf_pwm_set_value(2, temp_val);
								
							}
					}
			}
				
			
			
			else if(code_array[10]=='S')
			{
				temp_val=(((value)*1000)/65536);										//remapping value to 0-180 forthe display
				value=((value-1800)*180)/3600;											//remapping value for the pwm channel
					
				if(A3_val!=value)
				{
				
				if(((code_array[9]=='P')||(code_array[9]=='S'))&&((code_array[10]=='P')||(code_array[10]=='S')))	//if A2&A3 is pwm, then A3 pwm channel is 3,else 2
					{
								
						nrf_pwm_set_value(3, temp_val);
								
					}
					else 
					{
							
						nrf_pwm_set_value(2, temp_val);
				
					}
				}
				
				
			}
			
			
			
			
			
			
			
			
			
		
			A3_val=value;	
		
		
		break;
		case 4:											/* ============= A4 ======================= */
			port=A4;
			A4_val=value;
			if(value!=0)
				{					
						
					nrf_gpio_pin_set(port);	
				}
				else
				{
							
				nrf_gpio_pin_clear(port);
				}
		break;	
		case 5:
		//	port=A5;
		break;
		case 6:
		//	port=A6;
		break;		
		case 7:											/* ============= B1 ======================= */
			port=B1;
			B1_val=value;
				if(value!=0)
					{					
							
						nrf_gpio_pin_set(port);	
						
					}
					else
					{
								
					nrf_gpio_pin_clear(port);
					}
		break;		
		case 8:										/* ============= B2 ======================= */
			port=B2;
			B2_val=value;
			if(value!=0)
				{					
						
					nrf_gpio_pin_set(port);	
				}
				else
				{
							
				nrf_gpio_pin_clear(port);
				}
		break;		
		case 9:										/* ============= B3 ======================= */
			port=B3;
			B3_val=value;
			if(value!=0)
				{					
						
					nrf_gpio_pin_set(port);	
				}
				else
				{
							
				nrf_gpio_pin_clear(port);
				}
		break;		
		case 10:									/* ============= B4 ======================= */
			port=B4;
			B4_val=value;
			if(value!=0)
				{					
						
					nrf_gpio_pin_set(port);	
				}
				else
				{
							
				nrf_gpio_pin_clear(port);
				}
		break;		
		case 11:									/* ============= C1 ======================= */
			port=C1;
			C1_val=value;
			if(value!=0)
				{					
						
					nrf_gpio_pin_set(port);	
				}
				else
				{
							
				nrf_gpio_pin_clear(port);
				}
		break;	
		case 12:									/* ============= C2 ======================= */
			port=C2;
			C2_val=value;
			if(value!=0)
				{					
						
					nrf_gpio_pin_set(port);	
				}
				else
				{
							
				nrf_gpio_pin_clear(port);
				}
		break;	
		case 13:									/* ============= C3 ======================= */
			port=C3;
			C3_val=value;
			if(value!=0)
				{					
						
					nrf_gpio_pin_set(port);	
				}
				else
				{
							
				nrf_gpio_pin_clear(port);
				}
		break;	
		case 14:
			port=C4;								/* ============= C4 ======================= */
			C4_val=value;
			if(value!=0)
				{					
						
					nrf_gpio_pin_set(port);	
				}
				else
				{
							
				nrf_gpio_pin_clear(port);
				}
		
		break;	
		case 23:									/* ============= F1 ======================= */
			port=F1;
			F1_val=value;
			if(value!=0)
				{					
						
					nrf_gpio_pin_set(port);	
				}
				else
				{
							
				nrf_gpio_pin_clear(port);
				}
		break;	
		case 24:									/* ============= F2 ======================= */
			port=F2;
			F2_val=value;
			if(value!=0)
				{					
						
					nrf_gpio_pin_set(port);	
				}
				else
				{
							
				nrf_gpio_pin_clear(port);
				}
		break;		
		case 25:								
		/*	port=F3;
			F3_val=value;
			if(value!=0)
				{					
						
					nrf_gpio_pin_set(port);	
				}
				else
				{
							
				nrf_gpio_pin_clear(port);
				}
		*/
		break;
		case 26:
			/*
			port=F4;
			F4_val=value;
			if(value!=0)
				{					
						
					nrf_gpio_pin_set(port);	
				}
				else
				{
							
				nrf_gpio_pin_clear(port);
				}
		*/
		break;					
		case 27:
			port=G1;										/* ============= G1 ======================= */
			G1_val=value;
			if(value!=0)
				{					
						
					nrf_gpio_pin_set(port);	
				}
				else
				{
							
				nrf_gpio_pin_clear(port);
				}
		break;				
		case 28:											/* ============= G2	======================= */
			port=G2;
			G2_val=value;
			if(value!=0)
				{					
						
					nrf_gpio_pin_set(port);	
				}
				else
				{
							
				nrf_gpio_pin_clear(port);
				}
		break;				
		case 29:
			/*
			port=G3;
			G3_val=value;
			if(value!=0)
				{					
						
					nrf_gpio_pin_set(port);	
				}
				else
				{
							
				nrf_gpio_pin_clear(port);
				}
		*/
		break;				
		case 30:
			/*
			port=G4;
			G4_val=value;
			if(value!=0)
				{					
						
					nrf_gpio_pin_set(port);	
				}
				else
				{
							
				nrf_gpio_pin_clear(port);
				}
		*/
		break;		
		case 31:
			
		
//===============================================================================================================/
//===============   motor m1 control            =================================================================/
//===============================================================================================================/
		
	if(M1_Prev_val!=value)											//modify port only if the port has been changed
	{		
	
		if (value==20)
		{		
			
			  nrf_gpio_pin_clear(M1);								//===================================//
				nrf_gpio_pin_clear(M2);								//======stops motor 1================//
				nrf_pwm_set_value(0,0);
				M1_val=0;
			
		}
		
		else if(value>20)
		{		
			
			
				M1_val=value-20;
				temp_val=(value-20)*15.6;							//==========  motor m1 fwd   ========//
				nrf_gpio_pin_clear(M2);								//==========     m1=0        ========//					
				nrf_pwm_set_value(0, temp_val);				//==========   m2 pwm out    ========//	
																							// for moving forward M2 os 0 =======//
		}
		else if(value<20)
		{
				M1_val=value-20;
				temp_val=(20-value)*15.6;							//==========  motor m1 rev   ========//
				nrf_gpio_pin_set(M2);									//==========     m1=pwm      ========//					
				nrf_pwm_set_value(0,( 312-temp_val));	//==========     m2=0        ========//	
																							//== for moving backward M2 is 1 &
																							// pwm range is reversed             //
		}
		
	}
//===============================================================================================================//		

		M1_Prev_val=value;
		break;
		case 32:
//===============================================================================================================/
//===============   motor m2 control            =================================================================/
//===============================================================================================================/		
	
		if(M2_Prev_Val!=value)									//modify port only if the port has been changed
		{
		
				if (value==20)
				{		
						M2_val=0;
						nrf_gpio_pin_clear(M3);														//===================================//
						nrf_gpio_pin_clear(M4);														//======stops motor 1================//
						nrf_pwm_set_value(1,0);																									//===================================//
						
				}
				
				else if(value>20)
				{
						M2_val=value-20;
						temp_val=(value-20)*15.6;													//==========  motor m1 fwd   ========//
						nrf_gpio_pin_clear(M4);														//==========     m1=0        ========//	
						nrf_pwm_set_value(1, temp_val);										//==========   m2 pwm out    ========//	
								
				}
				else if(value<20)
				{
						M2_val=value-20;
						temp_val=(20-value)*15.6;													//==========  motor m1 rev   ========//
						nrf_gpio_pin_set(M4);															//==========     m1=pwm      ========//	
						nrf_pwm_set_value(1,(312- temp_val));							//==========     m2=0        ========//	
							
				}
				
					
		}
		
		M2_Prev_Val=value;
		break;

//===============================================================================================================//


			case 35:
				Bi_Counter1+=value;
			
			break;
			case 36:
				Bi_Counter2+=value;
			
			break;	
			case 37:
				Bi_Counter3+=value;
			
			break;			
			case 38:
				Bi_Counter1-=value;
			
			break;		
			case 39:
				Bi_Counter2-=value;
			
			break;		
			case 40:
				Bi_Counter3-=value;
			
			break;	
			case 50:
				Bi_Counter1=value;
			
			break;			
			case 51:
				Bi_Counter2=value;
			
			break;	
			case 52:
				Bi_Counter3=value;
			
			break;		
			case 53:
				Bi_Flag1=value;
			
			break;
			case 54:
				Bi_Flag2=value;
			
			break;		
			case 55:
				Bi_Flag3=value;
			
			break;		
			
case 56:
				array_index++;
				
				switch(code_array[array_index])
				{
					case 0: Bi_Data1=value;break;
					case 1: Bi_Data1=A1_val;break;
					case 2: Bi_Data1=A2_val;break;
					case 3: Bi_Data1=A3_val;break;
					case 4: Bi_Data1=A4_val;break;
					case 7: Bi_Data1=B1_val;break;
					case 8: Bi_Data1=B2_val;break;
					case 9: Bi_Data1=B3_val;break;
					case 10: Bi_Data1=B4_val;break;
					case 11: Bi_Data1=C1_val;break;
					case 12: Bi_Data1=C2_val;break;
					case 13: Bi_Data1=C3_val;break;
					case 14: Bi_Data1=C4_val;break;
					case 23: Bi_Data1 = F1_val; break;
					case 24: Bi_Data1 = F2_val; break;
					case 25: Bi_Data1 = F3_val; break;
					case 26: Bi_Data1 = F4_val; break;
					case 27: Bi_Data1 = G1_val; break;
					case 28: Bi_Data1 = G2_val; break;
					case 29: Bi_Data1 = G3_val; break;
					case 30: Bi_Data1 = G4_val; break;
					case 31: Bi_Data1 = M1_val; break;
					case 32: Bi_Data1 = M2_val; break;
					case 37: Bi_Data1 = Bi_Counter1; break;
					case 38: Bi_Data1 = Bi_Counter2; break;
					case 39: Bi_Data1 = Bi_Counter3; break;
					case 40: Bi_Data1 = Bi_Flag1; break;	
					case 41: Bi_Data1 = Bi_Flag2; break;	
					case 42: Bi_Data1 = Bi_Flag3; break;
					case 43: Bi_Data1 = Bi_Data1; break;
					case 44: Bi_Data1 = Bi_Data2; break;
					case 45: Bi_Data1 = Bi_Data3; break;
					case 46: Bi_Data1 = Ble_Remote; break;
					case 47: Bi_Data1 = Bt_Slider; break;
				}
				
					break;
	case 57:
				array_index++;
				
				switch(code_array[array_index])
				{
					case 0: Bi_Data2=value;break;
					case 1: Bi_Data2=A1_val;break;
					case 2: Bi_Data2=A2_val;break;
					case 3: Bi_Data2=A3_val;break;
					case 4: Bi_Data2=A4_val;break;
					case 7: Bi_Data2=B1_val;break;
					case 8: Bi_Data2=B2_val;break;
					case 9: Bi_Data2=B3_val;break;
					case 10: Bi_Data2=B4_val;break;
					case 11: Bi_Data2=C1_val;break;
					case 12: Bi_Data2=C2_val;break;
					case 13: Bi_Data2=C3_val;break;
					case 14: Bi_Data2=C4_val;break;
					case 23: Bi_Data2 = F1_val; break;
					case 24: Bi_Data2 = F2_val; break;
					case 25: Bi_Data2 = F3_val; break;
					case 26: Bi_Data2 = F4_val; break;
					case 27: Bi_Data2 = G1_val; break;
					case 28: Bi_Data2 = G2_val; break;
					case 29: Bi_Data2 = G3_val; break;
					case 30: Bi_Data2 = G4_val; break;
					case 31: Bi_Data2 = M1_val; break;
					case 32: Bi_Data2 = M2_val; break;
					case 37: Bi_Data2 = Bi_Counter1; break;
					case 38: Bi_Data2 = Bi_Counter2; break;
					case 39: Bi_Data2 = Bi_Counter3; break;
					case 40: Bi_Data2 = Bi_Flag1; break;	
					case 41: Bi_Data2 = Bi_Flag2; break;	
					case 42: Bi_Data2 = Bi_Flag3; break;
					case 43: Bi_Data2 = Bi_Data1; break;
					case 44: Bi_Data2 = Bi_Data2; break;
					case 45: Bi_Data2 = Bi_Data3; break;
					case 46: Bi_Data2 = Ble_Remote; break;
					case 47: Bi_Data2 = Bt_Slider; break;				
					
					
				}
			
			break;
				
case 58:                                                     
				array_index++;
				
				switch(code_array[array_index])
				{
					case 0: Bi_Data3=value;break;
					case 1: Bi_Data3=A1_val;break;
					case 2: Bi_Data3=A2_val;break;
					case 3: Bi_Data3=A3_val;break;
					case 4: Bi_Data3=A4_val;break;
					case 7: Bi_Data3=B1_val;break;
					case 8: Bi_Data3=B2_val;break;
					case 9: Bi_Data3=B3_val;break;
					case 10: Bi_Data3=B4_val;break;
					case 11: Bi_Data3=C1_val;break;
					case 12: Bi_Data3=C2_val;break;
					case 13: Bi_Data3=C3_val;break;
					case 14: Bi_Data3=C4_val;break;
					case 23: Bi_Data3 = F1_val; break;
					case 24: Bi_Data3 = F2_val; break;
					case 25: Bi_Data3 = F3_val; break;
					case 26: Bi_Data3 = F4_val; break;
					case 27: Bi_Data3 = G1_val; break;
					case 28: Bi_Data3 = G2_val; break;
					case 29: Bi_Data3 = G3_val; break;
					case 30: Bi_Data3 = G4_val; break;
					case 31: Bi_Data3 = M1_val; break;
					case 32: Bi_Data3 = M2_val; break;
					case 37: Bi_Data3 = Bi_Counter1; break;
					case 38: Bi_Data3	= Bi_Counter2; break;
					case 39: Bi_Data3 = Bi_Counter3; break;
					case 40: Bi_Data3 = Bi_Flag1; break;	
					case 41: Bi_Data3 = Bi_Flag2; break;	
					case 42: Bi_Data3 = Bi_Flag3; break;
					case 43: Bi_Data3 = Bi_Data1; break;
					case 44: Bi_Data3 = Bi_Data2; break;
					case 45: Bi_Data3 = Bi_Data3; break;
					case 46: Bi_Data3 = Ble_Remote; break;
					case 47: Bi_Data3 = Bt_Slider; break;				
				
			}		
			break;					
			
	case 59:		
			Millis=value;
			break;
	case 60:
			Dot_Matrix1=value;
			Send_Dotmatrix();
		
			break;
	
	case 61:
			Dot_Matrix2=value;
			Send_Dotmatrix();
		
			break;
	case 62:
			Dot_Matrix3=value;
			Send_Dotmatrix();
	
			break;
	case 63:
			Dot_Matrix4=value;
			Send_Dotmatrix();
	
			break;
	
	case 64:
			Dot_Matrix5=value;
			Send_Dotmatrix();
			break;
	case 65:			
			array_index++;			
			switch(code_array[array_index])
			{
					case 0: IOT_1=value;break;
					case 1: IOT_1=A1_val;break;
					case 2: IOT_1=A2_val;break;
					case 3: IOT_1=A3_val;break;
					case 4: IOT_1=A4_val;break;
					case 7: IOT_1=B1_val;break;
					case 8: IOT_1=B2_val;break;
					case 9: IOT_1=B3_val;break;
					case 10: IOT_1=B4_val;break;
					case 11: IOT_1=C1_val;break;
					case 12: IOT_1=C2_val;break;
					case 13: IOT_1=C3_val;break;
					case 14: IOT_1=C4_val;break;
					case 23: IOT_1 = F1_val; break;
					case 24: IOT_1 = F2_val; break;
					case 25: IOT_1 = F3_val; break;
					case 26: IOT_1 = F4_val; break;
					case 27: IOT_1 = G1_val; break;
					case 28: IOT_1 = G2_val; break;
					case 29: IOT_1 = G3_val; break;
					case 30: IOT_1 = G4_val; break;
					case 31: IOT_1 = M1_val; break;
					case 32: IOT_1 = M2_val; break;
					case 37: IOT_1 = Bi_Counter1; break;
					case 38: IOT_1	= Bi_Counter2; break;
					case 39: IOT_1 = Bi_Counter3; break;
					case 40: IOT_1 = Bi_Flag1; break;	
					case 41: IOT_1 = Bi_Flag2; break;	
					case 42: IOT_1 = Bi_Flag3; break;
					case 43: IOT_1 = Bi_Data1; break;
					case 44: IOT_1 = Bi_Data2; break;
					case 45: IOT_1 = Bi_Data3; break;
					case 46: IOT_1 = Ble_Remote; break;
					case 47: IOT_1 = Bt_Slider; break;				
				
			}
	
	
			break;
	case 66:
			array_index++;			
			switch(code_array[array_index])
				{
					case 0: IOT_2=value;break;
					case 1: IOT_2=A1_val;break;
					case 2: IOT_2=A2_val;break;
					case 3: IOT_2=A3_val;break;
					case 4: IOT_2=A4_val;break;
					case 7: IOT_2=B1_val;break;
					case 8: IOT_2=B2_val;break;
					case 9: IOT_2=B3_val;break;
					case 10: IOT_2=B4_val;break;
					case 11: IOT_2=C1_val;break;
					case 12: IOT_2=C2_val;break;
					case 13: IOT_2=C3_val;break;
					case 14: IOT_2=C4_val;break;
					case 23: IOT_2 = F1_val; break;
					case 24: IOT_2 = F2_val; break;
					case 25: IOT_2 = F3_val; break;
					case 26: IOT_2 = F4_val; break;
					case 27: IOT_2 = G1_val; break;
					case 28: IOT_2 = G2_val; break;
					case 29: IOT_2 = G3_val; break;
					case 30: IOT_2 = G4_val; break;
					case 31: IOT_2 = M1_val; break;
					case 32: IOT_2 = M2_val; break;
					case 37: IOT_2 = Bi_Counter1; break;
					case 38: IOT_2	= Bi_Counter2; break;
					case 39: IOT_2 = Bi_Counter3; break;
					case 40: IOT_2 = Bi_Flag1; break;	
					case 41: IOT_2 = Bi_Flag2; break;	
					case 42: IOT_2 = Bi_Flag3; break;
					case 43: IOT_2 = Bi_Data1; break;
					case 44: IOT_2 = Bi_Data2; break;
					case 45: IOT_2 = Bi_Data3; break;
					case 46: IOT_2 = Ble_Remote; break;
					case 47: IOT_2 = Bt_Slider; break;				
				
			}
	
	
	
			break;
	case 67:
			array_index++;			
			switch(code_array[array_index])
			{
					case 0: IOT_3=value;break;
					case 1: IOT_3=A1_val;break;
					case 2: IOT_3=A2_val;break;
					case 3: IOT_3=A3_val;break;
					case 4: IOT_3=A4_val;break;
					case 7: IOT_3=B1_val;break;
					case 8: IOT_3=B2_val;break;
					case 9: IOT_3=B3_val;break;
					case 10: IOT_3=B4_val;break;
					case 11: IOT_3=C1_val;break;
					case 12: IOT_3=C2_val;break;
					case 13: IOT_3=C3_val;break;
					case 14: IOT_3=C4_val;break;
					case 23: IOT_3 = F1_val; break;
					case 24: IOT_3 = F2_val; break;
					case 25: IOT_3 = F3_val; break;
					case 26: IOT_3 = F4_val; break;
					case 27: IOT_3 = G1_val; break;
					case 28: IOT_3 = G2_val; break;
					case 29: IOT_3 = G3_val; break;
					case 30: IOT_3 = G4_val; break;
					case 31: IOT_3 = M1_val; break;
					case 32: IOT_3 = M2_val; break;
					case 37: IOT_3 = Bi_Counter1; break;
					case 38: IOT_3	= Bi_Counter2; break;
					case 39: IOT_3 = Bi_Counter3; break;
					case 40: IOT_3 = Bi_Flag1; break;	
					case 41: IOT_3 = Bi_Flag2; break;	
					case 42: IOT_3 = Bi_Flag3; break;
					case 43: IOT_3 = Bi_Data1; break;
					case 44: IOT_3 = Bi_Data2; break;
					case 45: IOT_3 = Bi_Data3; break;
					case 46: IOT_3 = Ble_Remote; break;
					case 47: IOT_3 = Bt_Slider; break;				
				
			}
		
			break;
	case 68:
			array_index++;			
			switch(code_array[array_index])
			{
					case 0: IOT_4=value;break;
					case 1: IOT_4=A1_val;break;
					case 2: IOT_4=A2_val;break;
					case 3: IOT_4=A3_val;break;
					case 4: IOT_4=A4_val;break;
					case 7: IOT_4=B1_val;break;
					case 8: IOT_4=B2_val;break;
					case 9: IOT_4=B3_val;break;
					case 10: IOT_4=B4_val;break;
					case 11: IOT_4=C1_val;break;
					case 12: IOT_4=C2_val;break;
					case 13: IOT_4=C3_val;break;
					case 14: IOT_4=C4_val;break;
					case 23: IOT_4 = F1_val; break;
					case 24: IOT_4 = F2_val; break;
					case 25: IOT_4 = F3_val; break;
					case 26: IOT_4 = F4_val; break;
					case 27: IOT_4 = G1_val; break;
					case 28: IOT_4 = G2_val; break;
					case 29: IOT_4 = G3_val; break;
					case 30: IOT_4 = G4_val; break;
					case 31: IOT_4 = M1_val; break;
					case 32: IOT_4 = M2_val; break;
					case 37: IOT_4 = Bi_Counter1; break;
					case 38: IOT_4	= Bi_Counter2; break;
					case 39: IOT_4 = Bi_Counter3; break;
					case 40: IOT_4 = Bi_Flag1; break;	
					case 41: IOT_4 = Bi_Flag2; break;	
					case 42: IOT_4 = Bi_Flag3; break;
					case 43: IOT_4 = Bi_Data1; break;
					case 44: IOT_4 = Bi_Data2; break;
					case 45: IOT_4 = Bi_Data3; break;
					case 46: IOT_4 = Ble_Remote; break;
					case 47: IOT_4 = Bt_Slider; break;				
				
			}
	
	
	
	
	
			break;
	case 69:
			array_index++;			
			switch(code_array[array_index])
			{
					case 0: IOT_5=value;break;
					case 1: IOT_5=A1_val;break;
					case 2: IOT_5=A2_val;break;
					case 3: IOT_5=A3_val;break;
					case 4: IOT_5=A4_val;break;
					case 7: IOT_5=B1_val;break;
					case 8: IOT_5=B2_val;break;
					case 9: IOT_5=B3_val;break;
					case 10: IOT_5=B4_val;break;
					case 11: IOT_5=C1_val;break;
					case 12: IOT_5=C2_val;break;
					case 13: IOT_5=C3_val;break;
					case 14: IOT_5=C4_val;break;
					case 23: IOT_5 = F1_val; break;
					case 24: IOT_5 = F2_val; break;
					case 25: IOT_5 = F3_val; break;
					case 26: IOT_5 = F4_val; break;
					case 27: IOT_5 = G1_val; break;
					case 28: IOT_5 = G2_val; break;
					case 29: IOT_5 = G3_val; break;
					case 30: IOT_5 = G4_val; break;
					case 31: IOT_5 = M1_val; break;
					case 32: IOT_5 = M2_val; break;
					case 37: IOT_5 = Bi_Counter1; break;
					case 38: IOT_5	= Bi_Counter2; break;
					case 39: IOT_5 = Bi_Counter3; break;
					case 40: IOT_5 = Bi_Flag1; break;	
					case 41: IOT_5 = Bi_Flag2; break;	
					case 42: IOT_5 = Bi_Flag3; break;
					case 43: IOT_5 = Bi_Data1; break;
					case 44: IOT_5 = Bi_Data2; break;
					case 45: IOT_5 = Bi_Data3; break;
					case 46: IOT_5 = Ble_Remote; break;
					case 47: IOT_5 = Bt_Slider; break;				
				
			}
	
	
	
			break;
	case 70:
			array_index++;			
			switch(code_array[array_index])
			{
					case 0: IOT_6=value;break;
					case 1: IOT_6=A1_val;break;
					case 2: IOT_6=A2_val;break;
					case 3: IOT_6=A3_val;break;
					case 4: IOT_6=A4_val;break;
					case 7: IOT_6=B1_val;break;
					case 8: IOT_6=B2_val;break;
					case 9: IOT_6=B3_val;break;
					case 10: IOT_6=B4_val;break;
					case 11: IOT_6=C1_val;break;
					case 12: IOT_6=C2_val;break;
					case 13: IOT_6=C3_val;break;
					case 14: IOT_6=C4_val;break;
					case 23: IOT_6 = F1_val; break;
					case 24: IOT_6 = F2_val; break;
					case 25: IOT_6 = F3_val; break;
					case 26: IOT_6 = F4_val; break;
					case 27: IOT_6 = G1_val; break;
					case 28: IOT_6 = G2_val; break;
					case 29: IOT_6 = G3_val; break;
					case 30: IOT_6 = G4_val; break;
					case 31: IOT_6 = M1_val; break;
					case 32: IOT_6 = M2_val; break;
					case 37: IOT_6 = Bi_Counter1; break;
					case 38: IOT_6	= Bi_Counter2; break;
					case 39: IOT_6 = Bi_Counter3; break;
					case 40: IOT_6 = Bi_Flag1; break;	
					case 41: IOT_6 = Bi_Flag2; break;	
					case 42: IOT_6 = Bi_Flag3; break;
					case 43: IOT_6 = Bi_Data1; break;
					case 44: IOT_6 = Bi_Data2; break;
					case 45: IOT_6 = Bi_Data3; break;
					case 46: IOT_6 = Ble_Remote; break;
					case 47: IOT_6 = Bt_Slider; break;				
				
			}
	
	
	
	
	
			break;	
	case 71:
			array_index++;			
			switch(code_array[array_index])
			{
					case 0: IOT_7=value;break;
					case 1: IOT_7=A1_val;break;
					case 2: IOT_7=A2_val;break;
					case 3: IOT_7=A3_val;break;
					case 4: IOT_7=A4_val;break;
					case 7: IOT_7=B1_val;break;
					case 8: IOT_7=B2_val;break;
					case 9: IOT_7=B3_val;break;
					case 10: IOT_7=B4_val;break;
					case 11: IOT_7=C1_val;break;
					case 12: IOT_7=C2_val;break;
					case 13: IOT_7=C3_val;break;
					case 14: IOT_7=C4_val;break;
					case 23: IOT_7 = F1_val; break;
					case 24: IOT_7 = F2_val; break;
					case 25: IOT_7 = F3_val; break;
					case 26: IOT_7 = F4_val; break;
					case 27: IOT_7 = G1_val; break;
					case 28: IOT_7 = G2_val; break;
					case 29: IOT_7 = G3_val; break;
					case 30: IOT_7 = G4_val; break;
					case 31: IOT_7 = M1_val; break;
					case 32: IOT_7 = M2_val; break;
					case 37: IOT_7 = Bi_Counter1; break;
					case 38: IOT_7	= Bi_Counter2; break;
					case 39: IOT_7 = Bi_Counter3; break;
					case 40: IOT_7 = Bi_Flag1; break;	
					case 41: IOT_7 = Bi_Flag2; break;	
					case 42: IOT_7 = Bi_Flag3; break;
					case 43: IOT_7 = Bi_Data1; break;
					case 44: IOT_7 = Bi_Data2; break;
					case 45: IOT_7 = Bi_Data3; break;
					case 46: IOT_7 = Ble_Remote; break;
					case 47: IOT_7 = Bt_Slider; break;				
				
			}
	
	
	
			break;	
	case 72:
			array_index++;			
			switch(code_array[array_index])
			{
					case 0: IOT_8=value;break;
					case 1: IOT_8=A1_val;break;
					case 2: IOT_8=A2_val;break;
					case 3: IOT_8=A3_val;break;
					case 4: IOT_8=A4_val;break;
					case 7: IOT_8=B1_val;break;
					case 8: IOT_8=B2_val;break;
					case 9: IOT_8=B3_val;break;
					case 10: IOT_8=B4_val;break;
					case 11: IOT_8=C1_val;break;
					case 12: IOT_8=C2_val;break;
					case 13: IOT_8=C3_val;break;
					case 14: IOT_8=C4_val;break;
					case 23: IOT_8 = F1_val; break;
					case 24: IOT_8 = F2_val; break;
					case 25: IOT_8 = F3_val; break;
					case 26: IOT_8 = F4_val; break;
					case 27: IOT_8 = G1_val; break;
					case 28: IOT_8 = G2_val; break;
					case 29: IOT_8 = G3_val; break;
					case 30: IOT_8 = G4_val; break;
					case 31: IOT_8 = M1_val; break;
					case 32: IOT_8 = M2_val; break;
					case 37: IOT_8 = Bi_Counter1; break;
					case 38: IOT_8	= Bi_Counter2; break;
					case 39: IOT_8 = Bi_Counter3; break;
					case 40: IOT_8 = Bi_Flag1; break;	
					case 41: IOT_8 = Bi_Flag2; break;	
					case 42: IOT_8 = Bi_Flag3; break;
					case 43: IOT_8 = Bi_Data1; break;
					case 44: IOT_8 = Bi_Data2; break;
					case 45: IOT_8 = Bi_Data3; break;
					case 46: IOT_8 = Ble_Remote; break;
					case 47: IOT_8 = Bt_Slider; break;				
				
			}
	
	
	
	
			break;
	case 73:
			array_index++;			
			switch(code_array[array_index])
			{
					case 0: IOT_9=value;break;
					case 1: IOT_9=A1_val;break;
					case 2: IOT_9=A2_val;break;
					case 3: IOT_9=A3_val;break;
					case 4: IOT_9=A4_val;break;
					case 7: IOT_9=B1_val;break;
					case 8: IOT_9=B2_val;break;
					case 9: IOT_9=B3_val;break;
					case 10: IOT_9=B4_val;break;
					case 11: IOT_9=C1_val;break;
					case 12: IOT_9=C2_val;break;
					case 13: IOT_9=C3_val;break;
					case 14: IOT_9=C4_val;break;
					case 23: IOT_9 = F1_val; break;
					case 24: IOT_9 = F2_val; break;
					case 25: IOT_9 = F3_val; break;
					case 26: IOT_9 = F4_val; break;
					case 27: IOT_9 = G1_val; break;
					case 28: IOT_9 = G2_val; break;
					case 29: IOT_9 = G3_val; break;
					case 30: IOT_9 = G4_val; break;
					case 31: IOT_9 = M1_val; break;
					case 32: IOT_9 = M2_val; break;
					case 37: IOT_9 = Bi_Counter1; break;
					case 38: IOT_9	= Bi_Counter2; break;
					case 39: IOT_9 = Bi_Counter3; break;
					case 40: IOT_9 = Bi_Flag1; break;	
					case 41: IOT_9 = Bi_Flag2; break;	
					case 42: IOT_9 = Bi_Flag3; break;
					case 43: IOT_9 = Bi_Data1; break;
					case 44: IOT_9 = Bi_Data2; break;
					case 45: IOT_9 = Bi_Data3; break;
					case 46: IOT_9 = Ble_Remote; break;
					case 47: IOT_9 = Bt_Slider; break;				
				
			}
			break;
	case 74:
			array_index++;			
			switch(code_array[array_index])
				{
					case 0: IOT_10=value;break;
					case 1: IOT_10=A1_val;break;
					case 2: IOT_10=A2_val;break;
					case 3: IOT_10=A3_val;break;
					case 4: IOT_10=A4_val;break;
					case 7: IOT_10=B1_val;break;
					case 8: IOT_10=B2_val;break;
					case 9: IOT_10=B3_val;break;
					case 10: IOT_10=B4_val;break;
					case 11: IOT_10=C1_val;break;
					case 12: IOT_10=C2_val;break;
					case 13: IOT_10=C3_val;break;
					case 14: IOT_10=C4_val;break;
					case 23: IOT_10 = F1_val; break;
					case 24: IOT_10 = F2_val; break;
					case 25: IOT_10 = F3_val; break;
					case 26: IOT_10 = F4_val; break;
					case 27: IOT_10 = G1_val; break;
					case 28: IOT_10 = G2_val; break;
					case 29: IOT_10 = G3_val; break;
					case 30: IOT_10 = G4_val; break;
					case 31: IOT_10 = M1_val; break;
					case 32: IOT_10 = M2_val; break;
					case 37: IOT_10 = Bi_Counter1; break;
					case 38: IOT_10	= Bi_Counter2; break;
					case 39: IOT_10 = Bi_Counter3; break;
					case 40: IOT_10 = Bi_Flag1; break;	
					case 41: IOT_10 = Bi_Flag2; break;	
					case 42: IOT_10 = Bi_Flag3; break;
					case 43: IOT_10 = Bi_Data1; break;
					case 44: IOT_10 = Bi_Data2; break;
					case 45: IOT_10 = Bi_Data3; break;
					case 46: IOT_10 = Ble_Remote; break;
					case 47: IOT_10 = Bt_Slider; break;				
				
			}
			break;
	
	case 75:
			Ble_Remote=value;
	
			break;
	case 76:
			Servo_Direct1=value;
	
			break;	
	case 77:
			Servo_Direct2=value;
		
	
			break;
	
	case 78:
			/////////////////// MP3 Module ////////////////////
		if(Song_Number!=value)
		{
			mp3_play(value);
			nrf_delay_ms(10);
			mp3_playback();
			nrf_delay_ms(10);
			Song_Number=value;
		}
			
			break;
	
	
	case 79:
		////////////////////////////////////////////////
		////////////	image port  //////////////////////
		////////////////////////////////////////////////
	if(value>0&&value<101)
	{
		Image_Enabled=true;
						
		SetAddrWindow(2,2,129,129);
		nrf_gpio_pin_clear(FLASH_CS);
		Winbond_Send_Address(65536+(value*32768));
		
	
				for(int i=0;i<128*128;i++)
				{
					//simple_uart_put(SPI_Write_Read(0));

					//ST7735_Write_Data(ImageArray[i]>>8);
					//ST7735_Write_Data(ImageArray[i]);
					
					ST7735_Write_Data(SPI_Write_Read(0));
					ST7735_Write_Data(SPI_Write_Read(0));

				}

		nrf_gpio_pin_set(FLASH_CS);
		Image_Enabled=true;
		nrf_delay_ms(2000);
		
		Initialize_Display();
				Image_Enabled=false;
		nrf_delay_ms(1000);
	}		
			break;		
		case 80:
			Bt_Slider=value;
			break;	
		
	 
	case 98:			

		// Bi MP3 implementation
			buf[0]=value;
			buf[1]=value>>8;
			ble_attempt_to_send(&buf[0],2);	// send back the port value
			
			break;		
		
				


		
	}																			
		
		
				
	array_index++;
				
			
//* ====================================================================================================== *//				
//* ================== IF END OF OUTPUT IS NOT DETECTED LOOP AGAIN ======================================= *//	
//* ====================================================================================================== *//	
				SKIP:		
				if(code_array[array_index]!=125)			
				{
					goto again;
				}
				if(array_index>2047)	// fatal error corrupt program
				{
					
					Corrupt_Program=true;
					return;
				}
				
//* ====================================================================================================== *//			
				
	
}



/* ======================================================================================= */
/* ================   WAIT SEQUENCE              ========================================= */
/* ======================================================================================= */




void wait_sequence(void)
{
	uint32_t delay=0x00000000;
	uint32_t temp1=0x00000000;
	array_index++;
	delay=code_array[array_index];
	delay=delay<<24;
	array_index++;
	temp1=code_array[array_index];
	temp1=temp1<<16;
	delay=delay|temp1;
	temp1=0;
	array_index++;
	temp1=code_array[array_index];
	temp1=temp1<<8;
	delay|=temp1;
	array_index++;
	delay|=code_array[array_index];
	
	
			
/* ======================================================================================= */
/* ================ make delay, if  receiving new code exit ============================== */
/* ======================================================================================= */								
									
						for(int k=0;k<delay;k++)			
						{			
											
											if((new_code==true)||(Prog_Pin_Count>8)||(chip_overheat))
											{
																																																						
												break;;
											}	
													
							nrf_delay_us(950);
		
  
						}
	
	
}



//****************************************************************************************** //

bool Check_Signed_Condition(unsigned char operat,signed int rvalue,signed int lvalue,signed int bvalue)
{
	bool condition=false;
	switch(operat)
			{
				case 60:
					if(rvalue<lvalue)
					{
						condition=true;
					}
					break;
				case 61:
					if(rvalue==lvalue)
					{
						condition=true;
					}
					break;
				case 62:
					if(rvalue>lvalue)
					{
						condition=true;
					}
					break;
				case 33:
					if(rvalue!=lvalue)
					{
						condition=true;
					}
					break;
				case	63:
					
					if(lvalue>bvalue)
					{
						if((rvalue>bvalue)&&(rvalue<lvalue))
						{
							condition=true;
						}
					
					}
					else
					{
						if((rvalue<bvalue)&&(rvalue>lvalue))
						{
							condition=true;
						}
						
						
					}
							
						break;
				case 64:	
					
					
					if(lvalue>bvalue)
					{
						if((rvalue<bvalue)&&(rvalue>lvalue))
						{
							condition=true;
						}
					
					}
					else
					{
						if((rvalue>bvalue)&&(rvalue<lvalue))
						{
							condition=true;
						}
						
						
					}
					break;
					
									
					
				
					
			}
	
	
	return condition;
	
	
}



/* ======================================================================================== */
/* ================  FIND CONDITION           ============================================= */
/* ======================================================================================== */

bool find_condition(void)
{
	
	
	uint32_t pin;
	uint32_t rvalue=0,lvalue=0,bvalue=0;
	uint8_t operat,temp_uint8;
	bool condition=false;
	signed int acc_val[3];
	unsigned int ulval;
	float headingDegrees=0;
	unsigned short int Temp[10];


	
	array_index++;
	bvalue=code_array[array_index];
	bvalue<<=8;
	array_index++;
	bvalue|=code_array[array_index];		
	array_index++;		
	pin=code_array[array_index];
	array_index++;
	lvalue=code_array[array_index];
	lvalue=lvalue<<8;
	array_index++;
	lvalue|=code_array[array_index];
	array_index++;
	operat=code_array[array_index];
		switch(pin)
		{
			case 1:
				if(code_array[8]==73)
				{
					rvalue=nrf_gpio_pin_read(A1);
					
					
					
				}
				else if (code_array[8]==65)
				{
					
					rvalue=adc_read(4ul);
					
					if(adc_semaphore)
					{
						adc_semaphore=false;
						rvalue=adc_read(4ul);
					}
						
					
					
				}
				
				A1_val=rvalue;
				
				
				break;
			case 2:
				
				if(code_array[9]==73)
				{
					rvalue=nrf_gpio_pin_read(A2);
					
				}
				
				else if (code_array[9]==65)
				{
					rvalue=adc_read(8ul);				//A2 has no analog I/P
					
					if(adc_semaphore)
					{
						adc_semaphore=false;
						rvalue=adc_read(8ul);
					}
				}
				
				A2_val=rvalue;				
				
				break;
		case 3:
				if(code_array[10]==73)
				{
					rvalue=nrf_gpio_pin_read(A3);
					
				}
				
				
				else if (code_array[10]==65)
				{															
					rvalue=adc_read(16ul);
					
					if(adc_semaphore)
					{
						adc_semaphore=false;
						rvalue=adc_read(16ul);
					}
				}
				A3_val=rvalue;
				
				break;
				
		case 4:
				if(code_array[11]==73)
				{
					rvalue=nrf_gpio_pin_read(A4);
					
				}
				else if (code_array[11]==65)
				{
					rvalue=adc_read(32ul);
					
					if(adc_semaphore)
					{
						adc_semaphore=false;
						rvalue=adc_read(32ul);
					}
					
				}
				A4_val=rvalue;
				break;
		case 5:
				//if(code_array[12]==73)
				//{
			//		rvalue=nrf_gpio_pin_read(A4);
			//	}
				/*
				else if (code_array[12]==65)
				{
					rvalue=adc_read(16ul);
				}
				*/
				break;		
		case 6:
				//if(code_array[13]==73)
				//{
				//	rvalue=nrf_gpio_pin_read(A4);
				//}
				/*
				else if (code_array[13]==65)
				{
					rvalue=adc_read(32ul);
				}
				*/
				break;
	case 7:
				if(code_array[14]==73)
				{
					rvalue=nrf_gpio_pin_read(B1);
					
					
					B1_val=rvalue;
				}	
//				else if (code_array[14]==65)
//				{
//					rvalue=adc_read(64ul);
//					
//					if(adc_semaphore)
//					{
//						adc_semaphore=false;
//							rvalue=adc_read(64ul);
//					}

//				}
				
				
				
				break;
	case 8:
				if(code_array[15]==73)
				{
					rvalue=nrf_gpio_pin_read(B2);
					
					
					
				}	
				else if (code_array[15]==65)
				{
					rvalue=adc_read(64ul);
					if(adc_semaphore)
					{
						adc_semaphore=false;
						rvalue=adc_read(64ul);
					}
					
				}
				
				
				
				B2_val=rvalue;
				
				break;	
	case 9:
				if(code_array[16]==73)
				{
					rvalue=nrf_gpio_pin_read(B3);
					
					
				}	
				else if (code_array[16]==65)
				{
					rvalue=adc_read(128ul);
					if(adc_semaphore)
					{
						adc_semaphore=false;
						rvalue=adc_read(128ul);
					}
					
					
				}
				
				
				
				B3_val=rvalue;
				break;
	case 10:
				if(code_array[17]==73)
				{
					rvalue=nrf_gpio_pin_read(B4);
				}
					
				else if (code_array[17]==65)
				{
					rvalue=adc_read(1ul);
					
					if(adc_semaphore)
					{
						adc_semaphore=false;
						rvalue=adc_read(1ul);
					}
				}
				
				
				B4_val=rvalue;
					
					
				
				break;
	case 11:
				if(code_array[18]==73)
				{
					rvalue=nrf_gpio_pin_read(C1);
					C1_val=rvalue;
					
					
				}	
				break;
	case 12:
				if(code_array[19]==73)
				{
					rvalue=nrf_gpio_pin_read(C2);
					C2_val=rvalue;
					
					
				}	
				break;		
	case 13:
				if(code_array[20]==73)
				{
					rvalue=nrf_gpio_pin_read(C3);
					C3_val=rvalue;
					
					
				}	
				break;	
	case 14:
				if(code_array[21]==73)
				{
					rvalue=nrf_gpio_pin_read(C4);
					C4_val=rvalue;
					
					
				}	
				break;		
	case 23:
				if(code_array[30]==73)
				{
					rvalue=nrf_gpio_pin_read(F1);
					F1_val=rvalue;
					
					
				}	
				break;				
	case 24:
				if(code_array[31]==73)
				{
					rvalue=nrf_gpio_pin_read(F2);
					F2_val=rvalue;
					
					
				}	
				break;			
	case 25:
				if(code_array[32]==73)
				{
					/*
					rvalue=nrf_gpio_pin_read(F3);
					F3_val=rvalue;
					*/
					
				}	
				break;
	case 26:
		/*
				if(code_array[33]==73)
				{
					rvalue=nrf_gpio_pin_read(F4);
					F4_val=rvalue;
					
					
				}	
		*/
				break;

				
case 27:
				if(code_array[34]==73)
				{
					rvalue=nrf_gpio_pin_read(G1);
					G1_val=rvalue;
					
				}
				
				break;
case 28:
				if(code_array[35]==73)
				{
					rvalue=nrf_gpio_pin_read(G2);
					G2_val=rvalue;
					
				}
				
				break;				
				
case 29:
	/*
				if(code_array[36]==73)
				{
					rvalue=nrf_gpio_pin_read(G3);
					G3_val=rvalue;
					
				}
				else if (code_array[36]==65)
				{
					rvalue=adc_read(16ul);
					G3_val=rvalue;
				}
*/
				break;				
case 30:
	/*
				if(code_array[37]==73)
				{
					rvalue=nrf_gpio_pin_read(G4);
					G4_val=rvalue;
					
				}
				else if (code_array[37]==65)
				{
					rvalue=adc_read(32ul);
					G4_val=rvalue;
				}
	*/
				break;


				
				
		case 31:
			
				rvalue=Bi_Counter1;
				break;
		
		case 32:
			
				rvalue=Bi_Counter2;
				break;	
		case 33:
			
				rvalue=Bi_Counter3;
				break;	

		case 35:
			
				rvalue=Bi_Flag1;
				break;	

		case 36:
			
				rvalue=Bi_Flag2;
				break;		
				
		case 37:
			
				rvalue=Bi_Flag3;
				break;	

		case 38:
			
				rvalue=Ble_Remote;
				break;	
		case 39:
			
				rvalue=Bt_Slider;
				break;	

		case 42:
			
				rvalue=Bi_Data1;
			break;		
	
		case 43:
			
				rvalue=Bi_Data2;
			break;			

		case 44:
			
				rvalue=Bi_Data3;
			break;
		case 45:
			
			if(ULTRASONIC_READ1(&ulval))
			{
				Ultra1=ulval;
				
			}
			rvalue= Ultra1;
			
			
			break;
		
		case 46:		
			if(ULTRASONIC_READ2(&ulval))
			{
				Ultra2=ulval;
				
			}
			rvalue= Ultra2;
			break;	
		
		case 47:	
			read_color_sensor();
			rvalue=Red; 
			break;	
		
		case 48:
			read_color_sensor();
			rvalue=Green; 
			break;
		
		case 49:		
			read_color_sensor();
			rvalue=Blue; 
			break;	
		
		case 50:
			rvalue=Read_Rfid();
			Rfid_val=rvalue;
			break;
		case 51:
			rvalue=Millis;						// Millis is 32 byte, join msb and lsb 
			lvalue|=((bvalue<<16)&0xffff0000);
		
		  break;
		case 52:	
			rvalue=BAT;
			break;
		case 53:
			rvalue=IOT_1;
			break;
		case 54:
			rvalue=IOT_2;
			break;
		case 55:
			rvalue=IOT_3;
			break;
		case 56:
			rvalue=IOT_4;
			break;
		case 57:
			rvalue=IOT_5;
			break;
		case 58:
			rvalue=IOT_6;
			break;		
		case 59:
			rvalue=IOT_7;
			break;		
		case 60:
			rvalue=IOT_8;
			break;		
		case 61:
			rvalue=IOT_9;
			break;
		case 62:
			rvalue=IOT_10;
			break;
		case 63:			
			rvalue=R_HOUR;
			rvalue<<=8;
			rvalue|=R_MIN;			
			break;
		
		case 64:			// x direction port		
			
		
			if(MMA8452_Read_X(&acc_val[0]))
			{
				Acc_X_Val=acc_val[0];
			}
			return Check_Signed_Condition(operat,Acc_X_Val,(lvalue-1024),bvalue);						
			
									
				
		case 65:			// y direction port		
		
			if(MMA8452_Read_Y(&acc_val[1]))
			{
				Acc_Y_Val=acc_val[1];
			}
			return Check_Signed_Condition(operat,Acc_Y_Val,(lvalue-1024),bvalue);
								
								
			
		case 66:			// z direction port		
			
			if(MMA8452_Read_Z(&acc_val[2]))
			{
				Acc_Z_Val=acc_val[2];
			}	
			return Check_Signed_Condition(operat,Acc_Z_Val,(lvalue-1024),bvalue);
							
			
		case 67:	//Digital compass
				
				if( Hmc5883_readDegree(&headingDegrees) )
				{
				
					rvalue=(unsigned int)headingDegrees;
				}	
		
			
			break;		
				
	 case 69:	// APDS Color sensor
				if(APDS9960_readAmbientLight(&Temp[0]))
				{
					APDS9960_Val.Color.Red=Temp[1];
					
				}
			rvalue=APDS9960_Val.Color.Red;
			break;					
				
	 case 70:	
				if(APDS9960_readAmbientLight(&Temp[0]))
				{
					APDS9960_Val.Color.Green=Temp[2];
					
				}
			rvalue=APDS9960_Val.Color.Green;
			break;						
			
	 case 71:	
				if(APDS9960_readAmbientLight(&Temp[0]))
				{
					APDS9960_Val.Color.Blue=Temp[3];
					
				}
			rvalue=APDS9960_Val.Color.Blue;
			break;						
	 case 72:	
//				if(MPU6050_ReadGyro(&MPU6050_Val.GyroX,&MPU6050_Val.GyroY,&MPU6050_Val.GyroZ))
//				{
//				return Check_Signed_Condition(operat,MPU6050_Val.GyroX,(lvalue-65536),bvalue);	
//						
//				}
				
			//rvalue=APDS9960_Val.Color.Blue;	
					
									
			break;	 
			
	case 73:	
//				if(MPU6050_ReadGyro(&MPU6050_Val.GyroX,&MPU6050_Val.GyroY,&MPU6050_Val.GyroZ))
//				{
//				return Check_Signed_Condition(operat,MPU6050_Val.GyroY,(lvalue-65536),bvalue);	
//						
//				}
		if(APDS9960_ReadProximity(&temp_uint8))
					{
						APDS9960_Val.Proximity=temp_uint8;
					}
				rvalue=APDS9960_Val.Proximity;
									
			break;	 				
	case 74:	
//				if(MPU6050_ReadGyro(&MPU6050_Val.GyroX,&MPU6050_Val.GyroY,&MPU6050_Val.GyroZ))
//				{
//				return Check_Signed_Condition(operat,MPU6050_Val.GyroZ,(lvalue-65336),bvalue);	
//						
//				}
				if(APDS9960_ReadGesture(&temp_uint8))
				{
							if(temp_uint8!=DIR_NONE)
							{
								APDS9960_Val.Gesture=temp_uint8;
							}
				}
				rvalue=APDS9960_Val.Gesture;
				
									
			break;	 			
				
		
			
			}	
		
			
			
		
		
			switch(operat)
			{
				case 60:
					if(rvalue<lvalue)
					{
						condition=true;
					}
					break;
				case 61:
					if(rvalue==lvalue)
					{
						condition=true;
					}
					break;
				case 62:
					if(rvalue>lvalue)
					{
						condition=true;
					}
					break;
				case 33:
					if(rvalue!=lvalue)
					{
						condition=true;
					}
					break;
				case	63:
					
					if(lvalue>bvalue)
					{
						if((rvalue>bvalue)&&(rvalue<lvalue))
						{
							condition=true;
						}
					
					}
					else
					{
						if((rvalue<bvalue)&&(rvalue>lvalue))
						{
							condition=true;
						}
						
						
					}
							
						break;
				case 64:	
					
					
					if(lvalue>bvalue)
					{
						if((rvalue<bvalue)||(rvalue>lvalue))
						{
							condition=true;
						}
					
					}
					else
					{
						if((rvalue>bvalue)||(rvalue<lvalue))
						{
							condition=true;
						}
						
						
					}
					break;
					
									
					
				
					
			}
	
	
	return(condition);
	
	
	
	
	
	
	
	
}

/* =========================================================================================== */
/* ======================= CLEAR ALL PORTS =================================================== */
/* =========================================================================================== */

void clear_ports(void)
{
	
		nrf_gpio_pin_clear(A1);
		nrf_gpio_pin_clear(A2);
		nrf_gpio_pin_clear(A3);
		nrf_gpio_pin_clear(A4);
		nrf_gpio_pin_clear(B1);
		nrf_gpio_pin_clear(B2);
		nrf_gpio_pin_clear(B3);
		nrf_gpio_pin_clear(B4);
		nrf_gpio_pin_clear(C1);
		nrf_gpio_pin_clear(C2);
		nrf_gpio_pin_clear(C3);
		nrf_gpio_pin_clear(C4);
		nrf_gpio_pin_clear(F1);
		nrf_gpio_pin_clear(F2);
//		nrf_gpio_pin_clear(F3);
//		nrf_gpio_pin_clear(F4);
		nrf_gpio_pin_clear(G1);
		nrf_gpio_pin_clear(G2);
//		nrf_gpio_pin_clear(G3);
//		nrf_gpio_pin_clear(G4);
	  
		nrf_pwm_set_enabled(false);
		nrf_gpio_pin_clear(M1);														
	  nrf_gpio_pin_clear(M2);
		nrf_gpio_pin_clear(M3);														
	  nrf_gpio_pin_clear(M4);	
}


void read_ultrasonic(void)
{
		unsigned int ulval;
		/***************  read ultrasonic value *************************************/
																		
									
								
									if(code_array[30]=='U')
									{
										
										if(ULTRASONIC_READ1(&ulval))
										{
											Ultra1=ulval;
										}
									}	
									else if(code_array[30]=='C')
									{
										
										read_color_sensor();
									}	
									
									
									if(code_array[34]=='U')
									{
										
										if(ULTRASONIC_READ2(&ulval))
										{
											Ultra2=ulval;
										}
										
									}	else if(code_array[34]=='X')
									{
										
										
										signed int acc_val[3];
										//MMA8452_Standby_Mode();
										//MMA8452_Initial();	
										//MMA8452_Active_Mode();
										//MMA8452_Read_Raw_Values(&acc_val[0]);
										if(MMA8452_Read_X(&acc_val[0]))
										{
												Acc_X_Val=acc_val[0];
										}
											
										if(MMA8452_Read_Y(&acc_val[1]))
										{
												Acc_Y_Val=acc_val[1];
										}	
										if(MMA8452_Read_Z(&acc_val[2]))
										{
												Acc_Z_Val=acc_val[2];
										}
									
										
										
									}	
									else if(code_array[34]=='G')
									{
										uint16_t Temp[4];
										uint8_t temp_uint8;
										if(APDS9960_readAmbientLight(&Temp[0]))
										{
											APDS9960_Val.Color.Red=Temp[1];
											APDS9960_Val.Color.Green=Temp[2];
											APDS9960_Val.Color.Blue=Temp[3];
					
										}
										if(APDS9960_ReadProximity(&temp_uint8))
										{
												APDS9960_Val.Proximity=temp_uint8;
										}
										if(APDS9960_ReadGesture(&temp_uint8))
										{
											if(temp_uint8!=DIR_NONE)
											{
											APDS9960_Val.Gesture=temp_uint8;
											}
										}
										
										
									}
									else if(code_array[34]=='T')
									{
										if(MPU6050_ReadGyro(&MPU6050_Val.GyroX,&MPU6050_Val.GyroY,&MPU6050_Val.GyroZ))
										{
				
			
			
										}
									}
								
									if(code_array[14]=='R')	//Read rfid value
									{
										Rfid_val=Read_Rfid();
									}
	/****************************************************************************/								
}













//*======================================================================================= *//	
//*========================== FLASH WRITE FUNCTIONS       ================================ *//	
//*======================================================================================= *//	


void Write_to_External(void)
{
	
	Winbond_Release_Powerdown();
	Winbond_Write_Enable();
	Winbond_Chip_32kErase(0);
	while(Winbond_Flash_Busy()==true){};
	int j=0;
	uint32_t m=0x000000;
			for(int i=0;i<8;i++)
			{	
					
					Winbond_Write_Enable();
					Winbond_Write_8bit(m,256,&code_array[0]);
					while(Winbond_Flash_Busy()==true);
					j=j+128;
					m+=256;
					
			}	
		





}


void Read_Flash_External(unsigned int len,unsigned char * buf)
{
	
	
			nrf_gpio_pin_clear(FLASH_CS);
			SPI_Write_Read(SPIFLASH_ARRAYREADLOWFREQ);
			SPI_Write_Read(0);
			SPI_Write_Read(0);
			SPI_Write_Read(0);		
	
				for(int i=0;i<len;i++)
				{
					//simple_uart_put(SPI_Write_Read(0));
					code_array[i]=SPI_Write_Read(0);
					

				}

				nrf_gpio_pin_set(FLASH_CS);
	
	
	
	
	
}

			
void Clear_External_Flash(void)
{
	
	Winbond_Release_Powerdown();
	Winbond_Write_Enable();
	Winbond_Chip_32kErase(0);
	while(Winbond_Flash_Busy()==true){};
}









/* ============================================================================= */
/* ========================= ERASE THE FLASH MEMORY ============================ */
/* ============================================================================= */


								

								
void Clear_Flash(void)
{

retval = pstorage_block_identifier_get(&base_handle, 0, &block_handle);
		
	if (retval == NRF_SUCCESS)
	{
	 // Get Block Identifier successful.
				
	}
	else
	{
	 // Failed to get block id, take corrective action.
	}	
		
	err_code=app_timer_stop(refresh_timer_id);	
	pstorage_wait_flag = 1;
	retval = pstorage_clear(&block_handle, 1024);	
	while(pstorage_wait_flag) { power_manage(); }  		//clear first page of flash memmory		
	
	
	retval = pstorage_block_identifier_get(&base_handle, 1, &block_handle);
		
	if (retval == NRF_SUCCESS)
	{
		// Get Block Identifier successful.
															
	}
	else
	{
	  // Failed to get block id, take corrective action.
  }	
														
														
	pstorage_wait_flag = 1;
	retval = pstorage_clear(&block_handle, 1024);	
	while(pstorage_wait_flag) { power_manage(); }  		//clear second page of flash memmory




}										
								


/* =========================================================================================== */
/* ================== Web Bisoft Generated Function ========================================== */
/* =========================================================================================== */


/* =============== Initialization function ============ */


void Bisoft_Generated_Settings(void)
{
	
	












}



/* ============= Web Bisoft generated loop ============ */


void Bisoft_Generated_Loop(void)
{
	
	
	
	
	
	
	
	
}







/* =========================================================================================== */
/* =================================== MAIN FUNCTION ========================================= */
/* =========================================================================================== */








int main(void)
{
    	
    
    // Initialize
    //leds_init();
    timers_init();
    //buttons_init();
    //uart_init();
    ble_stack_init();
    gap_params_init();
    services_init();
    advertising_init();
    conn_params_init();
    sec_params_init();
    sd_clock_hfclk_request()	;																		// request crystal clock
		err_code = softdevice_sys_evt_handler_set(sys_evt_dispatch);	
	  APP_ERROR_CHECK(err_code);
		
		
		
   // uart_putstring((const uint8_t *)START_STRING);
    
    
   // simple_uart_config(RTS_PIN_NUMBER,TX_PIN_NUMBER,CTS_PIN_NUMBER,RX_PIN_NUMBER,false,UART_BAUDRATE_BAUDRATE_Baud9600);
		
		
		
		nrf_gpio_cfg_output(ST7735_BACKLIGHT);
		nrf_gpio_cfg_input(Prog_Pin,NRF_GPIO_PIN_PULLUP);
		nrf_gpio_cfg_input(OVER_HEAT,NRF_GPIO_PIN_PULLUP);
		nrf_gpio_cfg_input(Bat_pin,NRF_GPIO_PIN_NOPULL);
	
		
/* ==============    initialises SPI0 ======================================================== */
/*   SPI 8MHz speed, MSB first, sample on leading edge, clock active high 										 */
/* =========================================================================================== */

		
		ST7735_SPI_Initial();
		fill_display(0,0,130,132,ST7735_WHITE);	
		ST7735_Initial();
		Winbond_SPI_Initial();
		
		if(!Winbond_Check_Communication())
		{
			fill_display(0,0,130,132,ST7735_BLACK);	
			SetAddrWindow(2 , 55, 129, 132);	
			ST7735_Put_String2X("  Memory Failure.. !",ST7735_RED,ST7735_BLACK);		
			nrf_delay_ms(2000);
			
			
		}
	
	
		Winbond_Release_Powerdown();									//release flash ic from sleep
	


////////////////////////////////////////////////////
///////////// checking reset reason ////////////////
////////////////////////////////////////////////////


		if((NRF_POWER->RESETREAS&0x01)!=0)
		{
			goto START;
		}
		
		
////////////////////////////////////////////////////
////////////////////////////////////////////////////
////////////////////////////////////////////////////	
		
		
		
		
		
		
/* =================  OLED initalizes ========================================================*/
	
		
		fill_display(0,0,130,132,ST7735_BLACK);				//clear display
		

		//SetAddrWindow(2+13,2+35,101+13,44+35	);
		SetAddrWindow(2,2,129,129	);
		
																									// reading image from the flash IC
		nrf_gpio_pin_clear(FLASH_CS);
		SPI_Write_Read(SPIFLASH_ARRAYREADLOWFREQ);
		SPI_Write_Read(0x01);
		SPI_Write_Read(0x00);
		SPI_Write_Read(0x00);
		
	
				for(int i=0;i<128*128;i++)
				{
					//simple_uart_put(SPI_Write_Read(0));

					//ST7735_Write_Data(ImageArray[i]>>8);
					//ST7735_Write_Data(ImageArray[i]);
					
					ST7735_Write_Data(SPI_Write_Read(0));
					ST7735_Write_Data(SPI_Write_Read(0));

				}

		nrf_gpio_pin_set(FLASH_CS);
		
		nrf_gpio_pin_set(ST7735_BACKLIGHT);
				
		//Winbond_Power_Down();													 // sleep flash IC		
				
				
				
			
		nrf_delay_ms(2000);
				
				
				
				fill_display(0,0,130,132,ST7735_BLACK);				//clear display
		

		//SetAddrWindow(2+13,2+35,101+13,44+35	);
		SetAddrWindow(2,2,129,129	);
		
																									// reading image from the flash IC
		nrf_gpio_pin_clear(FLASH_CS);
		SPI_Write_Read(SPIFLASH_ARRAYREADLOWFREQ);
		SPI_Write_Read(0x00);
		SPI_Write_Read(0x80);
		SPI_Write_Read(0x00);
		
	
				for(int i=0;i<128*128;i++)
				{
					//simple_uart_put(SPI_Write_Read(0));

					//ST7735_Write_Data(ImageArray[i]>>8);
					//ST7735_Write_Data(ImageArray[i]);
					
					ST7735_Write_Data(SPI_Write_Read(0));
					ST7735_Write_Data(SPI_Write_Read(0));

				}

		nrf_gpio_pin_set(FLASH_CS);
		
		nrf_gpio_pin_set(ST7735_BACKLIGHT);
										

nrf_delay_ms(2000);
				
		
				
				
///////////////////////////////////////////////////////////
///////////////// SHOW WELCOME SCREEN /////////////////////
///////////////////////////////////////////////////////////
	
		nrf_gpio_pin_clear(FLASH_CS);
		SPI_Write_Read(SPIFLASH_ARRAYREADLOWFREQ);
		SPI_Write_Read(0x00);
		SPI_Write_Read(0x80);
		SPI_Write_Read(0x00);
				for(int i=0;i<10;i++)
				{	
					NAME[i]=SPI_Write_Read(0);
				}
				
				
				
				
/* ==============  reading MAC ID ====================== */
							
				
	fill_display(0,0,130,132,ST7735_BLACK);
	SetAddrWindow(2, 25, 129, 129);
	ST7735_Put_String3X("  WELCOME",ST7735_YELLOW,ST7735_BLACK);
	SetAddrWindow(2, 45, 129, 132);
	ST7735_PutChar3X(NAME[0],ST7735_YELLOW,ST7735_BLACK);
	ST7735_PutChar3X(NAME[1],ST7735_YELLOW,ST7735_BLACK);
	ST7735_PutChar3X(NAME[2],ST7735_YELLOW,ST7735_BLACK);
	ST7735_PutChar3X(NAME[3],ST7735_YELLOW,ST7735_BLACK);
	ST7735_PutChar3X(NAME[4],ST7735_YELLOW,ST7735_BLACK);
	ST7735_PutChar3X(NAME[5],ST7735_YELLOW,ST7735_BLACK);
	ST7735_PutChar3X(NAME[6],ST7735_YELLOW,ST7735_BLACK);
	ST7735_PutChar3X(NAME[7],ST7735_YELLOW,ST7735_BLACK);
	ST7735_PutChar3X(NAME[8],ST7735_YELLOW,ST7735_BLACK);
	ST7735_PutChar3X(NAME[9],ST7735_YELLOW,ST7735_BLACK);


	uint8_t temp;
	SetAddrWindow(0, 75, 129, 132);
	ST7735_Put_String1X("      Device ID",ST7735_WHITE,ST7735_BLACK);
	SetAddrWindow(15, 85, 129, 132);
	temp=0xff&(NRF_FICR->DEVICEADDR[1]>>8);	
	temp|=0xc0;	//msb of mac id should be 1 ,Ble std
	ST7735_PutChar1X(To_ASCII[temp>>4],ST7735_WHITE,ST7735_BLACK);
	ST7735_PutChar1X(To_ASCII[temp&0x0f],ST7735_WHITE,ST7735_BLACK);
	ST7735_PutChar1X(':',ST7735_WHITE,ST7735_BLACK);
	temp=0xff&(NRF_FICR->DEVICEADDR[1]);	
	ST7735_PutChar1X(To_ASCII[temp>>4],ST7735_WHITE,ST7735_BLACK);
	ST7735_PutChar1X(To_ASCII[temp&0x0f],ST7735_WHITE,ST7735_BLACK);
	ST7735_PutChar1X(':',ST7735_WHITE,ST7735_BLACK);
	temp=0xff&(NRF_FICR->DEVICEADDR[0]>>24);	
	ST7735_PutChar1X(To_ASCII[temp>>4],ST7735_WHITE,ST7735_BLACK);
	ST7735_PutChar1X(To_ASCII[temp&0x0f],ST7735_WHITE,ST7735_BLACK);
	ST7735_PutChar1X(':',ST7735_WHITE,ST7735_BLACK);
	temp=0xff&(NRF_FICR->DEVICEADDR[0]>>16);	
	ST7735_PutChar1X(To_ASCII[temp>>4],ST7735_WHITE,ST7735_BLACK);
	ST7735_PutChar1X(To_ASCII[temp&0x0f],ST7735_WHITE,ST7735_BLACK);
	ST7735_PutChar1X(':',ST7735_WHITE,ST7735_BLACK);
	temp=0xff&(NRF_FICR->DEVICEADDR[0]>>8);	
	ST7735_PutChar1X(To_ASCII[temp>>4],ST7735_WHITE,ST7735_BLACK);
	ST7735_PutChar1X(To_ASCII[temp&0x0f],ST7735_WHITE,ST7735_BLACK);	
	ST7735_PutChar1X(':',ST7735_WHITE,ST7735_BLACK);
	temp=0xff&(NRF_FICR->DEVICEADDR[0]);	
	ST7735_PutChar1X(To_ASCII[temp>>4],ST7735_WHITE,ST7735_BLACK);
	ST7735_PutChar1X(To_ASCII[temp&0x0f],ST7735_WHITE,ST7735_BLACK);



/* ============================================================== */

	SetAddrWindow(0, 110, 129, 132);
	ST7735_Put_String1X("    Android(v3.1.8)	",ST7735_YELLOW,ST7735_BLACK);

	nrf_delay_ms(2500);


	START:
	nrf_gpio_pin_set(ST7735_BACKLIGHT);	




//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////










/* ================================================================================ */
/* =========================    initialize app timer ============================== */
/* ================================================================================ */


err_code=app_timer_create(&refresh_timer_id,APP_TIMER_MODE_REPEATED ,refresh);





			
/* ==================================================================================== */
/* ======================= READ BYTE STREAM FROM FLASH MEMORY ========================= */
/* ==================================================================================== */

//	Read_Flash(2048,&code_array[0]);
//	
//	for(int i=0;i<100;i++)
//	{
//		simple_uart_put(code_array[i]);
//	}
	
	
	
	
	
	

/* ================================================================================= */
/* ============================== FLASH memory initializing ======================== */
/* ================================================================================= */

		
		retval = pstorage_init();
		if(retval == NRF_SUCCESS)
		{
			// Module initialization successful.
			
		}
		else
		{
		// Initialization failed, take corrective action.
		}



	param.block_size  = 1024;									/////////// MEMORY BLOCK SIZE /////////////
	param.block_count = 2;										/////////// SELECT TWO BLOCK	/////////////
																						/////////// TOTAL 2kb is used /////////////
	param.cb          = bibox_cb_handler;
			
	retval = pstorage_register(&param, & base_handle);
			
		if (retval == NRF_SUCCESS)
		{
			// Registration successful.
		
		}
		else
		{
			// Failed to register, take corrective action.
		}
			


		
	retval = pstorage_block_identifier_get(&base_handle, 0, &block_handle);

		if (retval == NRF_SUCCESS)
		{
			// Get Block Identifier successful.

		}
		else
		{
			// Failed to get block id, take corrective action.
		}	

/* ==================================================================================== */
/* ======================= READ BYTE STREAM FROM FLASH MEMORY ========================= */
/* ==================================================================================== */


/* =======================  reading first block of memory ============================  */
	

	pstorage_wait_flag = 1; 
	pstorage_load(code_array, &block_handle, 1024, 0);
	while(pstorage_wait_flag==1){power_manage();}  

//									for(int j=0;j<2000;j++)
//									{
//									
//									//while(app_uart_put(code_array[j]) != NRF_SUCCESS);	
//									SEGGER_RTT_printf(0, "%d ", code_array[j], RTT_CTRL_RESET);
//									}

	
	
	/* =======================  reading second block of memory ============================*/
	
	
	
	retval = pstorage_block_identifier_get(&base_handle, 1, &block_handle);

		if (retval == NRF_SUCCESS)
		{
			// Get Block Identifier successful.

		}
		else
		{
			// Failed to get block id, take corrective action.
		}	
	pstorage_wait_flag = 1; 
	pstorage_load(&code_array[1024], &block_handle, 1024, 0);
	while(pstorage_wait_flag==1){power_manage();}  
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
		
/*==================================================================================*/
/*====================== Initialize display with datas =============================*/
/*==================================================================================*/ 	

	Initialize_Display();
	
	
	Display_BleIcon();
	
	
/*===================================================================================*/
/*=================== Resetting port values =========================================*/
/*===================================================================================*/	
		
	if_count=0;
	loop_depth=0;															

	A1_val=0;
	A2_val=0;
	A3_val=0;
	A4_val=0;
	M1_val=0;
	M2_val=0;
	M1_Prev_val=100;
	M2_Prev_Val=100;
	Bat_prev_val=100;	
	B1_val=0;									
	B2_val=0;										
	B3_val=0;										
	B4_val=0;		
	C1_val=0;	
	C2_val=0;	
	C3_val=0;
	C4_val=0;									
	G1_val=0;
	G2_val=0;									
	G3_val=0;
	G4_val=0;
	F1_val=0;									
	F2_val=0;									
	F3_val=0;									
	F4_val=0;	
	Ultra1=0;
	Ultra2=0;
	Ble_Remote=0;
	Bt_Slider=0;
	Bi_Counter1=0;
	Bi_Counter2=0;
	Bi_Counter3=0;
	Bi_Flag1=0;
	Bi_Flag2=0;
	Bi_Flag3=0;
	Bi_Data1=0;
	Bi_Data2=0;
	Bi_Data3=0;
	IOT_1=0;
	IOT_2=0;
	IOT_3=0;
	IOT_4=0;
	IOT_5=0;
	IOT_6=0;
	IOT_7=0;
	IOT_8=0;
	IOT_9=0;
	IOT_10=0;
	SEC=0;
	Ultra1_En=false;
	Ultra2_En=false;
	Red=0;
	Green=0;
	Blue=0;
	Rfid_val=0;
	Millis=0;
	System_Error=false;
	chip_overheat=false;
	Dot_Matrix1=0;
	Dot_Matrix2=0;
	Dot_Matrix3=0;
	Dot_Matrix4=0;
	Dot_Matrix5=0;
	HOUR=0;
	HOUR_Prev=0;
	MIN=0;
	MIN_Prev=0;
	SEC=0;
	
	Refresh_OLED();										
	err_code=app_timer_start(refresh_timer_id, 8192, NULL);	// Star the timer tick
	
	
	advertising_start();		//Start advertising of bluetooth
	sd_ble_gap_tx_power_set(4);
	
	
	if(code_array[30]=='U'||code_array[34]=='U')
	{
		timeslot_sd_init();
	}
	else
	{
		sd_radio_session_close();		
	}
	
	
	nrf_delay_ms(1000);
	
	
	
	
	
	
	
//simple_uart_config(10,8,11,9,false,UART_BAUDRATE_BAUDRATE_Baud9600);	
//simple_uart_putstring("Start...\n");
//signed int rxr[10];			
//	unsigned char txr[10];	
//		TWI_Initial();
//	
//	if(MMA8452_Check_Communication()==0x2a)
//	{
//		
//		simple_uart_putstring("MMA8452 is detected\n");
//		
//	}
//	else
//	{
//		simple_uart_putstring("Failed\n");
//	}
//MMA8452_Standby_Mode();
//MMA8452_Initial();	
//MMA8452_Active_Mode();
//		
//		while(true){				
//	MMA8452_Standby_Mode();
//	MMA8452_Initial();	
//	MMA8452_Active_Mode();
//			
//	MMA8452_Read_Raw_Values(rxr);
//	A1_val=rxr[0];
//	A2_val=rxr[1];	
//	A3_val=rxr[2];			
//				
//	//nrf_delay_ms(10);
//			
//			
//			
//			
//		
//		}
		
	
	
//	SetAddrWindow(2,2,129,129);
	

	
	
//	nrf_delay_ms(4000);
//	uint32_t err_code = sd_softdevice_disable();
//  APP_ERROR_CHECK(err_code);

//  interrupts_disable();

//  err_code = sd_softdevice_vector_table_base_set(0x249f0);
//  APP_ERROR_CHECK(err_code);

//  bootloader_util_app_start(0x249f0);
	
	
	
	

/* ========================================================================================== */
/* ========================================================================================== */
/* ========================================================================================== */
	




//**********************************************************************//

		
    // Enter main loop
		
    //for (;;)
   // { 
        /*Stop reading new data if there are no ble buffers available */
     /*   if(ble_buffer_available)
        {
            if(app_uart_get(&newbyte) == NRF_SUCCESS)
            {
                data_array[index++] =  newbyte;
               
                if (index >= (BLE_NUS_MAX_DATA_LEN))
				{
                    ble_buffer_available=ble_attempt_to_send(&data_array[0],index);
                 	if(ble_buffer_available) index=0;
				}
            }
        }
			
			*/
        /* Re-transmission if ble_buffer_available was set to false*/
       /* if(tx_complete)
        {
            tx_complete=false;
            
            ble_buffer_available=ble_attempt_to_send(&data_array[0],index);
            if(ble_buffer_available) index =0;
        }

        //power_manage();
				
				
       
        // Add a delay to control the speed of the sine wave
       // nrf_delay_ms(8000);
				
    }*/
	
		
		
			
/* ======================================================================================= */
/* ================  BIBOX PARSING STARTS     ============================================ */
/* ======================================================================================= */
		
				
			
        if ((code_array[3]==0x01)&&(code_array[4]==0x01)&&(code_array[5]==0x53)&&(code_array[6]==0x45)&&(code_array[7]==0x54))
				{
			
					port_configuration();			//configuring all I/O pins
					
					
////////////////////////////////////////////////////////////////////
///////////////// Reading time /////////////////////////////////////
////////////////////////////////////////////////////////////////////					
					
					if(code_array[64]=='t')
					{
					
						R_HOUR=code_array[65];					
						R_MIN=code_array[66];
					}
					
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////					
					
					
					
					array_index=67;						//skipping all zeros
					
					
			
								while(true)
								{		
									read_ultrasonic();				
									
									switch (code_array[array_index])
									{
									case 111:
										
										output_sequence();
										break;
									case 119:
										wait_sequence();
										break;
									case 100:
										
										if_count++;
									if(find_condition()==false)
									{
//* ========================= skip output sequence ======================================================= *//										
										
												while(if_count>0)
												{
													array_index++;
														
																if((code_array[array_index]==0)&&(code_array[array_index+1]==69)&&(code_array[array_index+2]==68))
																{
																	if_count=0;
																	
																}	
												
													
												}
										
										array_index +=2;
										
										
										
										
									}
//* ====================================================================================================== *//									
									
									
									
										break;
									case 108:						// loop sequence
									array_index +=3;
									loop_depth++;
									loop_count[loop_depth]=code_array[array_index];
									loop_pc[loop_depth]=array_index+1;
								
										
										break;									
									
		
				
									case 0:
										
										if((code_array[array_index+1]==0)&&(code_array[array_index+2]==0))	//if data is not valid goto sleep
										{
											goto wait;
										}
									
									
									
										
									break;
									
										
									
									default:
									
									break;							
					
					
									}
						
					
								array_index++;	
																			

										 
																																			
/* ======================================================================================= */
/* ================  LOOPING SEQUENCES        ============================================ */
/* ======================================================================================= */										
									

									if((code_array[array_index]==0)&&(code_array[array_index+1]==69)&&(code_array[array_index+2]==76))
									{
									
									
													if(loop_count[loop_depth]>1)
													{
										
														loop_count[loop_depth]--;
														array_index=loop_pc[loop_depth];
														
													}
													else
													{
														
														
															if(loop_count[1]>1)
															{
																loop_depth--;		
																
																array_index=loop_pc[loop_depth];
																loop_count[loop_depth]--;
																
															}
															else
															{
															
																array_index+=3;
															
															}
															
													}
																																																																	
	
									}/////////////// infinite loop implementation /////////////////////////
									else if((code_array[array_index]==0)&&(code_array[array_index+1]==69)&&(code_array[array_index+2]==77))
									{
											
											if(loop_break==false)
											{
												array_index=loop_pc[loop_depth];
											}else{
												
												array_index+=3;
											}
									
									}
/* ======================================================================================= */
/* ======================================================================================= */									
									/* END OF IF CONDITON   */
									
												
										  if((code_array[array_index]==0)&&(code_array[array_index+1]==69)&&(code_array[array_index+2]==68))
												{
													if_count=0;
													array_index+=3;
																	
												}	
												
												
/* ======================================================================================= */
/* ======================================================================================= */													
								/*        REPEAT		*/	
												
									
	 									  if((code_array[array_index]==82)&&(code_array[array_index+1]==83)&&(code_array[array_index+2]==84))
								        	{
													array_index=67;
														
													}
													
/* ======================================================================================= */
/* ======================================================================================= */												
								/*          END      */
													
													
													
										 if((code_array[array_index]==69)&&(code_array[array_index+1]==78)&&(code_array[array_index+2]==68))
												{
													
												goto End;
													
												
						
												}
											
											
/* ======================================================================================= */
/* ======================================================================================= */							
									if(array_index>2047)
									{
										//app_uart_put('a');
										goto wait;
									}
									
									
									
							
									
/* ======================================================================================= */
/* ================  NEW DATA CAME WRITE TO FLASH  ======================================= */
/* ======================================================================================= */								
									
											if(new_code==true||chip_overheat||Corrupt_Program||Img_receiving)
											{
																																																			
												goto wait;
											}	
													
										 
/* ======================================================================================	*/
/* =========== if Prog swith is pressed more than 4 sec, erase the memory & restart====== */
/* ====================================================================================== */
										 
													if(Prog_Pin_Count>8)
													{
														Prog_Pin_Count=0;
														Clear_Flash();	
														Clear_External_Flash();
														fill_display(0,0,129,130,ST7735_BLACK);
														SetAddrWindow(2, 50, 129, 132);	
														ST7735_Put_String2X("    ...ERASED...",ST7735_YELLOW,ST7735_BLACK);	
														err_code=app_timer_stop(refresh_timer_id);
														clear_ports();
														nrf_delay_ms(3000);
														sd_nvic_SystemReset();
														
														
													}
										 


									
									
																			
												
						}		/*end loop */		
												
												
												
																		

				}
				
	

/* ======================================================================================= */
/* ============================= INFINITE LOOP, WAIT FOR NEW PROGRAM ===================== */
/* ======================================================================================= */				

				wait:
				clear_ports();
				End:
				Bisoft_Generated_Settings();
			
										while(true)
										{	
											read_ultrasonic();
											
											//power_manage();
											if(new_code==true)
											{
												err_code=app_timer_stop(refresh_timer_id);		
												fill_display(0,0,130,132,ST7735_BLACK);
												
												SetAddrWindow(2 , 55, 129, 132);	
												ST7735_Put_String2X("   ...MEMORISING...",ST7735_GREEN,ST7735_BLACK);	
												
												while(code_complete!=true);			//wait untill all the datas are received	
													
												bibox_pstorage_write();
												//Write_to_External();
												code_complete=false;
												new_code=false;
												
												goto START;
												
											}
											else if(System_Error)
											{
												err_code=app_timer_stop(refresh_timer_id);				
												fill_display(0,0,130,132,ST7735_BLACK);											
												SetAddrWindow(2 , 55, 129, 132);	
												ST7735_Put_String2X("  ...SYTEM ERROR!...",ST7735_RED,ST7735_BLACK);		
												nrf_delay_ms(2000);
												
												goto START;
												
											}
											else if(chip_overheat)
												
											{
												nrf_delay_ms(1000);
												while(nrf_gpio_pin_read(OVER_HEAT));	// wait till the IC is cool down
												goto START;
											}
											else if(Img_receiving)
											{
												err_code=app_timer_stop(refresh_timer_id);				
												fill_display(0,0,130,132,ST7735_BLACK);	
												SetAddrWindow(5,50,129,129);
												ST7735_Put_String1X("Receiving Image....",ST7735_GREEN,ST7735_BLACK);
												uint32_t m;
												if(image_counter==1)
												{
												m=0x008000;
												}
												if(image_counter==2)
												{
												m=0x010000;
												}
												
												//m=65536+(img_location*32768);
												Winbond_Release_Powerdown();
												Winbond_Write_Enable();												
												Winbond_Chip_32kErase(m);
												while(Winbond_Flash_Busy()==true);
												
													
												
												
												int Block_number=0;
												
												
												SetAddrWindow(25,65,129,129);
												sprintf (char_array, "%d",(Block_number));
												ST7735_Put_String1X(char_array,ST7735_GREEN,ST7735_BLACK);
												ST7735_PutChar1X('%',ST7735_GREEN,ST7735_BLACK);
												
												
												Send_Ack();
												while(Block_number<128)
												{
												
														while(Img_block_Complete==false);
														Img_block_Complete=false;
															
																	
																	Winbond_Write_Enable();
																	Winbond_Write_8bit(m,256,&code_array[0]);
																	while(Winbond_Flash_Busy()==true);																
																	m+=256;
																	
															
															
												SetAddrWindow(25,65,129,129);
												
												sprintf (char_array, "%d",Block_number*100/127);
												ST7735_Put_String1X(char_array,ST7735_GREEN,ST7735_BLACK);
												ST7735_PutChar1X('%',ST7735_GREEN,ST7735_BLACK);			
															
												Block_number++;
												
												Send_Ack();// send back the ack
																							
												}
												
												
												while(Img_receiving==true);
												
//												m=0x008000;
//												Winbond_Release_Powerdown();
//												Winbond_Write_Enable();												
//												Winbond_Chip_32kErase(m);
//												while(Winbond_Flash_Busy()==true);
//												Winbond_Write_Enable();
//												Winbond_Write_8bit(m,10,&NAME[0]);
//												while(Winbond_Flash_Busy()==true);	
												
												
												data_length=0;
//												sd_nvic_SystemReset();
												goto START;
												
												
												
											}else if(Corrupt_Program)
											{
												Corrupt_Program=false;
												Clear_Flash();
												fill_display(0,0,130,130,ST7735_BLACK);
												SetAddrWindow(4, 55, 129, 132);	
												ST7735_Put_String2X("...CORRUPT PROGRAM...",ST7735_RED,ST7735_BLACK);	
												nrf_delay_ms(3000);												
												fill_display(0,0,130,130,ST7735_BLACK);
												SetAddrWindow(5, 55, 129, 132);	
												ST7735_Put_String2X("    ...ERASED...",ST7735_YELLOW,ST7735_BLACK);	
												nrf_delay_ms(3000);
												sd_nvic_SystemReset();
												
											}

													if(Prog_Pin_Count>8)
													{
														Prog_Pin_Count=0;	
														Clear_Flash();	
														Clear_External_Flash();
														fill_display(0,0,130,132,ST7735_BLACK);
														SetAddrWindow(5, 55, 129, 132);	
														ST7735_Put_String2X("    ...ERASED...",ST7735_YELLOW,ST7735_BLACK);	
														nrf_delay_ms(3000);
														sd_nvic_SystemReset();
														
														
													}
	
										
													
													
										Bisoft_Generated_Loop();			
													

										 
										}
	

		}
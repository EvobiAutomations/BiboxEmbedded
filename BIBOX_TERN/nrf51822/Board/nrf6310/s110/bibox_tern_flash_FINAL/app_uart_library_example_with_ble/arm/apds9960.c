#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "twi.h"
#include "apds9960.h"
#include "simple_uart.h"
#include "twi_master.h"




unsigned char GestureR[32],GestureL[32],GestureU[32],GestureD[32],length;

int gesture_ud_delta_;
int gesture_lr_delta_;
int gesture_ud_count_;
int gesture_lr_count_;
int gesture_near_count_;
int gesture_far_count_;
int gesture_state_;
int gesture_motion_;


bool APDS9960_Initial(void)
{
	unsigned char txr[10];
	APDS9960_setMode(ALL,0);
	txr[0]=APDS9960_ATIME;	
	txr[1]=DEFAULT_ATIME;
	
	if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,2,true))
		{
			return false;
		}
	
	txr[0]=APDS9960_WTIME;	
	txr[1]=DEFAULT_WTIME;

	if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,2,true))
		{
			return false;
		}	
		
	
	txr[0]=APDS9960_PPULSE;	
	txr[1]=DEFAULT_PROX_PPULSE;

		if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,2,true))
		{
			return false;
		}
	
	txr[0]=APDS9960_POFFSET_UR;	
	txr[1]=DEFAULT_POFFSET_UR;

	if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,2,true))
		{
			return false;
		}
	txr[0]=APDS9960_POFFSET_DL;	
	txr[1]=DEFAULT_POFFSET_DL;

		if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,2,true))
		{
			return false;
		}
	
	txr[0]=APDS9960_CONFIG1;	
	txr[1]=DEFAULT_CONFIG1;
//	
		if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,2,true))
		{
			return false;
		}
	
	txr[0]=APDS9960_POFFSET_UR;	
	txr[1]=DEFAULT_POFFSET_UR;

		if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,2,true))
		{
			return false;
		}
	
	txr[0]=APDS9960_CONTROL;	// LED CURRENT 100ma, AGAIN 4, PGAIN 4
	txr[1]=0x00;

		if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,2,true))
		{
			return false;
		}
	
	txr[0]=APDS9960_PILT;	
	txr[1]=DEFAULT_PILT;
	if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,2,true))
		{
			return false;
		}
	txr[0]=APDS9960_PIHT;	
	txr[1]=DEFAULT_PIHT;
	if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,2,true))
		{
			return false;
		}
	
	txr[0]=APDS9960_AILTL;	
	txr[1]=0xff;
	txr[2]=0xff;
	txr[3]=0x0;
	txr[4]=0x0;
	if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,4,true))
		{
			return false;
		}
	
	
	
	txr[0]=APDS9960_PERS;	
	txr[1]=DEFAULT_PERS;
	if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,2,true))
		{
			return false;
		}
	
	txr[0]=APDS9960_CONFIG2;	
	txr[1]=0x01;
	if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,2,true))
		{
			return false;
		}
	
	txr[0]=APDS9960_CONFIG3;	
	txr[1]=0;
	if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,2,true))
		{
			return false;
		}
	
	
	
	txr[0]=APDS9960_GPENTH;	
	txr[1]=DEFAULT_GPENTH;
	if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,2,true))
		{
			return false;
		}
	txr[0]=APDS9960_GEXTH;	
	txr[1]=DEFAULT_GEXTH+20;
	if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,2,true))
		{
			return false;
		}
	txr[0]=APDS9960_GCONF1;	
	txr[1]=DEFAULT_GCONF1;
	if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,2,true))
		{
			return false;
		}
	txr[0]=APDS9960_GCONF2;	
	txr[1]=0x01;
	if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,2,true))
		{
			return false;
		}
	
	
	
	txr[0]=APDS9960_GOFFSET_U;	
	txr[1]=DEFAULT_GOFFSET;
	if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,2,true))
		{
			return false;
		}
	txr[0]=APDS9960_GOFFSET_D;	
	txr[1]=DEFAULT_GOFFSET;
	if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,2,true))
		{
			return false;
		}
	txr[0]=APDS9960_GOFFSET_L;	
	txr[1]=DEFAULT_GOFFSET;
	if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,2,true))
		{
			return false;
		}
	txr[0]=APDS9960_GOFFSET_R;	
	txr[1]=DEFAULT_GOFFSET;
	if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,2,true))
		{
			return false;
		}
	txr[0]=APDS9960_GPULSE;	
	txr[1]=DEFAULT_GPULSE;
	if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,2,true))
		{
			return false;
		}
	
	txr[0]=APDS9960_GCONF3;	
	txr[1]=DEFAULT_GCONF3;
	if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,2,true))
		{
			return false;
		}
	
	txr[0]=APDS9960_GCONF4;	
	txr[1]=0x03;
	if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,2,true))
		{
			return false;
		}
	
	return true;
}

bool APDS9960_Select_reg(unsigned char reg)
{
	unsigned char txr[2];
	txr[0]=reg;			
	if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,1,false))
	{
			return false;
	}
	return true;
	
}

bool APDS9960_setAmbientLightIntEnable(unsigned char enable)
{

		uint8_t val;
    unsigned char txr[2];
		
    /* Read value from ENABLE register */
	
		 APDS9960_Select_reg(APDS9960_ENABLE);
    if(!twi_master_transfer(APDS9960_I2C_ADDR|TWI_READ_BIT,&val,1,true)) {
        return false;
    }
    
    /* Set bits in register to given value */
    enable &= 0x01;
    enable = enable << 4;
    val &= 0xef;
    val |= enable;
    
		txr[0]=APDS9960_ENABLE;
		txr[1]=val;
    /* Write register value back into ENABLE register */
    if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,2,true)) {
        return false;
    }
    
    return true;



}	

uint8_t APDS9960_getMode(void)
{
    uint8_t enable_value;
    
    /* Read current ENABLE register */
		APDS9960_Select_reg(APDS9960_ENABLE);
    if(!twi_master_transfer(APDS9960_I2C_ADDR|TWI_READ_BIT,&enable_value,1,true)) {
        return ERROR;
    }
    
    return enable_value;
}




bool APDS9960_setMode(uint8_t mode, uint8_t enable)
{
    uint8_t reg_val;
		unsigned char txr[2];
    /* Read current ENABLE register */
    reg_val = APDS9960_getMode();
    if( reg_val == ERROR ) {
        return false;
    }
    
    /* Change bit(s) in ENABLE register */
    enable = enable & 0x01;
    if( (mode >=0) &&( mode <= 6 )) {
        if (enable) {
            reg_val |= (1 << mode);
        } else {
            reg_val &= ~(1 << mode);
        }
    } else if( mode == ALL ) {
        if (enable) {
            reg_val = 0x7F;
        } else {
            reg_val = 0x00;
        }
    }
    txr[0]=APDS9960_ENABLE;
		txr[1]=reg_val;    
    /* Write value back to ENABLE register */
    if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,2,true)) {
        return false;
    }
        
    return true;
}



/**
 * Turn the APDS-9960 on
 *
 * @return True if operation successful. False otherwise.
 */
bool APDS9960_enablePower(void)
{
    if( APDS9960_setMode(POWER, 1)==false ) {
        return false;
    }
    
    return true;
}



/**
 * @brief Starts the light (R/G/B/Ambient) sensor on the APDS-9960
 *
 * @param[in] interrupts true to enable hardware interrupt on high or low light
 * @return True if sensor enabled correctly. False on error.
 */
bool APDS9960_enableLightSensor(bool interrupts)
{
    
    /* Set default gain, interrupts, enable power, and enable sensor */
//    if( !setAmbientLightGain(DEFAULT_AGAIN) ) {
//        return false;
//    }
    if( interrupts ) {
        if( APDS9960_setAmbientLightIntEnable(1)==false ) {
            return false;
        }
    } else {
        if( APDS9960_setAmbientLightIntEnable(0)==false ) {
            return false;
        }
    }
    if( APDS9960_enablePower()==false ){
        return false;
    }
    if( APDS9960_setMode(AMBIENT_LIGHT, 1) ==false) {
        return false;
    }
    
    return true;

}





bool APDS9960_CheckCommunication(void)
{
	unsigned char txr[2],rxr[4];
	txr[0]=APDS9960_ID;	

		txr[0]=APDS9960_ID;	
		if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,1,false))
		{
			return false;
		}
		
		if(	!twi_master_transfer(APDS9960_I2C_ADDR| TWI_READ_BIT ,rxr,1,true))
		{
			return false;		
		}else{
					
		if(rxr[0]==APDS9960_ID_1||rxr[0]==APDS9960_ID_2)
		{
					
			return true;
		}
		else
		{
		return false;
		}

	}

}


bool APDS9960_EnableALSEngine(unsigned char int_en)
{
		unsigned char txr[2],rxr[5],val=0;
	
		txr[0]=APDS9960_ENABLE;
		if(	!twi_master_transfer(APDS9960_I2C_ADDR ,txr,1,false))
		{
			return false;
		}
							
		if(	!twi_master_transfer(APDS9960_I2C_ADDR| TWI_READ_BIT ,rxr,1,true))
			{
						
				return false;			
			}
	 val=int_en<<4;
	 val|=0x03;
	 val|=rxr[0];
	
		txr[0]=APDS9960_ENABLE	;
		txr[1]=0x0f;
		if(	!twi_master_transfer(APDS9960_I2C_ADDR ,txr,2,true))
		{
			return false;
		}	
					
		return true;		
			
	
}



bool APDS9960_readAmbientLight(unsigned short int *val)
{
	
	//int i=100;
	unsigned char txr[2],rxr[10];
	
	
		txr[0]=APDS9960_STATUS;
		if(	!twi_master_transfer(APDS9960_I2C_ADDR ,txr,1,false))
		{
			return false;
		}
							
		if(	!twi_master_transfer(APDS9960_I2C_ADDR| TWI_READ_BIT,rxr,1,true))
		{
			return false;
		}
		if((rxr[0]&0x01)!=0x01)
		{
			//simple_uart_putstring("error\n");;
			return false;
		}
		
						
					
		txr[0]=APDS9960_CDATAL;	
		if(	!twi_master_transfer(APDS9960_I2C_ADDR ,txr,1,false))
		{
			return false;
		}
						
		if(	!twi_master_transfer(APDS9960_I2C_ADDR| TWI_READ_BIT,rxr,8,true))
		{
			return false;
		}
		unsigned int temp=0;
					//char str [20];
					//simple_uart_putstring("clear=");
					temp=rxr[1];
					temp<<=8;
					temp|=rxr[0];
					//sprintf(str,"%d ",temp);
					//simple_uart_putstring(str);
					*val=temp;
					val++;
					//simple_uart_putstring("red=");
					temp=rxr[3];
					temp<<=8;
					temp|=rxr[2];
					*val=temp;
					val++;
					//sprintf(str,"%d ",temp);
					//simple_uart_putstring(str);
					
					//simple_uart_putstring("green=");
					temp=rxr[5];
					temp<<=8;
					temp|=rxr[4];
					*val=temp;
					val++;
					//sprintf(str,"%d ",temp);
					//simple_uart_putstring(str);
					
					//simple_uart_putstring("blue=");
					temp=rxr[7];
					temp<<=8;
					temp|=rxr[6];
					*val=temp;
					val++;
					//sprintf(str,"%d\n",temp);
					//simple_uart_putstring(str);
					
		return true;
	
	
}
	


bool APDS9960_EnableGestureSensor(void)
{
	unsigned char txr[10];
	txr[0]=APDS9960_WTIME;	
				txr[1]=0xff;
					if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,2,true))
					{
						//simple_uart_putstring("failed");
						return false;
					}
				
				txr[0]=APDS9960_PPULSE;	
				txr[1]=DEFAULT_GESTURE_PPULSE;
					if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,2,true))
					{
						//simple_uart_putstring("failed");
						return false;
					}
					
				txr[0]=APDS9960_CONFIG2;	//led boost
				txr[1]=0x31;;
					if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,2,true))
					{
						//simple_uart_putstring("failed");
						return false;
					}
				
				
					
				
				
					
		
					
				txr[0]=APDS9960_GCONF4;	
				txr[1]=0x03;		
				if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,2,true))
					{
						//simple_uart_putstring("failed");
						return false;
					}	
				
				if(!	APDS9960_enablePower())
				{
					return false;
				}
					
				if(!APDS9960_setMode(WAIT,1))
				{
					return false;
				}
				if(!	APDS9960_setMode(PROXIMITY,1))
				{
					return false;
				}
				if(!APDS9960_setMode(GESTURE,1))
				{
					return false;
				}
				
				return true;
	

					
}


bool APDS9960_GestureAvailable(void)
	
{
	unsigned char rxr[2];
	
			
		APDS9960_Select_reg(APDS9960_GSTATUS);
		if(!twi_master_transfer(APDS9960_I2C_ADDR|TWI_READ_BIT,(uint8_t*)rxr,1,true))
		{
			return false;	
				
		}else
		{
				
				if((rxr[0]&0x01)==0x01)
					{
						return true;
						
						
					}
					
		}
	
	return false;
	
	
	
}



bool APDS9960_ReadFIFO(unsigned char* rval,unsigned char* lval,unsigned char* uval,unsigned char* dval,unsigned char *len)
{
	
	unsigned char rxr[128];
	unsigned char level;
	nrf_delay_ms(FIFO_PAUSE_TIME*2);
	
	if(APDS9960_GestureAvailable())
		{	
			
			APDS9960_Select_reg(APDS9960_GFLVL);
			if(!twi_master_transfer(APDS9960_I2C_ADDR|TWI_READ_BIT,(uint8_t*)rxr,1,true))
			{
				return false;
			}
			*len=rxr[0];
			level=rxr[0];
			//simple_uart_put(level);
			
				if(*len>0)
				{
					
					APDS9960_Select_reg(APDS9960_GFIFO_U);
					if(!twi_master_transfer(APDS9960_I2C_ADDR|TWI_READ_BIT,(uint8_t*)rxr,(level*4),true))
					
					{
						return false;
					}
					nrf_delay_ms(FIFO_PAUSE_TIME);
					for(unsigned char i=0;i<(level*4);i+=4)
					{
						//simple_uart_put(rxr[i]);	//copy fifo datas
						
						*uval=rxr[i];
						*dval=rxr[i+1];
						*lval=rxr[i+2];
						*rval=rxr[i+3];
						uval++;
						dval++;
						lval++;
						rval++;
						
					}
					
					return true;
					
				
			
			
			
		}
		else
		{
			
			return false;
		}
		
	}
	return false;
	
	
	
}



bool APDS9960_ClearInterrupt(void)
{

	if(!APDS9960_Select_reg(APDS9960_CICLEAR))
	{
		return false;
	}
	if(!APDS9960_Select_reg(APDS9960_AICLEAR))
	{
		return false;
	}
return true;


}	
	
void resetGestureParameters(void )
{
   // gesture_data_.index = 0;
   //gesture_data_.total_gestures = 0;
    
    gesture_ud_delta_ = 0;
    gesture_lr_delta_ = 0;
    
    gesture_ud_count_ = 0;
    gesture_lr_count_ = 0;
    
    gesture_near_count_ = 0;
    gesture_far_count_ = 0;
    
    gesture_state_ = 0;
    gesture_motion_ = DIR_NONE;
}

bool APDS9960_ProcessGestureData(unsigned char* gesture)
{
		uint8_t u_first = 0;
    uint8_t d_first = 0;
    uint8_t l_first = 0;
    uint8_t r_first = 0;
    uint8_t u_last = 0;
    uint8_t d_last = 0;
    uint8_t l_last = 0;
    uint8_t r_last = 0;
	
	
		int ud_ratio_first;
    int lr_ratio_first;
    int ud_ratio_last;
    int lr_ratio_last;
    int ud_delta;
    int lr_delta;
    int i;
	
	
	*gesture=DIR_NONE;
	
	if(length<=4)
	{
		
		return false;
	}
	
	
	if( (length <= 32) && (length > 0) ) 
		{
        
        /* Find the first value in U/D/L/R above the threshold */
        for( i = 0; i < length; i++ ) {
            if( (GestureU[i] > GESTURE_THRESHOLD_OUT) &&
                (GestureD[i] > GESTURE_THRESHOLD_OUT) &&
                (GestureL[i] > GESTURE_THRESHOLD_OUT) &&
                (GestureR[i] > GESTURE_THRESHOLD_OUT) ) {
                
                u_first = GestureU[i];
                d_first = GestureD[i];
                l_first = GestureL[i];
                r_first = GestureR[i];
                break;
            }
        }
	
	 }
		
	  if( (u_first == 0) || (d_first == 0) || \
            (l_first == 0) || (r_first == 0) ) {
            
            return false;
        }
	 
	 
			        /* Find the last value in U/D/L/R above the threshold */
        for( i = length - 1; i >= 0; i-- ) {

         

            if( (GestureU[i] > GESTURE_THRESHOLD_OUT) &&
                (GestureD[i] > GESTURE_THRESHOLD_OUT) &&
                (GestureL[i] > GESTURE_THRESHOLD_OUT) &&
                (GestureR[i] > GESTURE_THRESHOLD_OUT) ) {
                
                u_last = GestureU[i];
                d_last = GestureD[i];
                l_last = GestureL[i];
                r_last = GestureR[i];
                break;
            }
        }	
				
				
				//simple_uart_putstring("ok");
				
				
				//simple_uart_put(u_last);
				//simple_uart_put(d_last);
				//simple_uart_put(l_last);
				//simple_uart_put(r_last);
				
//				
//				if(u_last<d_last&&u_last<l_last&&u_last<r_last)
//				{
//					simple_uart_putstring("up\n");
//				}else if(d_last<u_last&&d_last<l_last&&d_last<r_last)
//				{
//					simple_uart_putstring("down\n");
//				}
//				else if(l_last<u_last&&l_last<d_last&&l_last<r_last)
//				{
//					simple_uart_putstring("left\n");
//				}
//				
//				else if(r_last<u_last&&r_last<l_last&&r_last<d_last)
//				{
//					
//					simple_uart_putstring("right\n");
//				}
				
				
				 /* Calculate the first vs. last ratio of up/down and left/right */
    ud_ratio_first = ((u_first - d_first) * 100) / (u_first + d_first);
    lr_ratio_first = ((l_first - r_first) * 100) / (l_first + r_first);
    ud_ratio_last = ((u_last - d_last) * 100) / (u_last + d_last);
    lr_ratio_last = ((l_last - r_last) * 100) / (l_last + r_last);
		
		
		
		
		 /* Determine the difference between the first and last ratios */
    ud_delta = ud_ratio_last - ud_ratio_first;
    lr_delta = lr_ratio_last - lr_ratio_first;
		
		
		/* Accumulate the UD and LR delta values */
    gesture_ud_delta_ += ud_delta;
    gesture_lr_delta_ += lr_delta;
		
		
		
		
		if( gesture_ud_delta_ >= GESTURE_SENSITIVITY_1 ) {
        gesture_ud_count_ = 1;
    } else if( gesture_ud_delta_ <= -GESTURE_SENSITIVITY_1 ) {
        gesture_ud_count_ = -1;
    } else {
        gesture_ud_count_ = 0;
    }
    
    /* Determine L/R gesture */
    if( gesture_lr_delta_ >= GESTURE_SENSITIVITY_1 ) {
        gesture_lr_count_ = 1;
    } else if( gesture_lr_delta_ <= -GESTURE_SENSITIVITY_1 ) {
        gesture_lr_count_ = -1;
    } else {
        gesture_lr_count_ = 0;
    }
    
    /* Determine Near/Far gesture */
    if( (gesture_ud_count_ == 0) && (gesture_lr_count_ == 0) ) {
        if( (abs(ud_delta) < GESTURE_SENSITIVITY_2) && \
            (abs(lr_delta) < GESTURE_SENSITIVITY_2) ) {
            
            if( (ud_delta == 0) && (lr_delta == 0) ) {
                gesture_near_count_++;
            } else if( (ud_delta != 0) || (lr_delta != 0) ) {
                gesture_far_count_++;
            }
            
            if( (gesture_near_count_ >= 10) && (gesture_far_count_ >= 2) ) {
                if( (ud_delta == 0) && (lr_delta == 0) ) {
                    gesture_state_ = NEAR_STATE;
                } else if( (ud_delta != 0) && (lr_delta != 0) ) {
                    gesture_state_ = FAR_STATE;
                }
                return true;
            }
        }
    } else {
        if( (abs(ud_delta) < GESTURE_SENSITIVITY_2) && \
            (abs(lr_delta) < GESTURE_SENSITIVITY_2) ) {
                
            if( (ud_delta == 0) && (lr_delta == 0) ) {
                gesture_near_count_++;
            }
            
            if( gesture_near_count_ >= 10 ) {
                gesture_ud_count_ = 0;
                gesture_lr_count_ = 0;
                gesture_ud_delta_ = 0;
                gesture_lr_delta_ = 0;
            }
        }
    }
		
		
		
    /* Determine U/D gesture */
    if( gesture_ud_delta_ >= GESTURE_SENSITIVITY_1 ) {
        gesture_ud_count_ = 1;
    } else if( gesture_ud_delta_ <= -GESTURE_SENSITIVITY_1 ) {
        gesture_ud_count_ = -1;
    } else {
        gesture_ud_count_ = 0;
    }
    
    /* Determine L/R gesture */
    if( gesture_lr_delta_ >= GESTURE_SENSITIVITY_1 ) {
        gesture_lr_count_ = 1;
    } else if( gesture_lr_delta_ <= -GESTURE_SENSITIVITY_1 ) {
        gesture_lr_count_ = -1;
    } else {
        gesture_lr_count_ = 0;
    }
    
		
		char buf[20];
		sprintf(buf,"ud=%d ud_delta=%d\n",gesture_ud_count_,gesture_ud_delta_);
		//simple_uart_putstring(buf);
		
		sprintf(buf,"lr=%d lr_delta=%d\n",gesture_lr_count_,gesture_lr_delta_);
		//simple_uart_putstring(buf);
		
		
		gesture_motion_=DIR_NONE;
		
		if(gesture_ud_count_==0&&gesture_lr_count_==1)
		{
			
			gesture_motion_=DIR_RIGHT;
			
		}
		
		
		else if(gesture_ud_count_==1&&gesture_lr_count_==-1)
		{
			
			gesture_motion_=DIR_LEFT;
			
		}
		
		else if(gesture_ud_count_==-1&&gesture_lr_count_==0)
		{
			
				gesture_motion_=DIR_UP;
			
			
		}
		
		else if(gesture_ud_count_==1&&gesture_lr_count_==0)
		{
			
				gesture_motion_=DIR_DOWN;
			
			
		}
		
		
				else if(gesture_ud_count_==-1&&gesture_lr_count_==-1)
		{
			
				gesture_motion_=DIR_UP;
			
			
		}
		
		
			else if(gesture_ud_count_==1&&gesture_lr_count_==1)
		{
				if(gesture_ud_delta_>gesture_lr_delta_&&((gesture_ud_delta_-gesture_lr_delta_)>25))
				{
			
					gesture_motion_=DIR_DOWN;
				}else if(gesture_ud_delta_<gesture_lr_delta_&&((gesture_lr_delta_-gesture_ud_delta_)>25))
				{
					gesture_motion_=DIR_RIGHT;
				}
			
		}
		
			else if(gesture_ud_count_==0&&gesture_lr_count_==-1)
		{
				if(gesture_ud_delta_>gesture_lr_delta_&&((gesture_ud_delta_-gesture_lr_delta_)>40))
				{
			
					gesture_motion_=DIR_LEFT;
				}
			
		}
		
		
		
		
				else if(gesture_ud_count_==0&&gesture_lr_count_==0)
		{
			
				return true;
			
			
		}
		
		
		
		switch(gesture_motion_)
		{
			case DIR_LEFT:
				
			//	simple_uart_putstring("LEFT\n");
				//nrf_gpio_pin_set(A1);
				//nrf_gpio_pin_clear(A3);
					*gesture=DIR_LEFT;
			break;
			
			case DIR_RIGHT:
				//nrf_gpio_pin_set(A3);
				//nrf_gpio_pin_clear(A1);
					*gesture=DIR_RIGHT;
			//	simple_uart_putstring("RIGHT\n");
			break;
			
			case DIR_UP:
				//nrf_gpio_pin_set(A1);
				//nrf_gpio_pin_set(A3);
				*gesture=DIR_UP;
				//simple_uart_putstring("UP\n");
			break;
			
			case DIR_DOWN:
				//nrf_gpio_pin_clear(A1);
				//nrf_gpio_pin_clear(A3);
				*gesture=DIR_DOWN;
			//	simple_uart_putstring("DOWN\n");
			break;
			
			default:
				//char buf[20];
		//sprintf(buf,"ud=%d ud_delta=%d\n",gesture_ud_count_,gesture_ud_delta_);
		//simple_uart_putstring(buf);
		
		//sprintf(buf,"lr=%d lr_delta=%d\n",gesture_lr_count_,gesture_lr_delta_);
		//simple_uart_putstring(buf);
			
			//	simple_uart_putstring("Try again!\n");
			*gesture=DIR_NONE;
		}
		
		
		
		resetGestureParameters();
		
		
	 return true;
	 
	 
 } 

 
 
 
 bool APDS9960_ReadProximity(unsigned char* val)
 {
	 unsigned char rxr[2];
	 bool ret=false;
	 APDS9960_Select_reg(APDS9960_STATUS);

			if(!twi_master_transfer(APDS9960_I2C_ADDR|TWI_READ_BIT,(uint8_t*)rxr,1,true))
			
			{
				//simple_uart_putstring("failed");		
				return false;
				
			}else{
				//simple_uart_put(rxr[0]);	
				if((rxr[0]&0x02)!=0x00)		// checking availability of valid proximity value 
				{
					APDS9960_Select_reg(APDS9960_PDATA);
					if(!twi_master_transfer(APDS9960_I2C_ADDR|TWI_READ_BIT,(uint8_t*)rxr,1,true))
					
					{
						ret=false;		
						
						
					}else{
						
						*val=rxr[0];
						ret=true;
						
					}
				}
			}	
	 
	 
	 return ret;
	 
 }
 
 
 
 
 bool APDS9960_ExitGeture(void)
 {
	  unsigned char txr[2],rxr[2];
	 txr[0]=APDS9960_GCONF4;					// exit from the gesture engine
	 txr[1]=0x00;//
		
		if(!twi_master_transfer(APDS9960_I2C_ADDR,(uint8_t*)txr,2,true))
	 
					{
						//simple_uart_putstring("failed");
						return false;
					}	
			
			//nrf_delay_ms(100);
		
				
		APDS9960_Select_reg(APDS9960_STATUS);
		
		nrf_delay_ms(10);
		if(!twi_master_transfer(APDS9960_I2C_ADDR|TWI_READ_BIT,(uint8_t*)rxr,1,true))			
			
			{
				//simple_uart_putstring("failed4");		
				return false;
				
			}else{
					
				if((rxr[0]&0xc0)!=0x0)
				{		
					APDS9960_ClearInterrupt();
				}
			}
	 
	 
	 return true;
 }
 
 
 
 bool APDS9960_Enable(void)
	 
 {
	 if(APDS9960_CheckCommunication())
	 {
			
		
			if(!APDS9960_Initial())
			{
				return false;
			}
			if(!APDS9960_setMode(POWER,1))
			{
				return false;
			}
			if(!APDS9960_setMode(PROXIMITY,1))
			{
				return false;
			}
			if(!APDS9960_setMode(GESTURE,1))
			{
				return false;
			}
			if(!APDS9960_setMode(AMBIENT_LIGHT,1))
			{
				return false;
			}
		}else{
			return false;
		}
	 
	 return true;
 }
 
 
 
 bool APDS9960_ReadGesture(unsigned char* gest )
	 
 {
	 bool ret =false;
	 if(APDS9960_ReadFIFO(GestureR,GestureL,GestureU,GestureD,&length))
	 {
				
				if(APDS9960_ProcessGestureData(gest))
				{
					ret =true;
				}
			
		
		
		
		
	 }
	 APDS9960_ExitGeture();// should be called to exit gesture engine
	 return ret;
 }
 
 
 
 
 
 
 

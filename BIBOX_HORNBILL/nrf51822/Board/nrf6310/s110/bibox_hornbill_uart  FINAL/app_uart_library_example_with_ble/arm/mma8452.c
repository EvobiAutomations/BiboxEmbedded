#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include <stdio.h>
#include "twi.h"
#include "mma8452.h"



char MMA8452_Check_Communication(void)
{
	
	unsigned char rxr;			
	unsigned char txr;	
	txr=WHO_AM_I;	
	TWI_Send_Data(0x38,&txr,1);
	TWI_Receive_Data(0x38,&rxr,1);
	return rxr;


}



void MMA8452_Standby_Mode(void)
{
	
				
	unsigned char txr[2];	
	txr[0]=CTRL_REG1;	
	txr[1]=0x00;
	TWI_Send_Data(0x38,&txr[0],2);
	
}
	

void MMA8452_Set_range(char data)
{
	
		
	unsigned char txr[2];	
	txr[0]=XYZ_DATA_CFG;	
	txr[1]=data;
	TWI_Send_Data(0x38,&txr[0],2);
}



void MMA8452_Active_Mode(void)
{			
	unsigned char txr[2];	
	txr[0]=CTRL_REG1;	
	txr[1]=0x01;
	TWI_Send_Data(0x38,&txr[0],2);
}

	
	
void MMA8452_Initial(void)
{

	
	MMA8452_Set_range(0x00);	
			
	unsigned char txr[5];	
	txr[0]=CTRL_REG1;	
	txr[1]=0x00;
	txr[2]=0x04;
	txr[3]=0x78;
	txr[4]=0xBD;	
	TWI_Send_Data(0x38,&txr[0],5);
	


}


bool MMA8452_Read_Raw_Values(signed int *buf)
{
		unsigned char txr,rxr[6];
		txr=OUT_X_MSB;
	if(!	TWI_Send_Data(0x38,&txr,1))
	{
		return false;
	}
	
	if(	TWI_Receive_Data(0x38,&rxr[0],6))
	{
		buf[0]=rxr[0];
		buf[0]<<=4;
		buf[0]|=(rxr[1]>>4);
	
		if((buf[0]>>11)){
				buf[0]|=0xfffff000;
			}	
			
		buf[1]=rxr[2];
		buf[1]<<=4;
		buf[1]|=(rxr[3]>>4);
		
			if((buf[1]>>11)){
				buf[1]|=0xfffff000;
			}	
	
		buf[2]=rxr[4];
		buf[2]<<=4;
		buf[2]|=(rxr[5]>>4);
		
			if((buf[2]>>11)){
				buf[2]|=0xfffff000;
			}	
			return true;
		}	
		return false;
}

bool MMA8452_Read_X(signed int* val)
{
		unsigned char txr,rxr[2];
		txr=OUT_X_MSB;
		if(!	TWI_Send_Data(0x38,&txr,1))
		{
			return false;
		}
		if(TWI_Receive_Data(0x38,&rxr[0],2))
		{
		
		*val=rxr[0];
		*val<<=4;
		*val|=(rxr[1]>>4);
		if((*val>>11)){
				*val|=0xfffff000;
			}	
			return true;
			
		}else{
			return false;
		}
			
}


bool MMA8452_Read_Y(signed int* val)
{
		unsigned char txr,rxr[2];
		txr=OUT_Y_MSB;
		if(!	TWI_Send_Data(0x38,&txr,1))
		{
			return false;
		}
		if(TWI_Receive_Data(0x38,&rxr[0],2))
		{
		
		*val=rxr[0];
		*val<<=4;
		*val|=(rxr[1]>>4);
		if((*val>>11)){
				*val|=0xfffff000;
			}	
			
			
			return true;
			
		}else{
			return false;
		}
			
}

bool MMA8452_Read_Z(signed int* val)
{
		unsigned char txr,rxr[2];
		txr=OUT_Z_MSB;
		if(!	TWI_Send_Data(0x38,&txr,1))
		{
			return false;
		}
		if(TWI_Receive_Data(0x38,&rxr[0],2))
		{
		
		*val=rxr[0];
		*val<<=4;
		*val|=(rxr[1]>>4);						
		
			if((*val>>11)){
				*val|=0xfffff000;
			}				
			
			return true;
			
		}else{
			return false;
		}
			
}


#include <stdio.h> 
#include "nrf.h"
#include "nrf_delay.h"
#include "twi.h"
#include <math.h>
#include "hmc5883.h"



struct Vector V;
int xOffset, yOffset;
float mgPerDigit = 0.92f;




bool Hmc5883_Initial(void)
{
	unsigned char txr[5],rxr[5];
	
		txr[0]=HMC5883L_REG_IDENT_A;
		if(!TWI_Send_Data(HMC5883L_ADDRESS,txr,1))
		{
			return false;
		}
		if(!TWI_Receive_Data(HMC5883L_ADDRESS,rxr,3))
		{
			return false;
		}
		else{
			nrf_delay_ms(10);
			if(rxr[0]==0x48&&rxr[1]==0x34&&rxr[2]==0x33)
			{
				
				txr[0]=HMC5883L_REG_CONFIG_A;
				txr[1]=0x10;//00010000;
				txr[2]=0x20;//00100000;
				txr[3]=0x00;//00000000;
				if(!TWI_Send_Data(HMC5883L_ADDRESS,txr,4))
				{
					return false;
				}
	
	
	
				return true;
			}
			else{
					return false;
			}
			
			
			
		}
	  
	
	
	
	
}



bool Hmc5883_readRawValues(signed int* X,signed int* Y,signed int* Z)
{
	
	unsigned char txr[5],rxr[10];
	
		txr[0]=HMC5883L_REG_OUT_X_M;
		if(!TWI_Send_Data(HMC5883L_ADDRESS,txr,1))
		{
			return false;
		}
		if(!TWI_Receive_Data(HMC5883L_ADDRESS,rxr,6))
		{
			return false;
		}
		
			*X=0;
			*Y=0;
			*Z=0;
			
			*X=rxr[0];
			*X<<=8;
			*X|=rxr[1];				
			if((*X>>15)){
				*X|=0xffff0000;
			}	
			
						
			*Z=rxr[2];
			*Z<<=8;
			*Z|=rxr[3];
			if((*Z>>15)){
				*Z|=0xffff0000;
			}	
			
			
			*Y=rxr[4];
			*Y<<=8;
			*Y|=rxr[5];
			
			if((*Y>>15)){
				*Y|=0xffff0000;
			}	
			
	return true;
	
	
	
	
}


bool Hmc5883_readNormalize(void)
{
	signed int x,y,z;
	if(!Hmc5883_readRawValues(&x,&y,&z))
	{
		return false;
	}
		V.XAxis = ((float)x - xOffset) * mgPerDigit;
    V.YAxis = ((float)y - yOffset) * mgPerDigit;
    V.ZAxis = (float) z * mgPerDigit;
 return true;
	
	
	
	
}


bool Hmc5883_readDegree(float* deg)
{
	
	if(Hmc5883_readNormalize())
	{
				
		float heading = atan2(V.YAxis, V.XAxis);
		float declinationAngle = (0.0 - (52.0 / 60.0)) / (180 / M_PI);
		heading += declinationAngle;

				// Correct for heading < 0deg and heading > 360deg
		if (heading < 0)
		{
				heading += 2 * PI;
		}

		if (heading > 2 * PI)
		{
				heading -= 2 * PI;
		}

				// Convert to degrees
		float headingDegrees = heading * 180.0/M_PI; 
		*deg=headingDegrees;
		return true;

				// Output
						
	}
	
	return false;
	
	
}





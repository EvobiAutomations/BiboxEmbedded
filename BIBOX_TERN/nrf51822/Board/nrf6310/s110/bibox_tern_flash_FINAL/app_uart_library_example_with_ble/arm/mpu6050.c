
#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "simple_uart.h"
#include <stdio.h>
#include "twi.h"
#include "mpu6050.h"


unsigned char txr[10],rxr[10];



bool MPU6050_TestConnection(void)
{
	
	txr[0]=MPU6050_RA_WHO_AM_I;
	TWI_Send_Data(MPU6050_ADDRESS_AD0_LOW,&txr[0],1);
	
	TWI_Receive_Data(MPU6050_ADDRESS_AD0_LOW,rxr,1);
	if(rxr[0]==0x68)
	{
		return true;
	}
	else{
		
		return false;
	}
	
	
	
	
}

bool MPU6050_Initialize(void)
{
	//set clock source
	
	txr[0]=MPU6050_RA_PWR_MGMT_1;
	txr[1]=0x05;	//clock source is Xgyro, sleep disabled, temp disabled,cycle disabled,reset disabled
	if(!TWI_Send_Data(MPU6050_ADDRESS_AD0_LOW,txr,2))
	{
		return false;
	}
	
	txr[0]=MPU6050_RA_GYRO_CONFIG;
	txr[1]=0x00;	//Gyro full scale range =250
	if(!TWI_Send_Data(MPU6050_ADDRESS_AD0_LOW,txr,2))
	{
		return false;
	}
	
	txr[0]=MPU6050_RA_ACCEL_CONFIG;
	txr[1]=0x00;	//Accelero full scale range =2g
	if(!TWI_Send_Data(MPU6050_ADDRESS_AD0_LOW,txr,2))
	{
		return false;
	}
	
	return true;
	
}
	

bool MPU6050_ReadGyro(signed short int* gyro_x,signed short int* gyro_y,signed short int* gyro_z)
{
	txr[0]=MPU6050_RA_INT_STATUS;
	if(!TWI_Send_Data(MPU6050_ADDRESS_AD0_LOW,txr,1))
	{
		return false;
	}
	
	
	if(!TWI_Receive_Data(MPU6050_ADDRESS_AD0_LOW,rxr,1))
	{
		return false;
	}
	if((rxr[0]&0x01)!=0x01)
	{
		return false;
	}
	
	
	txr[0]=MPU6050_RA_GYRO_XOUT_H;
	if(!TWI_Send_Data(MPU6050_ADDRESS_AD0_LOW,txr,1))
	{
		return false;
	}
	
	if(!TWI_Receive_Data(MPU6050_ADDRESS_AD0_LOW,rxr,6))
	{
		return false;
	}
	
	*gyro_x=rxr[0];
	*gyro_x<<=8;
	*gyro_x|=rxr[1];	
	
	*gyro_y=rxr[2];
	*gyro_y<<=8;
	*gyro_y|=rxr[3];
	
	*gyro_z=rxr[4];
	*gyro_z<<=8;
	*gyro_z|=rxr[5];
	
	return true;
}


#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include <stdio.h>
#include "twi.h"




void TWI_Initial(void)
{

	nrf_gpio_cfg_input(G1,NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(G2,NRF_GPIO_PIN_NOPULL);
	
	NRF_TWI1->PSELSCL=TWI_SCL;
	NRF_TWI1->PSELSDA=TWI_SDA;
	NRF_TWI1->FREQUENCY=0x06680000;
	NRF_TWI1->ENABLE=5;
	//NRF_TWI1->INTENSET=0x1f;
	NRF_TWI1->POWER=1;
  NRF_TWI1->INTENCLR=0xffffffff;
	
	
	
}

void TWI_Initial1(void)
{

	nrf_gpio_cfg_input(TWI_SCL,NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_input(TWI_SDA,NRF_GPIO_PIN_NOPULL);
	
	NRF_TWI1->PSELSCL=TWI_SDA;
	NRF_TWI1->PSELSDA=TWI_SCL;
	NRF_TWI1->FREQUENCY=0x06680000;
	NRF_TWI1->ENABLE=5;
	//NRF_TWI1->INTENSET=0x1f;
	NRF_TWI1->POWER=1;
	NRF_TWI1->INTENCLR=0xffffffff;
	

	
	
	
}






	bool TWI_Send_Data(unsigned char addr,unsigned char *buf,unsigned char len)
	{
		unsigned char *data;
		data=buf;
		
		NRF_TWI1->ADDRESS=addr>>1;
		
		NRF_TWI1->EVENTS_TXDSENT=0;
		NRF_TWI1->EVENTS_STOPPED=0;
		NRF_TWI1->EVENTS_ERROR=0;
		
		NRF_TWI1->TASKS_STARTTX=1;
		
		while(len>0)
		{
		
			NRF_TWI1->TXD=*data;
			int i=100;
			while(NRF_TWI1->EVENTS_TXDSENT!=1&&NRF_TWI1->EVENTS_ERROR!=1&&i!=0){i--;};
			NRF_TWI1->EVENTS_TXDSENT=0;
			data++;
			len--;
			if(NRF_TWI1->EVENTS_ERROR)
			{
				return false;
			}
			
			
		}

			
		
		

		return true;
		
		
		
		
	}
		


	void TWI_Stop(void)
	{
		NRF_TWI1->TASKS_STOP=1;
		int i=100;
		while(NRF_TWI1->EVENTS_STOPPED!=1&&i!=0){i--;};
		NRF_TWI1->EVENTS_STOPPED=0;
	}





		
		

	bool TWI_Receive_Data(unsigned char addr,unsigned char *buf,unsigned len)
	{
				
			NRF_TWI1->ADDRESS=addr>>1;
		
			NRF_TWI1->EVENTS_RXDREADY=0;
			NRF_TWI1->EVENTS_STOPPED=0;
			NRF_TWI1->EVENTS_ERROR=0;
			NRF_TWI1->TASKS_STARTRX=1;

		while(len>0)
		{
			*buf=NRF_TWI1->RXD;
			int i=100;
			while(NRF_TWI1->EVENTS_RXDREADY!=1&&NRF_TWI1->EVENTS_ERROR!=1&&i!=0){i--;};
			NRF_TWI1->EVENTS_RXDREADY=0;
				if(NRF_TWI1->EVENTS_ERROR)
				{
					NRF_TWI1->EVENTS_ERROR=0;
					
					
					return false;
				}
			
			
			
			*buf=NRF_TWI1->RXD;
			buf++;
			len--;
			
		}
		int i=100;
		while(NRF_TWI1->EVENTS_RXDREADY!=1&&NRF_TWI1->EVENTS_ERROR!=1&&i!=0){i--;};
			NRF_TWI1->EVENTS_RXDREADY=0;
		
			
			
			
			
			
			NRF_TWI1->TASKS_STOP=1;
			 i=100;
			while(NRF_TWI1->EVENTS_STOPPED!=1&&i!=0){i--;};
			NRF_TWI1->EVENTS_STOPPED=0;
			*buf=NRF_TWI1->RXD;
			if(NRF_TWI1->EVENTS_ERROR)
				{
					NRF_TWI1->EVENTS_ERROR=0;
					
					
					return false;
				}
			
			
			
			
			
			
		return true;


			}








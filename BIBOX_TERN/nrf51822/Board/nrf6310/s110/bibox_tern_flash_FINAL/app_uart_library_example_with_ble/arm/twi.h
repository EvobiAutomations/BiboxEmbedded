#ifndef TWI
#define TWI

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include <stdio.h>



#define TWI_SCL G1
#define TWI_SDA G2

/****************************************************************
Pleae note that the address should be passed in 8 bit format, the function
will convert 8 bit address into 7 bit format automatically with R/W bit

****************************************************************/



void TWI_Initial(void);

void TWI_Initial1(void);

bool TWI_Send_Data(unsigned char addr,unsigned char *buf,unsigned char len);




void TWI_Stop(void);



bool TWI_Receive_Data(unsigned char addr,unsigned char *buf,unsigned len);



#endif 



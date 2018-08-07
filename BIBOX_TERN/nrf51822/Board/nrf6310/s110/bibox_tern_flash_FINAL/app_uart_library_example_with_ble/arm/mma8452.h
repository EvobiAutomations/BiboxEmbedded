#ifndef MMA8452
#define MMA8452

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include <stdio.h>



enum MMA8452Q_REGISTERS {
	STATUS       = 0x00,

	OUT_X_MSB    = 0x01,
	OUT_X_LSB    = 0x02,

	OUT_Y_MSB    = 0x03,
	OUT_Y_LSB    = 0x04,

	OUT_Z_MSB    = 0x05,
	OUT_Z_LSB    = 0x06,

	SYSMOD       = 0x0B,
	INT_SOURCE   = 0x0C,
	WHO_AM_I     = 0x0D,
	XYZ_DATA_CFG = 0x0E,

	PL_STATUS    = 0x10,
	PL_CFG       = 0x11,
	PL_COUNT     = 0x12,

	CTRL_REG1    = 0x2A,
	CTRL_REG2    = 0x2B,
	CTRL_REG3    = 0x2C,
	CTRL_REG4    = 0x2D,
	CTRL_REG5    = 0x2E,

	OFF_X        = 0x2F,
	OFF_Y        = 0x30,
	OFF_Z        = 0x31
};


/*******************************************************
************** Check communication *********************
@return WHO_AM_I value (should return 0x2a )

*******************************************************/

char MMA8452_Check_Communication(void);


/*******************************************************
************** Put IC into standby mode ****************
*******************************************************/

void MMA8452_Standby_Mode(void);


/*******************************************************
************** Set full scale range ********************
*******************************************************/

void MMA8452_Set_range(char data);



/*******************************************************
************** Put IC into active mode *****************
*******************************************************/

void MMA8452_Active_Mode(void);


/*******************************************************
************** Initialize Accelerometer ****************
*******************************************************/

void MMA8452_Initial(void);


/*******************************************************
************** Read RAW Values *************************
@param in Buffer for returning acclerometer val
*******************************************************/
bool MMA8452_Read_Raw_Values(signed int *buf);



/*******************************************************
************** Read X Axis Value ***********************
@param in Buffer for returning acclerometer val
*******************************************************/
bool MMA8452_Read_X(signed int* val);




/*******************************************************
************** Read Y Axis  Values *********************
@param in Buffer for returning acclerometer val
*******************************************************/
bool MMA8452_Read_Y(signed int* val);



/*******************************************************
************** Read Z axis val *************************
@param in Buffer for returning acclerometer val
*******************************************************/
bool MMA8452_Read_Z(signed int* val);


#endif



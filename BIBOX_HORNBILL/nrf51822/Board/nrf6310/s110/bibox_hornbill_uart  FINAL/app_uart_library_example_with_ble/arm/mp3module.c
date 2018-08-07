

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "simple_uart.h"
#include <stdio.h>
#include "mp3module.h"


uint8_t send_buf[10] = {0x7E, 0xFF, 06, 00, 00, 00, 00, 00, 00, 0xEF};


uint16_t mp3_get_checksum (uint8_t *thebuf) {
	uint16_t sum = 0;
	for (int i=1; i<7; i++) {
		sum += thebuf[i];
	}
	return 0-sum;
}


static void fill_uint16_bigend (uint8_t *thebuf, uint16_t data) {
	*thebuf =	(uint8_t)(data>>8);
	*(thebuf+1) =	(uint8_t)data;
}


void mp3_fill_checksum (void) {
	uint16_t checksum = mp3_get_checksum (send_buf);
	fill_uint16_bigend (send_buf+7, checksum);
}


void h_send_func (void) {
	for (int i=0; i<10; i++) {
		simple_uart_put (send_buf[i]);
	}
}

//
void s_send_func (void) {
	for (int i=0; i<10; i++) {
		simple_uart_put(send_buf[i]);
	}
}



//
void mp3_send_cmd (uint8_t cmd, uint16_t arg) {
	send_buf[3] = cmd;	
	send_buf[6]=arg;
	mp3_fill_checksum ();
	
	for(unsigned int i=0;i<10;i++)
	{
		simple_uart_put(send_buf[i]);
	}
}

void mp3_play (uint16_t num) {
	send_buf[4]=0x00;
	send_buf[5]=0x01;
	mp3_send_cmd (0x0f, num);
	
}



void mp3_source (void) {
	send_buf[4]=0;
	send_buf[5]=0x00;
	send_buf[6]=0x00;
	mp3_send_cmd (0x09, 0);
	
}

void mp3_set_vol (char vol) {
	send_buf[4]=0;
	send_buf[5]=0x00;
	send_buf[6]=vol;
	mp3_send_cmd (0x06, vol);
	
}


void mp3_playback(void)
{
	
	send_buf[4]=0;
	send_buf[5]=0x00;
	send_buf[6]=0x00;
	mp3_send_cmd (0x0d, 0);
	
	
}


void mp3_pause(void)
{
	
	send_buf[4]=0;
	send_buf[5]=0x00;
	send_buf[6]=0x00;
	mp3_send_cmd (0x0e, 0);
	
	
}

void mp3_next(void)
{
	
	send_buf[4]=0;
	send_buf[5]=0x00;
	send_buf[6]=0x00;
	mp3_send_cmd (0x01, 0);
	
	
}

void mp3_previous(void)
{
	
	send_buf[4]=0;
	send_buf[5]=0x00;
	send_buf[6]=0x00;
	mp3_send_cmd (0x02, 0);
	
	
}




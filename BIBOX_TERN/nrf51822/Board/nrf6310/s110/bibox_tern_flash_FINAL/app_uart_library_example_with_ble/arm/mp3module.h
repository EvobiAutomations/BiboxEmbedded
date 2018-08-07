#ifndef MP3_MODULE
#define MP3_MODULE


#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "simple_uart.h"
#include <stdio.h>


uint16_t mp3_get_checksum (uint8_t *thebuf) ;


static void fill_uint16_bigend (uint8_t *thebuf, uint16_t data);
	

void mp3_fill_checksum (void);


void h_send_func (void);


void s_send_func (void);


void mp3_send_cmd (uint8_t cmd, uint16_t arg);


void mp3_play (uint16_t num) ;


void mp3_source (void) ;


void mp3_set_vol (char vol);


void mp3_playback(void);


void mp3_pause(void);


void mp3_next(void);


void mp3_previous(void);





#endif




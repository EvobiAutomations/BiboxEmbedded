
/* =========================================================================== */
/* ===================== READ COLOR SENSOR VALUES ============================ */
/* =========================================================================== */



void read_color_sensor(void)
{
	unsigned char  rx_lsb,rx_msb;

	simple_uart_put('R');
	simple_uart_get_with_timeout(1000,&rx_lsb);
	simple_uart_get_with_timeout(1000,&rx_msb);
	
	simple_uart_put(rx_lsb);
	simple_uart_put(rx_msb);
	
	Red=rx_msb;
	Red<<=8;
	Red|=rx_lsb;
	
	simple_uart_put('G');
	simple_uart_get_with_timeout(1000,&rx_lsb);
	simple_uart_get_with_timeout(1000,&rx_msb);
	
	simple_uart_put(rx_lsb);
	simple_uart_put(rx_msb);
	
	Green=rx_msb;
	Green<<=8;
	Green|=rx_lsb;
	
	
	simple_uart_put('B');
	simple_uart_get_with_timeout(1000,&rx_lsb);
	simple_uart_get_with_timeout(1000,&rx_msb);
	
	simple_uart_put(rx_lsb);
	simple_uart_put(rx_msb);
	
	Blue=rx_msb;
	Blue<<=8;
	Blue|=rx_lsb;
	

}



/* =========================================================================== */
/* ===================== Read RFID card ====================================== */
/* =========================================================================== */

unsigned int Read_Rfid(void)
{
	unsigned char status;
	unsigned int card=0;
	status = MFRC522_Request(PICC_REQIDL, str);
		
		if (status == MI_OK)
    {
      //Serial.println("Card detected");
      //Serial.print(str[0],BIN);
      //Serial.print(" , ");
      //Serial.print(str[1],BIN);
      //Serial.println(" ");
			//app_uart_put(str[0]);
		/*	for(int i=0;i<16;i++)
			{
			app_uart_put(str[i]);
			}
			nrf_delay_ms(1000);
		*/
		}
		
		status = MFRC522_Anticoll(str);
	
    memcpy(serNum, str, 5);
    if (status == MI_OK)
    {
      MFRC522_Halt();                        //Command card into hibernation
      SetFormatRDM630();

      //printf("%s", (uchar_send));

      // Should really check all pairs, but for now we'll just use the first
     card=serNum[1];
		 card<<=8;
		 card|=serNum[0];
			//app_uart_put(serNum[2]);
			//app_uart_put(serNum[3]);
			//app_uart_put(0x0d);
      
      
     /*for (bytevar1=0;bytevar1<250;bytevar1++)
      {
          beep = !beep;
          delay_us(350);
      }*/
      //led = 0;   // turn the LED on (HIGH is the voltage level)
     // nrf_delay_ms(50);
      //led = 1;   // turn the LED on (HIGH is the voltage level)
      
      //beep350();
      // nrf_delay_ms(3000);
    }
    else
    {
      //Serial.println(" ");
      //delay(1000);
      MFRC522_Halt();                        // Command card into hibernation
      nrf_delay_ms(100);                         // wait for low consuption
    }

    // COUNT FOR TIMEOUT TO RESET BLUETOOTH MODULE
    //TimeOut1();
    
    // CHANGE STATE OF OUTPUT
 /*   if (stringComplete) 
    {
      stringComplete = 0;

      if (Output1 == 1)
      {
        pulseLED();

        beep900();
        
        pulseLED();

        beep900();
      }
      else
      {
        pulseLED();

        beep1136();
      }

    }*/
  
	return card;
}
#include "MFRC522.h"
#include "nrf_gpio.h"
#include "soft_spi.h"
#include "simple_uart.h"


//unsigned char  serNum[5];
//unsigned char uchar_send[15];




void Write_MFRC522(unsigned char addr, unsigned char val)
{
  nrf_gpio_pin_clear(SOFT_SPI_CS);
  //Address Format: 0XXXXXX0
  //NRF_SPI1->TXD=((addr<<1)&0x7E);
	// nrf_delay_us(2);
	//NRF_SPI1->TXD=val;
	//  nrf_delay_us(2);
		 Soft_Spi_Write_Read(((addr<<1)&0x7E));
		 Soft_Spi_Write_Read(val);
	
	 nrf_gpio_pin_set(SOFT_SPI_CS);
	
}

void Write_MFRC522_Long(unsigned char addr,unsigned char len, unsigned char *val)
{
  nrf_gpio_pin_clear(SOFT_SPI_CS);
  //Address Format: 0XXXXXX0
  //NRF_SPI1->TXD=((addr<<1)&0x7E);
	// nrf_delay_us(2);
	//NRF_SPI1->TXD=val;
	//  nrf_delay_us(2);
		Soft_Spi_Write_Read(((addr<<1)&0x7E));
		for(unsigned char index=0;index<len;index++)
		{
		 Soft_Spi_Write_Read(val[index]);
		}
	
	 nrf_gpio_pin_set(SOFT_SPI_CS);
	
}




unsigned char Read_MFRC522(unsigned char addr)
{
  unsigned char val;

	/*nrf_gpio_pin_clear(chipSelectPin);
  NRF_SPI1->TXD=((addr<<1) | 0x80);
  nrf_delay_us(1);	
	NRF_SPI1->TXD=((addr<<1) | 0x80);
  nrf_delay_us(1);
		
	val=NRF_SPI1->RXD;
	NRF_SPI1->EVENTS_READY=0;
*/
	 nrf_gpio_pin_clear(SOFT_SPI_CS);
	 Soft_Spi_Write_Read(((addr<<1) | 0x80));
	 val= Soft_Spi_Write_Read(0);
	 nrf_gpio_pin_set(SOFT_SPI_CS);
	return val;
}

void Read_Long_MFRC522(unsigned char reg,unsigned char count,unsigned char* values,unsigned char rxAlign )
{
  

	/*nrf_gpio_pin_clear(chipSelectPin);
  NRF_SPI1->TXD=((addr<<1) | 0x80);
  nrf_delay_us(1);	
	NRF_SPI1->TXD=((addr<<1) | 0x80);
  nrf_delay_us(1);
		
	val=NRF_SPI1->RXD;
	NRF_SPI1->EVENTS_READY=0;
*/
	if (count == 0) {
		return;
	}
	unsigned char address = 0x80 | (reg & 0x7E);		// MSB == 1 is for reading. LSB is not used in address. Datasheet section 8.1.2.3.
	unsigned char index = 0;							// Index in values array.	
	nrf_gpio_pin_clear(SOFT_SPI_CS);
	count--;
	Soft_Spi_Write_Read(address);
	
	while (index < count) {
		if (index == 0 && rxAlign) {		// Only update bit positions rxAlign..7 in values[0]
			// Create bit mask for bit positions rxAlign..7
			unsigned char mask = 0;
			for (unsigned char i = rxAlign; i <= 7; i++) {
				mask |= (1 << i);
			}
			// Read value and tell that we want to read the same address again.
			unsigned char value =Soft_Spi_Write_Read(address);
			// Apply mask to both current value of values[0] and the new data in value.
			values[0] = (values[index] & ~mask) | (value & mask);
		}
		else { // Normal case
			values[index] = Soft_Spi_Write_Read(address);	// Read value and tell that we want to read the same address again.
		}
		index++;
	}
	values[index] = Soft_Spi_Write_Read(address);			// Read the final byte. Send 0 to stop reading.
	nrf_gpio_pin_set(SOFT_SPI_CS);;			// Release slave again
	
}





void SetBitMask(unsigned char reg, unsigned char mask)
{
unsigned char tmp;
  tmp = Read_MFRC522(reg);
  Write_MFRC522(reg, tmp | mask);  // set bit mask
}



/*
 * Function Name: ClearBitMask
 * Description: clear RC522 register bit
 * Input parameters: reg - register address; mask - clear bit value
 * Return value: None
*/
void ClearBitMask(unsigned char reg, unsigned char mask)
{
  unsigned char tmp;
  tmp = Read_MFRC522(reg);
  Write_MFRC522(reg, tmp & (~mask));  // clear bit mask
}






unsigned char MFRC522_ToCard(unsigned char command, unsigned char*sendData, unsigned char sendLen, unsigned char *backData,unsigned int *backLen)
{
  unsigned char status = MI_ERR;
  unsigned char irqEn = 0x00;
	unsigned char waitIRq = 0x00;
  unsigned char lastBits;
  unsigned char n;
  unsigned char i;

  switch (command)
  {
    case PCD_AUTHENT:                //Certification cards close
    {
      irqEn = 0x12;
      waitIRq = 0x10;
      break;
    }
    case PCD_TRANSCEIVE:        //Transmit FIFO data
    {
      irqEn = 0x77;
      waitIRq = 0x30;
      break;
    }
    default:
      break;
  }

  Write_MFRC522(CommIEnReg, irqEn|0x80);        //Interrupt request
  ClearBitMask(CommIrqReg, 0x80);                        //Clear all interrupt request bit
  SetBitMask(FIFOLevelReg, 0x80);                        //FlushBuffer=1, FIFO Initialization

  Write_MFRC522(CommandReg, PCD_IDLE);        //NO action; Cancel the current command???

  //Writing data to the FIFO
  for (i=0; i<sendLen; i++)
  {
    Write_MFRC522(FIFODataReg, sendData[i]);
  }

  //Execute the command
  Write_MFRC522(CommandReg, command);
  if (command == PCD_TRANSCEIVE)
  {
    SetBitMask(BitFramingReg, 0x80);                //StartSend=1,transmission of data starts
  }

  //Waiting to receive data to complete
  i = 100;        //i according to the clock frequency adjustment, the operator M1 card maximum waiting time 25ms???
  do
  {
    //CommIrqReg[7..0]
    //Set1 TxIRq RxIRq IdleIRq HiAlerIRq LoAlertIRq ErrIRq TimerIRq
    n = Read_MFRC522(CommIrqReg);
		
    i--;
  }
  while ((i!=0) && !(n&0x01) && !(n&waitIRq));

  ClearBitMask(BitFramingReg, 0x80);                        //StartSend=0

  if (i != 0)
  {
    if(!(Read_MFRC522(ErrorReg) & 0x1B))        //BufferOvfl Collerr CRCErr ProtecolErr
    {
      status = MI_OK;
      if (n & irqEn & 0x01)
      {
        status = MI_NOTAGERR;                        //??
      }

      if (command == PCD_TRANSCEIVE)
      {
        n = Read_MFRC522(FIFOLevelReg);
        lastBits = Read_MFRC522(ControlReg) & 0x07;
        if (lastBits)
        {
          *backLen = (n-1)*8 + lastBits;
        }
        else
        {
          *backLen = n*8;
        }

        if (n == 0)
        {
          n = 1;
        }
        if (n > MAX_LEN)
        {
          n = MAX_LEN;
        }

        //Reading the received data in FIFO
        for (i=0; i<n; i++)
        {
          backData[i] = Read_MFRC522(FIFODataReg);
					//simple_uart_put(backData[i]);
        }
      }
    }
    else
    {
      status = MI_ERR;
    }

  }

  //SetBitMask(ControlReg,0x80);           //timer stops
  //Write_MFRC522(CommandReg, PCD_IDLE);

  return status;
}




unsigned char MFRC522_Request(unsigned char reqMode, unsigned char *TagType)
{
 unsigned char status;
 unsigned int backBits;                        //The received data bits

  Write_MFRC522(BitFramingReg, 0x07);                //TxLastBists = BitFramingReg[2..0]        ???

  TagType[0] = reqMode;
  status = MFRC522_ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);

  if ((status != MI_OK) || (backBits != 0x10))
  {
    status = MI_ERR;
  }

  return status;
}



void MFRC522_Reset(void)
{
  Write_MFRC522(CommandReg, PCD_RESETPHASE);
}


void AntennaOn(void)
  {
  unsigned char temp;

  temp = Read_MFRC522(TxControlReg);
  if (!(temp & 0x03))
  {
    SetBitMask(TxControlReg, 0x03);
}
}


void MFRC522_Init(void)
{
 

  MFRC522_Reset();

  //Timer: TPrescaler*TreloadVal/6.78MHz = 24ms
  Write_MFRC522(TModeReg, 0x8D);                //Tauto=1; f(Timer) = 6.78MHz/TPreScaler
  Write_MFRC522(TPrescalerReg, 0x3E);        		//TModeReg[3..0] + TPrescalerReg
  Write_MFRC522(TReloadRegL, 30);
  Write_MFRC522(TReloadRegH, 0);

  Write_MFRC522(TxAutoReg, 0x40);               //100%ASK
  Write_MFRC522(ModeReg, 0x3D);                	//CRC Initial value 0x6363        ???

	
  //ClearBitMask(Status2Reg, 0x08);             //MFCrypto1On=0
  //Write_MFRC522(RxSelReg, 0x86);              //RxWait = RxSelReg[5..0]
  //Write_MFRC522(RFCfgReg, 0x7F);              //RxGain = 48dB

  AntennaOn();                //Open the antenna
}


unsigned char MFRC522_Anticoll(unsigned char *serNum)
{
  unsigned char status,command[12];
  unsigned char i;
  unsigned char serNumCheck=0;
  unsigned int backlen;


  //ClearBitMask(Status2Reg, 0x08);                //TempSensclear
  //ClearBitMask(CollReg,0x80);                        //ValuesAfterColl
  Write_MFRC522(BitFramingReg, 0x00);                //TxLastBists = BitFramingReg[2..0]

  command[0]=PICC_ANTICOLL;
  command[1]=0x20;
  backlen=0;
  Write_MFRC522(BitFramingReg, 0x00);
  status=MFRC522_ToCard(PCD_TRANSCEIVE,command, 2,&serNum[0],&backlen);
	//app_uart_put(status);
	//app_uart_put(serNum[0]);
  if (status == MI_OK)
  {
    //Check card serial number
    for (i=0; i<4; i++)
    {
      serNumCheck ^= serNum[i];
    }
    if (serNumCheck != serNum[i])
    {
      status = MI_OK;
    }
  }
return status;

}

unsigned char CalulateCRC(unsigned char *pIndata, unsigned char len, unsigned char *pOutData)
{
  unsigned char i, n;

  ClearBitMask(DivIrqReg, 0x04);                        //CRCIrq = 0
  SetBitMask(FIFOLevelReg, 0x80);                        //Clear the FIFO pointer
  Write_MFRC522(CommandReg, PCD_IDLE);

  //Writing data to the FIFO
  for (i=0; i<len; i++)
  {
    Write_MFRC522(FIFODataReg, *(pIndata+i));
  }
  Write_MFRC522(CommandReg, PCD_CALCCRC);

  //Wait CRC calculation is complete
  i = 0xFF;
  do
  {
    n = Read_MFRC522(DivIrqReg);
    i--;
		if(i==0){return MI_ERR;}
  }
  while ((i!=0) && !(n&0x04));                        //CRCIrq = 1

  //Read CRC calculation result
  pOutData[0] = Read_MFRC522(CRCResultRegL);
  pOutData[1] = Read_MFRC522(CRCResultRegM);
	return MI_OK;
}


void MFRC522_Halt(void)
{
 
  unsigned int unLen;
  unsigned char buff[4];

  buff[0] = PICC_HALT;
  buff[1] = 0;
  CalulateCRC(buff, 2, &buff[2]);

  MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff,&unLen);

}


unsigned char Separate_hexP10(unsigned char val)
{
  val = val & 0xF0;
  val = val >> 4;
  if (val < 10)
  {
      return val + 48;
  }
  else
  {
      return val + 55;
  }
}

unsigned char Separate_hexP1(unsigned char val)
{
  val = val & 0x0F;
  if (val < 10)
  {
      return val + 48;
  }
  else
  {
      return val + 55;
  }
}




unsigned char MFRC522_SelectTag(unsigned char *serNum)
{
  unsigned char i;
  unsigned char status;
  unsigned char size;
  unsigned int recvBits;
  unsigned char buffer[9];

  //ClearBitMask(Status2Reg, 0x08);                   //MFCrypto1On=0

  buffer[0] = PICC_SElECTTAG;
  buffer[1] = 0x70;
  for (i=0; i<5; i++)
  {
    buffer[i+2] = *(serNum+i);
  }
  CalulateCRC(buffer, 7, &buffer[7]);                //??
  status = MFRC522_ToCard(PCD_TRANSCEIVE, buffer, 9, buffer, &recvBits);

  if ((status == MI_OK) && (recvBits == 0x18))
  {
    size = buffer[0];
  }
  else
  {
    size = 0;
  }

  return size;
}


/*
 * Function Name: MFRC522_Read
 * Description: Read block data
 * Input parameters: blockAddr - block address; recvData - read block data
 * Return value: the successful return MI_OK
 */
unsigned char MFRC522_Read(unsigned char blockAddr, unsigned char *recvData)
{
  unsigned char status;
  unsigned int unLen;

  recvData[0] = PICC_READ;
  recvData[1] = blockAddr;
  CalulateCRC(recvData,2, &recvData[2]);
  status = MFRC522_ToCard(PCD_TRANSCEIVE, recvData, 4, recvData, &unLen);

  if ((status != MI_OK) || (unLen != 0x90))
  {
    status = MI_ERR;
  }

  return status;
}
/*
 * Function Name: MFRC522_Write
 * Description: Write block data
 * Input parameters: blockAddr - block address; writeData - to 16-byte data block write
 * Return value: the successful return MI_OK
 */
unsigned char MFRC522_Write(unsigned char blockAddr, unsigned char *writeData)
{
  unsigned char status;
  unsigned int recvBits;
  unsigned char i;
  unsigned char buff[18];

  buff[0] = PICC_WRITE;
  buff[1] = blockAddr;
  CalulateCRC(buff, 2, &buff[2]);
  status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &recvBits);

  if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
  {
    status = MI_ERR;
  }

  if (status == MI_OK)
  {
    for (i=0; i<16; i++)                //Data to the FIFO write 16Byte
    {
      buff[i] = *(writeData+i);
    }
    CalulateCRC(buff, 16, &buff[16]);
    status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 18, buff, &recvBits);

    if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
    {
      status = MI_ERR;
    }
  }

  return status;
}


unsigned char PCD_Authentication(unsigned char cmd, unsigned char block_addr, const unsigned char *key, unsigned char *uid)
{
				unsigned int i,back;
				unsigned char command[12];
        command[0] = cmd;
        command[1] = block_addr;
        
        for( i = 0; i < 6; i++) {        // 6 key bytes
             command[2+i] = *key;
             key++;
        }
        // Use the last uid bytes as specified in http://cache.nxp.com/documents/application_note/AN10927.pdf
        // section 3.2.5 "MIFARE Classic Authentication".
        // The only missed case is the MF1Sxxxx shortcut activation,
        // but it requires cascade tag (CT) byte, that is not part of uid.
        for ( i = 0; i < 4; i++) {                                // The last 4 bytes of the UID
                command[8+i]= *uid;
                uid++;
        }
         
        
        MFRC522_ToCard (PCD_AUTHENT,command,12,command,&back);
				
				
				if(Read_MFRC522( Status2Reg)&0x08)
				{
				return MI_OK;
				}
				else{
					
					return MI_ERR;
					
				}
			
				
				
							

}



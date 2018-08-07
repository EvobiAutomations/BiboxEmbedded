#ifndef MFRC522
#define MFRC522



//MF522 Command word
#define PCD_IDLE              0x00               //NO action; Cancel the current command
#define PCD_AUTHENT           0x0E               //Authentication Key
#define PCD_RECEIVE           0x08               //Receive Data
#define PCD_TRANSMIT          0x04               //Transmit data
#define PCD_TRANSCEIVE        0x0C               //Transmit and receive data,
#define PCD_RESETPHASE        0x0F               //Reset
#define PCD_CALCCRC           0x03               //CRC Calculate

// Mifare_One card command word
#define PICC_REQIDL          0x26               // find the antenna area does not enter hibernation
#define PICC_REQALL          0x52               // find all the cards antenna area
#define PICC_ANTICOLL        0x93               // anti-collision
#define PICC_SElECTTAG       0x93               // election card
#define PICC_AUTHENT1A       0x60               // authentication key A
#define PICC_AUTHENT1B       0x61               // authentication key B
#define PICC_READ            0x30               // Read Block
#define PICC_WRITE           0xA0               // write block
#define PICC_DECREMENT       0xC0               // debit
#define PICC_INCREMENT       0xC1               // recharge
#define PICC_RESTORE         0xC2               // transfer block data to the buffer
#define PICC_TRANSFER        0xB0               // save the data in the buffer
#define PICC_HALT            0x50               // Sleep

//And MF522 The error code is returned when communication
#define MI_OK                 0               
#define MI_NOTAGERR           1
#define MI_ERR                2





//------------------MFRC522 Register---------------
//Page 0:Command and Status
#define     Reserved00            0x00
#define     CommandReg            0x01
#define     CommIEnReg            0x02
#define     DivlEnReg             0x03
#define     CommIrqReg            0x04
#define     DivIrqReg             0x05
#define     ErrorReg              0x06
#define     Status1Reg            0x07
#define     Status2Reg            0x08
#define     FIFODataReg           0x09
#define     FIFOLevelReg          0x0A
#define     WaterLevelReg         0x0B
#define     ControlReg            0x0C
#define     BitFramingReg         0x0D
#define     CollReg               0x0E
#define     Reserved01            0x0F

//Page 1:Command
#define     Reserved10            0x10
#define     ModeReg               0x11
#define     TxModeReg             0x12
#define     RxModeReg             0x13
#define     TxControlReg          0x14
#define     TxAutoReg             0x15
#define     TxSelReg              0x16
#define     RxSelReg              0x17
#define     RxThresholdReg        0x18
#define     DemodReg              0x19
#define     Reserved11            0x1A
#define     Reserved12            0x1B
#define     MifareReg             0x1C
#define     Reserved13            0x1D
#define     Reserved14            0x1E
#define     SerialSpeedReg        0x1F

//Page 2:CFG
#define     Reserved20            0x20
#define     CRCResultRegM         0x21
#define     CRCResultRegL         0x22
#define     Reserved21            0x23
#define     ModWidthReg           0x24
#define     Reserved22            0x25
#define     RFCfgReg              0x26
#define     GsNReg                0x27
#define     CWGsPReg              0x28
#define     ModGsPReg             0x29
#define     TModeReg              0x2A
#define     TPrescalerReg         0x2B
#define     TReloadRegH           0x2C
#define     TReloadRegL           0x2D
#define     TCounterValueRegH     0x2E
#define     TCounterValueRegL     0x2F

//Page 3:TestRegister
#define     Reserved30            0x30
#define     TestSel1Reg           0x31
#define     TestSel2Reg           0x32
#define     TestPinEnReg          0x33
#define     TestPinValueReg       0x34
#define     TestBusReg            0x35
#define     AutoTestReg           0x36
#define     VersionReg            0x37
#define     AnalogTestReg         0x38
#define     TestDAC1Reg           0x39
#define     TestDAC2Reg           0x3A
#define     TestADCReg            0x3B
#define     Reserved31            0x3C
#define     Reserved32            0x3D
#define     Reserved33            0x3E
#define     Reserved34            0x3F	



	
#define MAX_LEN 16


/*******************************************************************
**************** Write spi data to RC522 register ******************

@parm	1. register address 
			2. Data to be written
*******************************************************************/

void Write_MFRC522(unsigned char addr, unsigned char val);




/*******************************************************************
**************** Read spi data to RC522 register *******************
@parm	1. register address 
			2. Data to be written
*******************************************************************/

unsigned char Read_MFRC522(unsigned char addr);



/*******************************************************************
**************** Set bit mask **************************************
@parm	1. register address 
			2. Mask
*******************************************************************/
void SetBitMask(unsigned char reg, unsigned char mask);



/*******************************************************************
**************** Clear bit mask ************************************

@parm	1. register address 
			2. Mask
*******************************************************************/
void ClearBitMask(unsigned char reg, unsigned char mask);



/*******************************************************************
**************** Communicate with card *****************************
@parm	1. Commnad to be executed
			2. data to be send
			3. No. of bytes to be send
			4. Buffer to which the returned bytes will be stored
			5. No of bits received
@return MI_OK if communication is success else MI_ERR
*******************************************************************/

unsigned char MFRC522_ToCard(unsigned char command, unsigned char*sendData, unsigned char sendLen, unsigned char *backData,unsigned int *backLen);



/*******************************************************************
**************** Send request to the card(polling) *****************

@parm	1. Req type
			2. Received ATQA reply
			
@return MI_OK if communication is success else MI_ERR
*******************************************************************/

unsigned char MFRC522_Request(unsigned char reqMode, unsigned char *TagType);




/*******************************************************************
**************** Reset reader *****************************
*******************************************************************/

void MFRC522_Reset(void);




/*******************************************************************
**************** Initialize the device *****************************
*******************************************************************/

void MFRC522_Init(void);


/*******************************************************************
**************** Excute Anti collision procedure *******************

@parm	1.Buffer to which the ser number will be stored

@return MI_OK if communication is success else MI_ERR
*******************************************************************/

unsigned char MFRC522_Anticoll(unsigned char *serNum);





/*******************************************************************
**************** Calculate CRC *************************************
@parm	1.Input data
			2.No. of data
			3.Result location
		
@return MI_OK if communication is success else MI_ERR
*******************************************************************/

unsigned char CalulateCRC(unsigned char *pIndata, unsigned char len, unsigned char *pOutData);


/*******************************************************************
**************** Put card into hibernation *************************
*******************************************************************/

void MFRC522_Halt(void);


/*******************************************************************
**************** Select card  **************************************
@parm	1.Ser number
		
		
@return MI_OK if communication is success else MI_ERR
*******************************************************************/
unsigned char MFRC522_SelectTag(unsigned char *serNum);



/*******************************************************************
**************** Execute authentication procedure ******************
@parm	1.Authentication mode
			2.Block address
			3.Key
			4.ser no.
		
@return MI_OK if communication is success else MI_ERR
*******************************************************************/

unsigned char PCD_Authentication(unsigned char authMode, unsigned char BlockAddr,const unsigned char *Sectorkey, unsigned char *serNum);


/*******************************************************************
**************** Read block from card ******************************
@parm	1.Bllock address
			2.Buffer for received data (min 16bytes of size)
			
		
@return MI_OK if communication is success else MI_ERR
*******************************************************************/
unsigned char MFRC522_Read(unsigned char blockAddr, unsigned char *recvData);




/*******************************************************************
**************** Write data to card ********************************
@parm	1.Bllock address
			2.data to write
		
@return MI_OK if communication is success else MI_ERR
*******************************************************************/
unsigned char MFRC522_Write(unsigned char blockAddr, unsigned char *writeData);




void Write_MFRC522_Long(unsigned char addr,unsigned char len, unsigned char* val);


void Read_Long_MFRC522(unsigned char reg,unsigned char count,unsigned char* values,unsigned char rxAlign );




#endif 



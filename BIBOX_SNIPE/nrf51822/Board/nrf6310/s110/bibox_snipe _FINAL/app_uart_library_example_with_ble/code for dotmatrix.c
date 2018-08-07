

/* variable for holding display content */

unsigned char Dot_Matrix1,Dot_Matrix2,Dot_Matrix3,Dot_Matrix4,Dot_Matrix5; 





/************** IC initializing ***********************/





if(code_array[14]=='D')
		{
			Soft_Spi_Initial();
			Max_Shutdown(false,5);
			Max_Decode_Mode(NO_DECODE,5);
			Max_Intensity(0xff,5);
			Max_Scan_Limit(0x7,5);
			Max_Display_test(false,5);
			Max_Display_Clear(5);
		}
		
		
	
		
		
/* =========================================================================== */
/* ========================= Send data to dot matrix disply ================== */
/* =========================================================================== */
void Send_Dotmatrix(void)
{
	unsigned char Dot_Matrix[5];
	
	Dot_Matrix[0]=Dot_Matrix1;				/* saving display contents buffer*/
	Dot_Matrix[1]=Dot_Matrix2;
	Dot_Matrix[2]=Dot_Matrix3;
	Dot_Matrix[3]=Dot_Matrix4;
	Dot_Matrix[4]=Dot_Matrix5;
	Max_Display_Char_X(&Dot_Matrix[0],5);	// Send data to five displays
	
	
}	
		






void output_function(void)
{
	switch(port)
	{
case 59:		
			Millis=value;
			break;
	case 60:
			Dot_Matrix1=value;
			Send_Dotmatrix();
		
			break;
	
	case 61:
			Dot_Matrix2=value;
			Send_Dotmatrix();
		
			break;
	case 62:
			Dot_Matrix3=value;
			Send_Dotmatrix();
	
			break;
	case 63:
			Dot_Matrix4=value;
			Send_Dotmatrix();
	
			break;
	
	case 64:
			Dot_Matrix5=value;
			Send_Dotmatrix();
	
			break;
	}

}









/* ********************************************************************************
 * 
 * File: main.c
 * 
 * This file is a combination of RTTY code by Rob Harisson from the Icarus project
 * (http://www.robertharrison.org/svn/), 
 * code from Texas Instruments and code written by Maria Christopoulou (SV1PLE).
 * 
 * 	RTTY Baudot and ASCII Encoder.
 * 
 * 	TODO:
 * 	1) Modularize the functions
 * 	2) Add decoder
 * 
 * 	
 *  
 * 
 ***************************************************************************************

#include "stdio.h"
#include "string.h"
#include "usbstk5505.h"
#include "aic3204.h"
#include "PLL.h"
#include "sinewaves.h"

#define SAMPLES_PER_SECOND 48000
#define GAIN_IN_dB         0
#define ASCII 7 // 8 for 8 bit ascii
#define BAUDOT_LTRS	0x1F
#define BAUDOT_FIGS	0x1B
#define BAUDOT_SPACE	0x04


/*
 * 0 unknown state
 * 1 LTRS state
 * 2 FIGS state
 */


static char
baudot_encode_table[0x60][2] = {
    // index: ascii char; values: bits, ltrs_or_figs_or_neither_or_both

  /* 0x00 */
    /* NUL */	{ 0x00, 3 },	// NUL
    /* xxx */	{ 0, 0 },	// non-encodable
    /* xxx */	{ 0, 0 },	// non-encodable
    /* xxx */	{ 0, 0 },	// non-encodable
    /* xxx */	{ 0, 0 },	// non-encodable
    /* xxx */	{ 0, 0 },	// non-encodable
    /* xxx */	{ 0, 0 },	// non-encodable
    /* BEL */	{ 0x05, 2 },	// BELL (or CCITT2 apostrophe)
    /* BS */	{ 0, 0 },	// non-encodable (FIXME???)
    /* xxx */	{ 0, 0 },	// non-encodable
    /* LF */	{ 0x02, 3 },	// LF
    /* xxx */	{ 0, 0 },	// non-encodable
    /* xxx */	{ 0, 0 },	// non-encodable
    /* 0xD */	{ 0x08, 3 },	// CR
    /* xxx */	{ 0, 0 },	// non-encodable
    /* xxx */	{ 0, 0 },	// non-encodable

  /* 0x10 */
    /* xxx */	{ 0, 0 },	// non-encodable
    /* xxx */	{ 0, 0 },	// non-encodable
    /* xxx */	{ 0, 0 },	// non-encodable
    /* xxx */	{ 0, 0 },	// non-encodable
    /* xxx */	{ 0, 0 },	// non-encodable
    /* xxx */	{ 0, 0 },	// non-encodable
    /* xxx */	{ 0, 0 },	// non-encodable
    /* xxx */	{ 0, 0 },	// non-encodable
    /* xxx */	{ 0, 0 },	// non-encodable
    /* xxx */	{ 0, 0 },	// non-encodable
    /* xxx */	{ 0, 0 },	// non-encodable
    /* xxx */	{ 0, 0 },	// non-encodable
    /* xxx */	{ 0, 0 },	// non-encodable
    /* xxx */	{ 0, 0 },	// non-encodable
    /* xxx */	{ 0, 0 },	// non-encodable
    /* xxx */	{ 0, 0 },	// non-encodable

  /* 0x20 */
    /*   */	{ 0x04, 3 },	// SPACE
    /* ! */	{ 0x0d, 2 },	//
    /* " */	{ 0x11, 2 },	//
    /* # */	{ 0x14, 2 },	// '#' (or CCITT2 British pounds symbol)
    /* $ */	{ 0x09, 2 },	// '$' (or CCITT2 ENQ)
    /* % */	{ 0, 0 },	// non-encodable
    /* & */	{ 0x1a, 2 },	//
    /* ' */	{ 0x0b, 2 },	// apostrophe (or CCITT2 BELL)
    /* ( */	{ 0x0f, 2 },	//
    /* ) */	{ 0x12, 2 },	//
    /* * */	{ 0, 0 },	// non-encodable
    /* + */	{ 0x12, 2 },	//
    /* , */	{ 0x0c, 2 },	//
    /* - */	{ 0x03, 2 },	//
    /* . */	{ 0x1c, 2 },	//
    /* / */	{ 0x1d, 2 },	//

  /* 0x30 */
    /* 0 */	{ 0x16, 2 },	//
    /* 1 */	{ 0x17, 2 },	//
    /* 2 */	{ 0x13, 2 },	//
    /* 3 */	{ 0x01, 2 },	//
    /* 4 */	{ 0x0a, 2 },	//
    /* 5 */	{ 0x10, 2 },	//
    /* 6 */	{ 0x15, 2 },	//
    /* 7 */	{ 0x07, 2 },	//
    /* 8 */	{ 0x06, 2 },	//
    /* 9 */	{ 0x18, 2 },	//
    /* : */	{ 0x0e, 2 },	//
    /* ; */	{ 0x1e, 2 },	//
    /* < */	{ 0, 0 },	// non-encodable
    /* = */	{ 0, 0 },	// non-encodable
    /* > */	{ 0, 0 },	// non-encodable
    /* ? */	{ 0x19, 2 },	//

  /* 0x40 */
    /* @ */	{ 0, 0 },	// non-encodable
    /* A */	{ 0x03, 1 },	//
    /* B */	{ 0x19, 1 },	//
    /* C */	{ 0x0e, 1 },	//
    /* D */	{ 0x09, 1 },	//
    /* E */	{ 0x01, 1 },	//
    /* F */	{ 0x0d, 1 },	//
    /* G */	{ 0x1a, 1 },	//
    /* H */	{ 0x14, 1 },	//
    /* I */	{ 0x06, 1 },	//
    /* J */	{ 0x0b, 1 },	//
    /* K */	{ 0x0f, 1 },	//
    /* L */	{ 0x12, 1 },	//
    /* M */	{ 0x1c, 1 },	//
    /* N */	{ 0x0c, 1 },	//
    /* O */	{ 0x18, 1 },	//

  /* 0x50 */
    /* P */	{ 0x16, 1 },	//
    /* Q */	{ 0x17, 1 },	//
    /* R */	{ 0x0a, 1 },	//
    /* S */	{ 0x05, 1 },	//
    /* T */	{ 0x10, 1 },	//
    /* U */	{ 0x07, 1 },	//
    /* V */	{ 0x1e, 1 },	//
    /* W */	{ 0x13, 1 },	//
    /* X */	{ 0x1d, 1 },	//
    /* Y */	{ 0x15, 1 },	//
    /* Z */	{ 0x11, 1 },	//
    /* [ */	{ 0, 0 },	// non-encodable
    /* \\ */	{ 0, 0 },	// non-encodable
    /* ] */	{ 0, 0 },	// non-encodable
    /* ^ */	{ 0, 0 },	// non-encodable
    /* _ */	{ 0, 0 },	// non-encodable

};




Int16 left_input;
Int16 right_input;
Int16 left_output;
Int16 right_output;
unsigned long int i = 0;
float bit_duration;
int bits;
int k;
int ind;


/*--------------------------------------------------------------------
 * rtty_txbit
 * 
 * Assign frequencies on bits 0,1
 * 
 * -------------------------------------------------------------------*/
  
void rtty_txbit (int bit, float bit_duration)
{
  if (bit)
  {
    // Mark
     	for ( i = 0  ; i < SAMPLES_PER_SECOND * bit_duration  ;i++  )
 			{

        		left_output = generate_sinewave_1(2295, 1000);
				right_output = generate_sinewave_2(2295,1000); 
    
     	 		aic3204_codec_write(left_output, right_output);
 			}
  }
  else
  {
    // Space
 	 	for ( i = 0  ; i < SAMPLES_PER_SECOND * bit_duration  ;i++  )
 			{
     		
         	left_output = generate_sinewave_1(2125, 1000); 
     	 	right_output = generate_sinewave_2(2125,1000); 
    
     	 	aic3204_codec_write(left_output, right_output);
 			}
  }
 
 
}

void rtty_txhalfbit(int bit, float bit_duration)
{
	if (bit)
	{
		for (i = 0; i < SAMPLES_PER_SECOND * (bit_duration/2); i++)
		{
			
        		left_output = generate_sinewave_1(2295, 1000);
				right_output = generate_sinewave_2(2295,1000); 
    
     	 		aic3204_codec_write(left_output, right_output);
		}
	}
}

void rtty_txbyte(char c, int enc, float stop_bits, float bit_duration)
{
  /* Simple function to sent each bit of a char to 
   	** rtty_txbit function. 
   	** NB The bits are sent Least Significant Bit first
   	**
   	** All chars should be preceded with a 0 and 
   	** proceded with a 1. 0 = Start bit; 1 = Stop bit
   	** enc: 1 for ASCII, 2 for BAUDOT
   	*/
 
  //int i;
  int flag;
 int ind;
 //c = *MessagePtr;
 
 
 if (enc==1)
 {
 	  // Send bits for for char LSB first	
  rtty_txbit(0, bit_duration); // Start bit
  	for (i=0;i<ASCII;i++) // Change this here 7 or 8 for ASCII-7 / ASCII-8
  	{
  		
  		 
    	if (c & 1) rtty_txbit(1, bit_duration); 
 
    	else rtty_txbit(0,bit_duration);	
 
    	c = c >> 1;
  	}
 	    	if (stop_bits == 2)
  			{
  				rtty_txbit(1,bit_duration);
  				rtty_txbit(1,bit_duration);
  			}
  			else
  			{	// Send 1.5 bit
  				rtty_txhalfbit(1,bit_duration);
			}
 
  	}
 
  	
 else if (enc==2)
 {
 	ind = (int)c;
	bits = baudot_encode_table[ind][0];
			 bits &= 0x1F;
	if (baudot_encode_table[ind][1] == 1)
	{
		
		flag = BAUDOT_LTRS;
		rtty_txbit(0,bit_duration);
		for (k=0;k<5;k++)
            {
                if (flag&1)
                    {
                    	rtty_txbit(1,bit_duration);
                    	                    	
                    }
                else
                {
                	rtty_txbit(0,bit_duration);              							
                	
                }
                
                
                flag = flag >> 1;
            }
             	    	if (stop_bits == 2)
  			{
  				rtty_txbit(1,bit_duration);
  				rtty_txbit(1,bit_duration);
  			}
  			else
  			{	// Send 1.5 bit
  				rtty_txhalfbit(1,bit_duration);
			}
	}
	
	else if (baudot_encode_table[ind][1] == 2)
	{
		flag = BAUDOT_FIGS;
		rtty_txbit(0,bit_duration);
		for (k=0;k<5;k++)
            {
                if (flag&1)
                    {
                    	rtty_txbit(1,bit_duration);
                    	                    	
                    }
                else
                {
                	rtty_txbit(0,bit_duration);              							
                	
                }
                
                
                flag = flag >> 1;
            }
             	    	if (stop_bits == 2)
  			{
  				rtty_txbit(1,bit_duration);
  				rtty_txbit(1,bit_duration);
  			}
  			else
  			{	// Send 1.5 bit
  				rtty_txhalfbit(1,bit_duration);
			}
	}
		
		
							
			rtty_txbit(0,bit_duration); // Start bit
			for (k=0;k<5;k++)
            {
                if (bits&1)
                    {
                    	rtty_txbit(1,bit_duration);
                    	                    	
                    }
                else
                {
                	rtty_txbit(0,bit_duration);              							
                	
                }
                
                
                bits = bits >> 1;
                
             }
              	    	if (stop_bits == 2)
  			{
  				rtty_txbit(1,bit_duration);
  				rtty_txbit(1,bit_duration);
  			}
  			else
  			{	// Send 1.5 bit
  				rtty_txhalfbit(1,bit_duration);
			}
             
             
 	
} // end of if(enc==2) 

} // end of function


/* ------------------------------------------------------------------------ *
 *                                                                          *
 *  main( )                                                                 *
 *                                                                          *
 * ------------------------------------------------------------------------ */
void main( void ) 
{
	int enc;
	float baud_rate;
	float stop_bits;
	//int ind;
	char c;
	char Message[]="CQ CQ CQ DE SV1PLE 012345679 A LAZY FOX JUMPED CQ CQ CQ DE SV1PLE 012345679 A LAZY FOX JUMPED CQ CQ CQ DE SV1PLE 012345679 A LAZY FOX JUMPED";
	char *MessagePtr;
	MessagePtr=Message;
    /* Initialize BSL */
    USBSTK5505_init( );
    
    /* Initialize the Phase Locked Loop in EEPROM */
    pll_frequency_setup(100);

    /* Initialise hardware interface and I2C for code */
    aic3204_hardware_init();
    
    /* Initialise the AIC3204 codec */
	aic3204_init(); 

 
	/* Set sampling frequency in Hz and ADC gain in dB */
    set_sampling_frequency_and_gain(SAMPLES_PER_SECOND, GAIN_IN_dB);
	
	

	// Silly Introduction
	printf("\n");
	printf("**************************************************");
	printf("\n\tEzDSP RTTY digital trasmitter");
	printf("\n\tMaria Christopoulou SV1PLE");
	printf("\n*************************************************");
	printf("\n\n");
	
	// Ask user which encoding to use: ASCII(A) or BAUDOT (B)?
	printf("Which encoding to use? ASCII (1) or BAUDOT (2)?\n");
	scanf("%d", &enc);	
	
	// Ask user the Baud rate
	printf("\nChoose Baud Rate (45.45, 75)");
	scanf("%f", &baud_rate);
	//Set bit time duration
	bit_duration = 1/baud_rate;
	// Ask user how many stop bits to use: 1.5 or 2 bits
	printf("\nHow many stop bits to use: 1.5 or 2?");
	scanf("%f", &stop_bits); 
	
	
	
	while(*MessagePtr != '\0')
	{
		c = *MessagePtr;
		rtty_txbyte(c, enc, stop_bits,bit_duration);
		*MessagePtr++;
	} 
	
             
        
		
	

		
		
	
   /* Disable I2S and put codec into reset */ 
    aic3204_disable();

    printf( "\n***Program has Terminated***\n" );
    SW_BREAKPOINT;
}

/* ------------------------------------------------------------------------ *
 *                                                                          *
 *  End of main.c                                                           *
 *                                                                          *
 * ------------------------------------------------------------------------ */













// PIC18F4520 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config OSC = INTIO67      // Oscillator Selection bits (HS oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = ON       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = OFF       // Power-up Timer Enable bit (PWRT disabled)
#pragma config BOREN = SBORDIS  // Brown-out Reset Enable bits (Brown-out Reset enabled in hardware only (SBOREN is disabled))
#pragma config BORV = 3         // Brown Out Reset Voltage bits (Minimum setting)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 1        // Watchdog Timer Postscale Select bits (1:1)

// CONFIG3H
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.
/* SSPCON1 REGISTER */
#define   SSPENB    			0b00100000  	/* Enable serial port and configures SCK, SDO, SDI*/
#define   SLAVE_7   			0b00000110     	/* I2C Slave mode, 7-bit address*/
#define   SLAVE_10  			0b00000111    	/* I2C Slave mode, 10-bit address*/
#define   MASTER    			0b00001000     	/* I2C Master mode */
#define   MASTER_FIRMW			0b00001011		//I2C Firmware Controlled Master mode (slave Idle)
#define   SLAVE_7_STSP_INT 		0b00001110		//I2C Slave mode, 7-bit address with Start and Stop bit interrupts enabled
#define   SLAVE_10_STSP_INT 	0b00001111		//I2C Slave mode, 10-bit address with Start and Stop bit interrupts enabled

/* SSPSTAT REGISTER */
#define   SLEW_OFF  			0b10000000  	/* Slew rate disabled for 100kHz mode */
#define   SLEW_ON   			0b00000000  	/* Slew rate enabled for 400kHz mode  */
#define USE_OR_MASKS

#include <xc.h>
#include <pic18f4520.h>

void MyOpenI2C( unsigned char sync_mode, unsigned char slew );
void MyRestartI2C( void );
void MyStartI2C();
void MyCloseI2C();
void MyIdleI2C();
signed char MyWriteI2C( unsigned char data_out );
unsigned char MyReadI2C( void );
signed char MyputsI2C( unsigned char *wrptr );
signed char MygetsI2C( unsigned char *rdptr, unsigned char length );
void MyNotAckI2C( void );

unsigned char I2C_Send[21]="MICROCHIP:I2C_SLAVE";
unsigned char I2C_Recv[21];
void main(void) {
    unsigned char sync_mode=0, slew = 0,data=0,add1=0,status=0,temp=0,i=0,length=0;
    for(i=0;i<21;i++)
        I2C_Recv[i]=0;    
    MyCloseI2C();
    //initialize module
    sync_mode = SLAVE_7;
    slew = SLEW_OFF;
    MyOpenI2C(sync_mode,slew);
    
    //data = SSPBUF;
    //slave address
    SSPADD=0xA5;
    
    //read address sent by master from buffer
    while(SSPSTATbits.BF==0); //wait until master send address
    temp=MyReadI2C();
    //data reception from master
    do{
        while(SSPSTATbits.BF==0);     //wait until master send data
        I2C_Recv[length++]=MyReadI2C();//save byte received
    }while(length!=20);
    //wait until stop condition
    while(SSPSTATbits.S != 1);    
    
    while(SSPSTATbits.BF==0);
    temp=MyReadI2C();//read address send by master
    //slave transmit
    if(SSPSTAT & 0x04)//chaeck if master ready for reception
        while(MyputsI2C(I2C_Send));
        
        MyCloseI2C();
        while(1);
    
    return;
}
void MyStartI2C()
{
  SSPCON2bits.SEN = 1;            // initiate bus start condition
}
void MyCloseI2C()
{
  SSPCON1 &= 0xDF;                // disable synchronous serial port
}
void MyIdleI2C()
{
  while ( ( SSPCON2 & 0x1F ) || ( SSPSTATbits.R_W ) )
     continue;
}
signed char MyWriteI2C( unsigned char data_out )
{
  SSPBUF = data_out;           // write single byte to SSPBUF
  if ( SSPCON1bits.WCOL )      // test if write collision occurred
   return ( -1 );              // if WCOL bit is set return negative #
  else
  {
	if( ((SSPCON1&0x0F)!=0x08) && ((SSPCON1&0x0F)!=0x0B) )	//Slave mode only
	{
	      SSPCON1bits.CKP = 1;        // release clock line 
	      while ( !PIR1bits.SSPIF );  // wait until ninth clock pulse received

	      if ( ( !SSPSTATbits.R_W ) && ( !SSPSTATbits.BF ) )// if R/W=0 and BF=0, NOT ACK was received
	      {
	        return ( -2 );           //return NACK
	      }
		  else
		  {
			return ( 0 );				//return ACK
		  }	
	}
	else if( ((SSPCON1&0x0F)==0x08) || ((SSPCON1&0x0F)==0x0B) )	//master mode only
	{ 
	    while( SSPSTATbits.BF );   // wait until write cycle is complete   
	    MyIdleI2C();                 // ensure module is idle
	    if ( SSPCON2bits.ACKSTAT ) // test for ACK condition received
	    	 return ( -2 );			// return NACK
		else return ( 0 );              //return ACK
	}
	
  }
}
signed char MyputsI2C( unsigned char *wrptr )
{
	  unsigned char temp;
  while ( *wrptr )                // transmit data until null character 
  {
    if ( SSPCON1bits.SSPM3 )      // if Master transmitter then execute the following
    {
	  temp = MyWriteI2C ( *wrptr );
	  if (temp ) return ( temp );            // return with write collision error
    
//      if ( putcI2C ( *wrptr ) )   // write 1 byte
//      {
//        return ( -3 );            // return with write collision error
//      }
//      IdleI2C();                  // test for idle condition
//      if ( SSPCON2bits.ACKSTAT )  // test received ack bit state
//      {
//        return ( -2 );            // bus device responded with  NOT ACK
//      }                           // terminate putsI2C() function
    }

    else                          // else Slave transmitter
    {
      PIR1bits.SSPIF = 0;         // reset SSPIF bit
      SSPBUF = *wrptr;            // load SSPBUF with new data
      SSPCON1bits.CKP = 1;        // release clock line 
      while ( !PIR1bits.SSPIF );  // wait until ninth clock pulse received

      if ( ( SSPCON1bits.CKP ) && ( !SSPSTATbits.BF ) )// if R/W=0 and BF=0, NOT ACK was received
      {
        return ( -2 );            // terminate PutsI2C() function
      }
    }

  wrptr ++;                       // increment pointer

  }                               // continue data writes until null character

  return ( 0 );
}
void MyOpenI2C( unsigned char sync_mode, unsigned char slew )
{
  SSPSTAT &= 0x3F;                // power on state 
  SSPCON1 = 0x00;                 // power on state
  SSPCON2 = 0x00;                 // power on state
  SSPCON1 |= sync_mode;           // select serial mode 
  SSPSTAT |= slew;                // slew rate on/off 

  SCL = 1;
  SDA = 1;
  SSPCON1 |= SSPENB;              // enable synchronous serial port 

}
void MyRestartI2C( void )
{
  SSPCON2bits.RSEN = 1;           // initiate bus restart condition
}
signed char MygetsI2C( unsigned char *rdptr, unsigned char length )
{
    while ( length-- )           // perform getcI2C() for 'length' number of bytes
    {
      *rdptr++ = MyReadI2C();       // save byte received
      while ( SSPCON2bits.RCEN ); // check that receive sequence is over    

      if ( PIR2bits.BCLIF )       // test for bus collision
      {
        return ( -1 );            // return with Bus Collision error 
      }

	if( ((SSPCON1&0x0F)==0x08) || ((SSPCON1&0x0F)==0x0B) )	//master mode only
	{	
      if ( length )               // test if 'length' bytes have been read
      {
        SSPCON2bits.ACKDT = 0;    // set acknowledge bit state for ACK        
        SSPCON2bits.ACKEN = 1;    // initiate bus acknowledge sequence
        while ( SSPCON2bits.ACKEN ); // wait until ACK sequence is over 
      } 
	} 
	  
    }
    return ( 0 );                 // last byte received so don't send ACK      
}
void MyNotAckI2C( void )
{
  SSPCON2bits.ACKDT = 1;          // set acknowledge bit for not ACK
  SSPCON2bits.ACKEN = 1;          // initiate bus acknowledge sequence
}
unsigned char MyReadI2C( void )
{
    if( ((SSPCON1&0x0F)==0x08) || ((SSPCON1&0x0F)==0x0B) )	//master mode only
    SSPCON2bits.RCEN = 1;           // enable master for 1 byte reception
    while ( !SSPSTATbits.BF );      // wait until byte received  
    return ( SSPBUF );              // return with read byte 
}
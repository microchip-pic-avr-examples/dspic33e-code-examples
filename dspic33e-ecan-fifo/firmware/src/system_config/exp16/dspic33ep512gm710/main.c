/*******************************************************************************
  ce427 main source file

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This file consists of functions that send and receive messages through the
    ECAN1 and ECAN2 when the transmitter and receiver are crosswired.

*******************************************************************************/
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <xc.h>
#include <stdint.h>
#include "ecan1_config.h"
#include "ecan2_config.h"
#include "common.h"

#if __XC16_VERSION < 1011
#warning "Please upgrade to XC16 v1.11 or newer."
#endif

// *****************************************************************************
// *****************************************************************************
// Section: Macros for Configuration Fuse Registers:
// *****************************************************************************
// *****************************************************************************
/* Invoke macros to set up  device configuration fuse registers.The fuses will
   select the oscillator source, power-up timers, watch-dog timers etc. The
   macros are defined within the device header files. The configuration fuse
   registers reside in Flash memory.
 */
// DSPIC33EP512GM710 Configuration Bit Settings
// 'C' source line config statements
// FICD
#pragma config ICS = PGD1           // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config JTAGEN = OFF         // JTAG Enable bit (JTAG is disabled)

// FPOR
#pragma config BOREN = ON           // Brown-out Reset (BOR) Detection Enable bit (BOR is enabled)
#pragma config ALTI2C1 = OFF        // Alternate I2C1 pins (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2 = OFF        // Alternate I2C2 pins (I2C2 mapped to SDA2/SCL2 pins)
#pragma config WDTWIN = WIN25       // Watchdog Window Select bits (WDT Window is 25% of WDT period)

// FWDT
#pragma config WDTPOST = PS32768    // Watchdog Timer Postscaler bits (1:32,768)
#pragma config WDTPRE = PR128       // Watchdog Timer Prescaler bit (1:128)
#pragma config PLLKEN = ON          // PLL Lock Enable bit (Clock switch to PLL source will wait until the PLL lock signal is valid.)
#pragma config WINDIS = OFF         // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF         // Watchdog Timer Enable bit (Watchdog timer enabled/disabled by user software)

// FOSC
#pragma config POSCMD = XT          // Primary Oscillator Mode Select bits (XT Crystal Oscillator Mode)
#pragma config OSCIOFNC = OFF       // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config IOL1WAY = OFF        // Peripheral pin select configuration (Allow multiple reconfigurations)
#pragma config FCKSM = CSECMD       // Clock Switching Mode bits (Clock switching is enabled,Fail-safe Clock Monitor is disabled)

// FOSCSEL
#pragma config FNOSC = FRC          // Oscillator Source Selection (Internal Fast RC (FRC))
#pragma config PWMLOCK = ON         // PWM Lock Enable bit (Certain PWM registers may only be written after key sequence)
#pragma config IESO = OFF           // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FGS
#pragma config GWRP = OFF           // General Segment Write-Protect bit (General Segment may be written)
#pragma config GCP = OFF            // General Segment Code-Protect bit (General Segment Code protect is Disabled)

// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************
// Define ECAN Message Buffers
__eds__ ECAN1MSGBUF ecan1msgBuf __attribute__( (space(eds), aligned(ECAN1_MSG_BUF_LENGTH * 16)) );
__eds__ ECAN2MSGBUF ecan2msgBuf __attribute__( (space(eds), aligned(ECAN2_MSG_BUF_LENGTH * 16)) );

// *****************************************************************************
// Section: Static Function declaration
// *****************************************************************************
void                            OscConfig( void );
void                            ClearIntrflags( void );
void                            Ecan1WriteMessage( void );
void                            Ecan2WriteMessage( void );

// *****************************************************************************
// Section: Function definition
// *****************************************************************************

/******************************************************************************
 * Function:        int main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Sets CPU clock and initializes CAN1 and CAN2.
 *****************************************************************************/
#ifdef TEST_MODE
unsigned char test_flag1,test_flag2;
#endif
#ifdef TEST_MODE
int ce_main(void)
#else
int main(void)
#endif
{

#ifdef TEST_MODE
    test_flag1= test_flag2= 0;
#endif
    /* Configure Oscillator Clock Source     */
    OscConfig();

    /* Clear Interrupt Flags                 */
    ClearIntrflags();

    /* ECAN1 Initialisation         
   Configure DMA Channel 0 for ECAN1 Transmit
   Configure DMA Channel 2 for ECAN1 Receive */
    Ecan1Init();
    DMA0Init();
    DMA2Init();

    /* Enable ECAN1 Interrupt */
    IEC2bits.C1IE = 1;
    C1INTEbits.TBIE = 1;
    C1INTEbits.RBIE = 1;

    /* ECAN2 Initialisation         
   Configure DMA Channel 1 for ECAN2 Transmit
   Configure DMA Channel 3 for ECAN2 Receive */
    Ecan2Init();
    DMA1Init();
    DMA3Init();

    /* Enable ECAN2 Interrupt */
    IEC3bits.C2IE = 1;
    C2INTEbits.TBIE = 1;
    C2INTEbits.RBIE = 1;

    /* Write Message to ECAN1 Message Buffer 0 to 5 	
   Request Message Transmission			*/
    Ecan1WriteMessage();

    C1TR45CONbits.TXREQ5 = 1;
    C1TR45CONbits.TXREQ4 = 1;
    C1TR23CONbits.TXREQ3 = 1;
    C1TR23CONbits.TXREQ2 = 1;

    C1TR01CONbits.TXREQ1 = 1;
    C1TR01CONbits.TXREQ0 = 1;

    /* Write Message to ECAN2 Message Buffer 0 to 5 
   Request Message Transmission			*/
    Ecan2WriteMessage();
    C2TR01CONbits.TXREQ0 = 1;
    C2TR01CONbits.TXREQ1 = 1;
    C2TR23CONbits.TXREQ2 = 1;
    C2TR23CONbits.TXREQ3 = 1;
    C2TR45CONbits.TXREQ4 = 1;
    C2TR45CONbits.TXREQ5 = 1;
#ifdef TEST_MODE
    if (
            (ecan1msgBuf[8][3] == 0xFFFF) && (ecan1msgBuf[8][4] == 0xFFFF)  &&
            (ecan1msgBuf[8][5] == 0xFFFF) && (ecan1msgBuf[8][6] == 0xFFFF)  &&
            (ecan1msgBuf[9][3] == 0xEEEE) && (ecan1msgBuf[9][4] == 0xEEEE)  &&
            (ecan1msgBuf[9][5] == 0xEEEE) && (ecan1msgBuf[9][6] == 0xEEEE)  &&
            (ecan1msgBuf[10][3] == 0xDDDD) &&(ecan1msgBuf[10][4] == 0xDDDD) &&
            (ecan1msgBuf[10][5] == 0xDDDD) &&(ecan1msgBuf[10][6] == 0xDDDD) &&
            (ecan1msgBuf[11][3] == 0xCCCC) && (ecan1msgBuf[11][4] == 0xCCCC)  &&
            (ecan1msgBuf[11][5] == 0xCCCC) && (ecan1msgBuf[11][6] == 0xCCCC)
         )
        test_flag1=1;
    else
        test_flag1=0;
 if (
            (ecan2msgBuf[8][3] == 0x6666) && (ecan2msgBuf[8][4] == 0x6666)  &&
            (ecan2msgBuf[8][5] == 0x6666) && (ecan2msgBuf[8][6] == 0x6666)  &&
            (ecan2msgBuf[9][3] == 0x5555) && (ecan2msgBuf[9][4] == 0x5555)  &&
            (ecan2msgBuf[9][5] == 0x5555) && (ecan2msgBuf[9][6] == 0x5555)  &&
            (ecan2msgBuf[10][3] == 0x4444) &&(ecan2msgBuf[10][4] == 0x4444) &&
            (ecan2msgBuf[10][5] == 0x4444) &&(ecan2msgBuf[10][6] == 0x4444) &&
            (ecan2msgBuf[11][3] == 0x3333) && (ecan2msgBuf[11][4] == 0x3333)  &&
            (ecan2msgBuf[11][5] == 0x3333) && (ecan2msgBuf[11][6] == 0x3333)
         )
        test_flag2=1;
    else
        test_flag2=0;

    if( (test_flag1==1) && (test_flag2 ==1))
        return 0;
    else
        return 1;
#else
    /* Loop infinitely */
    while( 1 );
#endif
}

/******************************************************************************
 * Function:        void Ecan1WriteMessage(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is used to write the ECAN1 buffer with the
 *                  identifiers and the data. Writes the message to be
 *                  transmitted.
 *****************************************************************************/
void Ecan1WriteMessage( void )
{
    /* Writing the message for Transmission
Ecan1WriteTxMsgBufId(unsigned int buf, long txIdentifier, unsigned int ide, unsigned int remoteTransmit);
Ecan1WriteTxMsgBufData(unsigned int buf, unsigned int dataLength, unsigned int data1, unsigned int data2, unsigned int data3, unsigned int data4);

buf -> Transmit Buffer number

txIdentifier -> SID<10:0> : EID<17:0>

ide = 0 -> Message will transmit standard identifier
ide = 1 -> Message will transmit extended identifier

remoteTransmit = 0 -> Normal message
remoteTransmit = 1 -> Message will request remote transmission

dataLength -> Data length can be from 0 to 8 bytes

data1, data2, data3, data4 -> Data words (2 bytes) each

*/
    Ecan1WriteTxMsgBufId( 0, 0x1FFEFFFF, 1, 0 );
    Ecan1WriteTxMsgBufData( 0, 8, 0x1111, 0x1111, 0x1111, 0x1111 );

    Ecan1WriteTxMsgBufId( 1, 0x1FFEFFFF, 1, 0 );
    Ecan1WriteTxMsgBufData( 1, 8, 0x2222, 0x2222, 0x2222, 0x2222 );

    Ecan1WriteTxMsgBufId( 2, 0x1FFEFFFF, 1, 0 );
    Ecan1WriteTxMsgBufData( 2, 8, 0x3333, 0x3333, 0x3333, 0x3333 );

    Ecan1WriteTxMsgBufId( 3, 0x1FFEFFFF, 1, 0 );
    Ecan1WriteTxMsgBufData( 3, 8, 0x4444, 0x4444, 0x4444, 0x4444 );

    Ecan1WriteTxMsgBufId( 4, 0x1FFEFFFF, 1, 0 );
    Ecan1WriteTxMsgBufData( 4, 8, 0x5555, 0x5555, 0x5555, 0x5555 );

    Ecan1WriteTxMsgBufId( 5, 0x1FFEFFFF, 1, 0 );
    Ecan1WriteTxMsgBufData( 5, 8, 0x6666, 0x6666, 0x6666, 0x6666 );
}

/******************************************************************************
 * Function:        void Ecan2WriteMessage(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is used to write the ECAN2 buffer with the
 *                  identifiers and the data. Writes the message to be
 *                  transmitted.
 *****************************************************************************/
void Ecan2WriteMessage( void )
{
    /* Writing the message for Transmission

Ecan2WriteTxMsgBufId(unsigned int buf, long txIdentifier, unsigned int ide, unsigned int remoteTransmit);
Ecan2WriteTxMsgBufData(unsigned int buf, unsigned int dataLength, unsigned int data1, unsigned int data2, unsigned int data3, unsigned int data4);

buf -> Transmit Buffer Number

txIdentifier -> SID<10:0> : EID<17:0>

ide = 0 -> Message will transmit standard identifier
ide = 1 -> Message will transmit extended identifier

remoteTransmit = 0 -> Normal message
remoteTransmit = 1 -> Message will request remote transmission


dataLength -> Data length can be from 0 to 8 bytes

data1, data2, data3, data4 -> Data words (2 bytes) each


*/
    Ecan2WriteTxMsgBufId( 0, 0x1FFEFFFF, 1, 0 );
    Ecan2WriteTxMsgBufData( 0, 8, 0xAAAA, 0xAAAA, 0xAAAA, 0xAAAA );
    Ecan2WriteTxMsgBufId( 1, 0x1FFEFFFF, 1, 0 );
    Ecan2WriteTxMsgBufData( 1, 8, 0xBBBB, 0xBBBB, 0xBBBB, 0xBBBB );

    Ecan2WriteTxMsgBufId( 2, 0x1FFEFFFF, 1, 0 );
    Ecan2WriteTxMsgBufData( 2, 8, 0xCCCC, 0xCCCC, 0xCCCC, 0xCCCC );

    Ecan2WriteTxMsgBufId( 3, 0x1FFEFFFF, 1, 0 );
    Ecan2WriteTxMsgBufData( 3, 8, 0xDDDD, 0xDDDD, 0xDDDD, 0xDDDD );

    Ecan2WriteTxMsgBufId( 4, 0x1FFEFFFF, 1, 0 );
    Ecan2WriteTxMsgBufData( 4, 8, 0xEEEE, 0xEEEE, 0xEEEE, 0xEEEE );

    Ecan2WriteTxMsgBufId( 5, 0x1FFEFFFF, 1, 0 );
    Ecan2WriteTxMsgBufData( 5, 8, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF );
}

/******************************************************************************
 * Function:        void ClearIntrflags(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:         Clears all the interrupt flag registers.
 *****************************************************************************/
void ClearIntrflags( void )
{
    /* Clear Interrupt Flags */
    IFS0 = 0;
    IFS1 = 0;
    IFS2 = 0;
    IFS3 = 0;
    IFS4 = 0;
}

/******************************************************************************
 * Function:        void OscConfig(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:       This function configures the Oscillator to work at 60MHz.
 *****************************************************************************/
void OscConfig( void )
{
    /*  Configure Oscillator to operate the device at 60Mhz
     Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
     Fosc= 8M*60/(2*2)=120Mhz for 8M input clock */
    PLLFBD = 58;                                /* M=60 */
    CLKDIVbits.PLLPOST = 0;                     /* N1=2 */
    CLKDIVbits.PLLPRE = 0;                      /* N2=2 */
    OSCTUN = 0;                                 /* Tune FRC oscillator, if FRC is used */

    /* Disable Watch Dog Timer */
    RCONbits.SWDTEN = 0;

    /* Clock switch to incorporate PLL*/
    __builtin_write_OSCCONH( 0x03 );            // Initiate Clock Switch to Primary

    // Oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONL( OSCCON || 0x01 );  // Start clock switching
    while( OSCCONbits.COSC != 0b011 );

    // Wait for Clock switch to occur
    /* Wait for PLL to lock */
    while( OSCCONbits.LOCK != 1 )
    { };
}

/******************************************************************************
 * Function:      void __attribute__((interrupt, no_auto_psv))_C1Interrupt(void)
 *
 * PreCondition:  None
 *
 * Input:         None
 *
 * Output:        None
 *
 * Side Effects:  None
 *
 * Overview:      Interrupt service routine to handle CAN1 Transmit and
 *                recieve interrupt.
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _C1Interrupt( void )
{
    IFS2bits.C1IF = 0;      // clear interrupt flag
    if( C1INTFbits.TBIF )
    {
        C1INTFbits.TBIF = 0;
    }

    if( C1INTFbits.RBIF )
    {
        C1INTFbits.RBIF = 0;
    }
}

/******************************************************************************
 * Function:      void __attribute__((interrupt, no_auto_psv))_C2Interrupt(void)
 *
 * PreCondition:  None
 *
 * Input:         None
 *
 * Output:        None
 *
 * Side Effects:  None
 *
 * Overview:      Interrupt service routine to handle CAN2 Transmit and
 *                recieve interrupt.
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _C2Interrupt( void )
{
    IFS3bits.C2IF = 0;      // clear interrupt flag
    if( C2INTFbits.TBIF )
    {
        C2INTFbits.TBIF = 0;
    }

    if( C2INTFbits.RBIF )
    {
        C2INTFbits.RBIF = 0;
    }
}

//------------------------------------------------------------------------------
//    DMA interrupt handlers
//------------------------------------------------------------------------------
/******************************************************************************
 * Function:   void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void)
 *
 * PreCondition:  None
 *
 * Input:         None
 *
 * Output:        None
 *
 * Side Effects:  None
 *
 * Overview:      Interrupt service routine to handle DMA0interrupt
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _DMA0Interrupt( void )
{
    IFS0bits.DMA0IF = 0;    // Clear the DMA0 Interrupt Flag;
}

/******************************************************************************
 * Function:   void __attribute__((interrupt, no_auto_psv)) _DMA1Interrupt(void)
 *
 * PreCondition:  None
 *
 * Input:         None
 *
 * Output:        None
 *
 * Side Effects:  None
 *
 * Overview:      Interrupt service routine to handle DMA1interrupt
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _DMA1Interrupt( void )
{
    IFS0bits.DMA1IF = 0;    // Clear the DMA1 Interrupt Flag;
}

/******************************************************************************
 * Function:   void __attribute__((interrupt, no_auto_psv)) _DMA2Interrupt(void)
 *
 * PreCondition:  None
 *
 * Input:         None
 *
 * Output:        None
 *
 * Side Effects:  None
 *
 * Overview:      Interrupt service routine to handle DMA2interrupt
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _DMA2Interrupt( void )
{
    IFS1bits.DMA2IF = 0;    // Clear the DMA2 Interrupt Flag;
}

/******************************************************************************
 * Function:   void __attribute__((interrupt, no_auto_psv)) _DMA3Interrupt(void)
 *
 * PreCondition:  None
 *
 * Input:         None
 *
 * Output:        None
 *
 * Side Effects:  None
 *
 * Overview:      Interrupt service routine to handle DMA3interrupt
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _DMA3Interrupt( void )
{
    IFS2bits.DMA3IF = 0;    // Clear the DMA3 Interrupt Flag;
}

/*******************************************************************************
 End of File
*/

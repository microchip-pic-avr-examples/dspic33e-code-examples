/*******************************************************************************
  ce437 main source file

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    UART intialization and communication without DMA.

  Description:
    calls UART config functions to intialize and calls SoftwareDebounce 
    functions continuously, if any of button pressed in Explorer16 board a
    charecter is transferred over UART.

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

#if __XC16_VERSION < 1011
#warning "Please upgrade to XC16 v1.11 or newer."
#endif

//-----------------------------------------------------------------------------
//Macros for Configuration Fuse Registers:
//-----------------------------------------------------------------------------
//Invoke macros to set up  device configuration fuse registers.
//The fuses will select the oscillator source, power-up timers, watch-dog
//timers etc. The macros are defined within the device
//header files. The configuration fuse registers reside in Flash memory.
// Configuration fuse definitions
// DSPIC33EP256GP506 Configuration Bit Settings
// 'C' source line config statements
// FICD
#pragma config ICS = PGD1           // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config JTAGEN = OFF         // JTAG Enable bit (JTAG is disabled)

// FPOR
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
#pragma config IESO = OFF           // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FGS
#pragma config GWRP = OFF           // General Segment Write-Protect bit (General Segment may be written)
#pragma config GCP = OFF            // General Segment Code-Protect bit (General Segment Code protect is Disabled)

// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************
#define TRUE    1
#define FALSE   0

uint8_t s3flag, s4flag, s5flag, S6Flag;

/******************************************************************************
 * Function:  void __attribute__ ((interrupt, no_auto_psv)) _U1RXInterrupt(void)
 *
 * PreCondition: UART Module must be Initialized with receive interrupt enabled.
 *
 * Input:        None
 *                  
 * Output:       None
 *
 * Side Effects: None
 *
 * Overview:     UART receive interrupt service routine called whenever a byte
 *               of data received in UART Rx buffer.
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _U1RXInterrupt( void )
{
    LATA = U1RXREG;
    IFS0bits.U1RXIF = 0;
}

/******************************************************************************
 * Function:   void __attribute__ ((interrupt, no_auto_psv)) _U1TXInterrupt(void)
 *
 * PreCondition: UART Module must be Initialized with transmit interrupt enabled.
 *
 * Input:        None
 *                  
 * Output:       None
 *
 * Side Effects: None
 *
 * Overview:     UART transmit interrupt service routine called whenever a data
 *               is sent from UART Tx buffer
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _U1TXInterrupt( void )
{
    IFS0bits.U1TXIF = 0;
}

/******************************************************************************
 * Function:        void InitClock()
 *
 * PreCondition:    None
 *
 * Input:           None
 *                  
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function configures CLOCK
 *****************************************************************************/
void InitClock( void )
{
    PLLFBD = 58;                        // M = 60
    CLKDIVbits.PLLPOST = 0;             // N1 = 2
    CLKDIVbits.PLLPRE = 0;              // N2 = 2
    OSCTUN = 0;
    RCONbits.SWDTEN = 0;

    // Clock switch to incorporate PLL
    __builtin_write_OSCCONH( 0x03 );    // Initiate Clock Switch to

    // External oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONL( OSCCON || 0x01 );  // Start clock switching
    while( OSCCONbits.COSC != 0b011 );

    // Wait for Clock switch to occur
    while( OSCCONbits.LOCK != 1 )
    { };
}

/******************************************************************************
 * Function:        void InitUART2()
 *
 * PreCondition:    None
 *
 * Input:           None
 *                  
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function configures UART module
 *****************************************************************************/
void InitUART2( void )
{
    // This is an EXAMPLE, so brutal typing goes into explaining all bit sets
    // The HPC16 board has a DB9 connector wired to UART2, so we will
    // be configuring this port only
    // configure U1MODE
    U1MODEbits.UARTEN = 0;  // Bit15 TX, RX DISABLED, ENABLE at end of func

    //U1MODEbits.notimplemented;// Bit14
    U1MODEbits.USIDL = 0;   // Bit13 Continue in Idle
    U1MODEbits.IREN = 0;    // Bit12 No IR translation
    U1MODEbits.RTSMD = 0;   // Bit11 Simplex Mode

    //U1MODEbits.notimplemented;// Bit10
    U1MODEbits.UEN = 0;     // Bits8,9 TX,RX enabled, CTS,RTS not
    U1MODEbits.WAKE = 0;    // Bit7 No Wake up (since we don't sleep here)
    U1MODEbits.LPBACK = 0;  // Bit6 No Loop Back
    U1MODEbits.ABAUD = 0;   // Bit5 No Autobaud (would require sending '55')
    U1MODEbits.BRGH = 0;    // Bit3 16 clocks per bit period
    U1MODEbits.PDSEL = 0;   // Bits1,2 8bit, No Parity
    U1MODEbits.STSEL = 0;   // Bit0 One Stop Bit

    // Load a value into Baud Rate Generator.  Example is for 9600.
    // See section 19.3.1 of datasheet.
    //  U1BRG = (Fcy/(16*BaudRate))-1
    //  U1BRG = (37M/(16*9600))-1
    //  U1BRG = 240
    U1BRG = 389;            // 60Mhz osc, 9600 Baud

    // Load all values in for U1STA SFR
    U1STAbits.UTXISEL1 = 0; //Bit15 Int when Char is transferred (1/2 config!)
    U1STAbits.UTXINV = 0;   //Bit14 N/A, IRDA config
    U1STAbits.UTXISEL0 = 0; //Bit13 Other half of Bit15

    //U1STAbits.notimplemented = 0;//Bit12
    U1STAbits.UTXBRK = 0;   //Bit11 Disabled
    U1STAbits.UTXEN = 0;    //Bit10 TX pins controlled by periph
    U1STAbits.UTXBF = 0;    //Bit9 *Read Only Bit*
    U1STAbits.TRMT = 0;     //Bit8 *Read Only bit*
    U1STAbits.URXISEL = 0;  //Bits6,7 Int. on character recieved
    U1STAbits.ADDEN = 0;    //Bit5 Address Detect Disabled
    U1STAbits.RIDLE = 0;    //Bit4 *Read Only Bit*
    U1STAbits.PERR = 0;     //Bit3 *Read Only Bit*
    U1STAbits.FERR = 0;     //Bit2 *Read Only Bit*
    U1STAbits.OERR = 0;     //Bit1 *Read Only Bit*
    U1STAbits.URXDA = 0;    //Bit0 *Read Only Bit*
    IPC7 = 0x4400;          // Mid Range Interrupt Priority level, no urgent reason
    IFS0bits.U1TXIF = 0;    // Clear the Transmit Interrupt Flag
    IEC0bits.U1TXIE = 1;    // Enable Transmit Interrupts
    IFS0bits.U1RXIF = 0;    // Clear the Recieve Interrupt Flag
    IEC0bits.U1RXIE = 1;    // Enable Recieve Interrupts
    RPOR1bits.RP37R = 1;    //RP37/RB5 as U1TX
    RPINR18bits.U1RXR = 38; //RP38/RB6 as U1RX
    U1MODEbits.UARTEN = 1;  // And turn the peripheral on
    U1STAbits.UTXEN = 1;
}

/******************************************************************************
 * Function:        void InitPorts()
 *
 * PreCondition:    None
 *
 * Input:           None
 *                  
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function configures IO ports
 *****************************************************************************/
void InitPorts( void )
{
    // S3 (portD Pin 6, chosen as trigger for sending 'M' to UART)
    // S6 (   RC7 //portD Pin 7, chosen as trigger for sending 'C' to UART)
    // S5 (portC Pin 8, chosen as trigger for sending 'H' to UART)
    // S4 (portD Pin 5, chosen as trigger for sending 'P' to UART)
    ANSELA = 0;
    ANSELC = 0;

    //ANSELD=0;
    // we need to config the pin as DIGITAL
    TRISD = 0x0060;         // D6,D5 inputs
    TRISCbits.TRISC7 = 1;
    TRISCbits.TRISC8 = 1;

    TRISBbits.TRISB5 = 0;   //B4 as output
    TRISBbits.TRISB6 = 1;   //A8 as input
    s3flag = s4flag = s5flag = S6Flag = 0;  // Some Debounce Flags
}

/******************************************************************************
 * Function:        void SoftwareDebounce()
 *
 * PreCondition:    UART module should be initialized 
 *
 * Input:           None
 *                  
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function checks which  button is pressed in Explorer 16
 *                  board.Depending on button pressed different data is written
 *                  to UART TX register to transfer.
 *****************************************************************************/
void SoftwareDebounce( void )
{
    if( PORTDbits.RD6 == FALSE )
    {
        if( s3flag == FALSE )
        {
            s3flag = TRUE;
            U1TXREG = 'M';
        }
    }
    else
    {
        s3flag = FALSE;
    }

    if( PORTCbits.RC7 == FALSE )
    {
        if( S6Flag == FALSE )
        {
            S6Flag = TRUE;
            U1TXREG = 'C';
        }
    }
    else
    {
        S6Flag = FALSE;
    }

    if( PORTCbits.RC8 == FALSE )
    {
        if( s5flag == FALSE )
        {
            s5flag = TRUE;
            U1TXREG = 'H';
        }
    }
    else
    {
        s5flag = FALSE;
    }

    if( PORTDbits.RD5 == FALSE )
    {
        if( s4flag == FALSE )
        {
            s4flag = TRUE;
            U1TXREG = 'P';
        }
    }
    else
    {
        s4flag = FALSE;
    }
}

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
 * Overview:        main function
 *****************************************************************************/
int main( void )
{
    InitClock();    // This is the PLL settings
    InitUART2();    // Initialize UART2 for 9600,8,N,1 TX/RX
    InitPorts();    // LEDs outputs, Switches Inputs
    while( 1 )
    {               // The ever versatile Infinite Loop!
        SoftwareDebounce();
    }
}

/*******************************************************************************
 End of File
*/

/*******************************************************************************
  External Interrupts Source file

  Company:
    Microchip Technology Inc.

  File Name:
    external_interrupts.c

  Summary:
    Generates external Inerrupts at rising or falling edge.

  Description:
    This source file configures the INT0 through INT4 pins as the interrupt sources.
    INT0 is configured to generate an interrupt on the rising edge of the signal on the pin
    while INT1 through INT4 are configured to generate interrupts on the falling edge.
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
// DSPIC33EP512MU810 Configuration Bit Settings
// FICD
#pragma config ICS = PGD1           // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config JTAGEN = OFF         // JTAG Enable bit (JTAG is disabled)

// FPOR
#pragma config BOREN = ON           // Brown-out Reset (BOR) Detection Enable bit (BOR is enabled)
#pragma config ALTI2C1 = OFF        // Alternate I2C1 pins (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2 = OFF        // Alternate I2C2 pins (I2C2 mapped to SDA2/SCL2 pins)

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

/* Global Variables and Functions */
void INTx_IO_Init ( void );
void __attribute__ ( (__interrupt__) )  _INT0Interrupt( void ); /*Declare external interrupt ISRs*/
void __attribute__ ( (__interrupt__) )  _INT1Interrupt( void );
void __attribute__ ( (__interrupt__) )  _INT2Interrupt( void );
void __attribute__ ( (__interrupt__) )  _INT3Interrupt( void );
void __attribute__ ( (__interrupt__) )  _INT4Interrupt( void );

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
 * Overview:        Main function
 *****************************************************************************/
#ifdef TEST_MODE
unsigned int test_flag=0;
int ce_main(void)
#else
int main(void)
#endif
{

#ifdef TEST_MODE
    test_flag=0;
#endif
    // Configure Oscillator to operate the device at 60Mhz
    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
    // Fosc= 8M*60/(2*2)=120Mhz for 8M input clock
    PLLFBD = 58;                                // M=60
    CLKDIVbits.PLLPOST = 0;                     // N1=2
    CLKDIVbits.PLLPRE = 0;                      // N2=2
    OSCTUN = 0;                                 // Tune FRC oscillator, if FRC is used

    // Disable Watch Dog Timer
    RCONbits.SWDTEN = 0;

    // The PPS configuration varies from device to device. Refer the datasheet of the device being used and
    // use the appropriate values for the RPINR/RPOR registers.
    // Configure the Analog functional pins as digital
   // ANSELA = 0xffff;
    ANSELA = 0x0000;
    RPINR0 = 0;
    RPINR0bits.INT1R = 20;                      //RA4
    RPINR1 = 0;
    RPINR1bits.INT2R = 30;                      //RA14
    RPINR1 = 0;
    RPINR1bits.INT3R = 21;                      //RA5
    RPINR2 = 0;
    RPINR2bits.INT4R = 31;                      //RA15
    TRISAbits.TRISA4 = 1;
    TRISAbits.TRISA14 = 1;

    TRISAbits.TRISA5 = 1;
    TRISAbits.TRISA15 = 1;
    TRISAbits.TRISA0 = 0;
    TRISAbits.TRISA7 = 0;


    // Clock switch to incorporate PLL
    __builtin_write_OSCCONH( 0x03 );            // Initiate Clock Switch to External

    // Crystal with PLL (NOSC=0b011)
    __builtin_write_OSCCONL( OSCCON || 0x01 );  // Start clock switching
    while( OSCCONbits.COSC != 0b011 );

    // Wait for Clock switch to occur
    // Wait for PLL to lock
    while( OSCCONbits.LOCK != 1 )
    { };

    _TRISA0 = 0;
    ANSELA = 0x0000;
    ANSELB = 0x0000;
    TRISD = 0x0001; /* LEDs on dsPICDEM 1.1 board are connected to RD0-RD3 */
    /* We will configure Port D to be output so we can use */
    /* use LEDs as an indicator of the occurrence of external */
    /* interrupts */
    INTx_IO_Init(); /* Call function to initialize the External Interrupts */
    while( 1 )
    {
       _LATA0 = 0; //RA0
       _LATA0 = 1;
#ifdef TEST_MODE
// In test mode, while testing connect the pins RA0 and RA4 in order to get external INT1(Expl:16 board mapping)
    if(test_flag ==1)
        break;
        
#endif
    }               /* Loop endlessly...anytime an interrupt occurs */
#ifdef TEST_MODE
    if (test_flag==1)
        return 0;
    else
        return 1;
#endif
    /* the processor will vector to the interrupt and */
    /* return back to the while(1) loop */
}

/******************************************************************************
 * Function:        void INTx_IO_Init(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        INTx_IO_Init() sets up the INT0, INT1, INT2, INT3 & INT4 pins.
                    INT1 - INT4 pins on the device are connected to switches S1 - S4,
 *                  on the dsPICDEM1.1 board.
 *****************************************************************************/
void INTx_IO_Init( void )
{
    INTCON2 = 0x801E;           /*Setup INT1, INT2, INT3 & INT4 pins to interupt */

    /*on falling edge and set up INT0 pin to interupt */
    /*on rising edge */
    IFS0bits.INT0IF = 0;        /*Reset INT0 interrupt flag */
    IEC0bits.INT0IE = 1;        /*Enable INT0 Interrupt Service Routine */

    IFS1bits.INT1IF = 0;        /*Reset INT1 interrupt flag */
    IEC1bits.INT1IE = 1;        /*Enable INT1 Interrupt Service Routine */

    IFS1bits.INT2IF = 0;        /*Reset INT0 interrupt flag */
    IEC1bits.INT2IE = 1;        /*Enable INT0 Interrupt Service Routine */

    IFS3bits.INT3IF = 0;        /*Reset INT1 interrupt flag */
    IEC3bits.INT3IE = 1;        /*Enable INT1 Interrupt Service Routine */

    IFS3bits.INT4IF = 0;        /*Reset INT1 interrupt flag */
    IEC3bits.INT4IE = 1;        /*Enable INT1 Interrupt Service Routine */
}

/******************************************************************************
 * Function:        void __attribute__((interrupt, no_auto_psv)) _INT0Interrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        _INT0Interrupt() is the INT0 interrupt service routine (ISR).
                    The routine must have global scope in order to be an ISR.
                    The ISR name is chosen from the device linker script.
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _INT0Interrupt( void )
{
    IFS0bits.INT0IF = 0;        //Clear the INT0 interrupt flag or else
    //the CPU will keep vectoring back to the ISR
}

/******************************************************************************
 * Function:        void __attribute__((interrupt, no_auto_psv)) _INT1Interrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        _INT0Interrupt() is the INT0 interrupt service routine (ISR).
                    The routine must have global scope in order to be an ISR.
                    The ISR name is chosen from the device linker script.
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _INT1Interrupt( void )
{
    LATA = ( PORTA ^ 0x80 );    //Toggle RA7
    IFS1bits.INT1IF = 0;        //Clear the INT1 interrupt flag or else
    //the CPU will keep vectoring back to the ISR
#ifdef TEST_MODE

    test_flag =1;

#endif
}

/******************************************************************************
 * Function:        void __attribute__((interrupt, no_auto_psv)) _INT2Interrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        _INT2Interrupt() is the INT2 interrupt service routine (ISR).
                    The routine must have global scope in order to be an ISR.
                    The ISR name is chosen from the device linker script.
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _INT2Interrupt( void )
{
    LATA = ( PORTA ^ 0x40 );    //Toggle RA1
    IFS1bits.INT2IF = 0;        //Clear the INT2 interrupt flag or else
    //the CPU will keep vectoring back to the ISR
}

/******************************************************************************
 * Function:        void __attribute__((interrupt, no_auto_psv)) _INT3Interrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        _INT3Interrupt() is the INT3 interrupt service routine (ISR).
                    The routine must have global scope in order to be an ISR.
                    The ISR name is chosen from the device linker script.
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _INT3Interrupt( void )
{
    LATA = ( PORTA ^ 0x02 );  //Toggle RA9
    IFS3bits.INT3IF = 0;        //Clear the INT3 interrupt flag or else
    //the CPU will keep vectoring back to the ISR
}

/******************************************************************************
 * Function:        void __attribute__((interrupt, no_auto_psv)) _INT4Interrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        _INT4Interrupt() is the INT4 interrupt service routine (ISR).
                    The routine must have global scope in order to be an ISR.
                    The ISR name is chosen from the device linker script.
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _INT4Interrupt( void )
{
    LATA = ( PORTA ^ 0x04 );  //Toggle RA10
    IFS3bits.INT4IF = 0;        //Clear the INT4 interrupt flag or else
    //the CPU will keep vectoring back to the ISR
}

/*******************************************************************************
 End of File
 */

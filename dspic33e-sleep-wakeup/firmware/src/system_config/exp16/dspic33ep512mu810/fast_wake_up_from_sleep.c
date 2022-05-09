/*******************************************************************************
  Fast wake up from sleep source file

  Company:
    Microchip Technology Inc.

  File Name:
    fast_wake_up_from_sleep_mode.c

  Summary:
    Wakes up the device from sleep mode.

  Description:
    This source file puts the device in the Sleep mode and waits for an inerrupt on
    the INT1 pin to wake up the device. Port A pins are toggled to indicate the 
    entry to the ISR (interrupt generated) and also serve to measure the time
    taken for the device to wake up from the sleep mode. 
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
#pragma config POSCMD = NONE        // Primary Oscillator Mode Select bits (Primary Oscillator disabled)
#pragma config OSCIOFNC = OFF       // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config IOL1WAY = OFF        // Peripheral pin select configuration (Allow multiple reconfigurations)
#pragma config FCKSM = CSECMD       // Clock Switching Mode bits (Clock switching is enabled,Fail-safe Clock Monitor is disabled)

// FOSCSEL
#pragma config FNOSC = FRC          // Oscillator Source Selection (Internal Fast RC (FRC))
#pragma config IESO = OFF           // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FGS
#pragma config GWRP = OFF           // General Segment Write-Protect bit (General Segment may be written)

/* Global Variables and Functions */
void INT1_Init ( void );
void __attribute__ ( (__interrupt__) )  _INT1Interrupt( void );

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
 ***** main************************************************************************/
int main( void )
{
    // Configure Oscillator to operate the device at 60Mhz
    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
    // Fosc= 7.37M*65(2*2)=120Mhz for 7.37M input clock
    PLLFBD = 63;                                // M=65
    CLKDIVbits.PLLPOST = 0;                     // N1=2
    CLKDIVbits.PLLPRE = 0;                      // N2=2
    OSCTUN = 0;                                 // Tune FRC oscillator, if FRC is used

    // Disable Watch Dog Timer
    RCONbits.SWDTEN = 0;

    // Clock switch to incorporate PLL
    __builtin_write_OSCCONH( 0x01 );            // Initiate Clock Switch to

    // FRC with PLL (NOSC=0b001)
    __builtin_write_OSCCONL( OSCCON || 0x01 );  // Start clock switching
    while( OSCCONbits.COSC != 0b001 );

    // Wait for Clock switch to occur
    // Wait for PLL to lock
    while( OSCCONbits.LOCK != 1 )
    { };

    INT1_Init();    /* Call function to initialize the External Interrupt- INT1 */
    _TRISF4 = 0;
    Sleep();
    while( 1 )      /* Loop endlessly */
    {
        _LATF4 = 0; //RF4
        _LATF4 = 1;

        /* View the time between the falling edge on the INT1 pin*/
        /* and the toggling of Port A pins to estimate the */
        /* time it takes to wake up from SLEEP */
        /*NOTE1: In this project, we set up FOSC so that the device */
        /*operates using the Internal Fast RC oscillator with PLL */
        /*NOTE2: If you need the device to wake up faster, then */
        /*configure the FOSC to use the FRC mode without PLL*/
    }
}

/******************************************************************************
 * Function:        void INT1_Init(void)
 *
 * PreCondition:    None.
 *
 * Input:           None.
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        1.INT1_Init() sets up the INT1 pin

 *****************************************************************************/
void INT1_Init( void )
{
    //S1 connected to INT1
    INTCON2 = 0x8002;           /*Setup INT1 to interupt on falling edge*/
    IFS1bits.INT1IF = 0;        /*Reset INT1 interrupt flag */
    IEC1bits.INT1IE = 1;        /*Enable INT1 Interrupt Service Routine */

    ANSELA = 0x0000;
    TRISA = 0x0000;
    TRISAbits.TRISA4 = 1;
    RPINR0 = 0;
    RPINR0bits.INT1R = 77;      //RD13
}

/******************************************************************************
 * Function:        void __attribute__((interrupt, no_auto_psv)) _INT1Interrupt(void)
 *
 * PreCondition:    INT1 should be initialized.
 *
 * Input:           None.
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        _INT1Interrupt() is the INT1 interrupt service routine (ISR).
                    The routine must have global scope in order to be an ISR.
                    The ISR name is chosen from the device linker script.

 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _INT1Interrupt( void )
{
    LATA = ( LATA ^ 0x0F );    //Toggle RA0, RA1, RA2, RA3
    IFS1bits.INT1IF = 0;        /* Clear the INT1 interrupt flag or else */

    /* the CPU will keep vectoring back to the ISR */
    /* View the time between the falling edge on the INT1 pin*/
    /* and the rising edge of Port A pins to estimate the */
    /* time it takes to wake up from SLEEP */
}

/*******************************************************************************
 End of File
 */

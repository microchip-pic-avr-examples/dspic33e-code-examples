/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    windowed_wdt.c

  Summary:
    This file demonstrates WDT time out.

  Description:
    The WDT is programmed in the windowed mode and the timer is programmed to 
    time out within the window and hence reset the WDT and avoid a device reset.
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
// 'C' source line config statements
// FGS
#pragma config GWRP = OFF       // General Segment Write-Protect bit (General Segment may be written)
#pragma config GSS = OFF        // General Segment Code-Protect bit (General Segment Code protect is disabled)
#pragma config GSSK = OFF       // General Segment Key bits (General Segment Write Protection and Code Protection is Disabled)

// FOSCSEL
#pragma config FNOSC = FRC      // Initial Oscillator Source Selection Bits (Internal Fast RC (FRC))
#pragma config IESO = OFF       // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FOSC
#pragma config POSCMD = XT      // Primary Oscillator Mode Select bits (XT Crystal Oscillator Mode)
#pragma config OSCIOFNC = OFF   // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config IOL1WAY = OFF    // Peripheral pin select configuration (Allow multiple reconfigurations)
#pragma config FCKSM = CSECMD   // Clock Switching Mode bits (Clock switching is enabled,Fail-safe Clock Monitor is disabled)

// FWDT
#pragma config WDTPOST = PS1    // Watchdog Timer Postscaler Bits (1:1)
#pragma config WDTPRE = PR128   // Watchdog Timer Prescaler bit (1:128)
#pragma config PLLKEN = ON      // PLL Lock Wait Enable bit (Clock switch to PLL source will wait until the PLL lock signal is valid.)
#pragma config WINDIS = OFF     // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF     // Watchdog Timer Enable bit (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR128   // Power-on Reset Timer Value Select bits (128ms)
#pragma config BOREN = ON       // Brown-out Reset (BOR) Detection Enable bit (BOR is enabled)
#pragma config ALTI2C1 = OFF    // Alternate I2C pins for I2C1 (SDA1/SCK1 pins are selected as the I/O pins for I2C1)
#pragma config ALTI2C2 = OFF    // Alternate I2C pins for I2C2 (SDA2/SCK2 pins are selected as the I/O pins for I2C2)

// FICD
#pragma config ICS = PGD1       // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config RSTPRI = PF      // Reset Target Vector Select bit (Device will obtain reset instruction from Primary flash)
#pragma config JTAGEN = OFF     // JTAG Enable bit (JTAG is disabled)

// FAS
#pragma config AWRP = OFF       // Auxiliary Segment Write-protect bit (Auxiliary program memory is not write-protected)
#pragma config APL = OFF        // Auxiliary Segment Code-protect bit (Aux Flash Code protect is disabled)
#pragma config APLK = OFF       // Auxiliary Segment Key bits (Aux Flash Write Protection and Code Protection is Disabled)

// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************
// *****************************************************************************
// Section: Function definition
// *****************************************************************************
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
 * Overview:        Sets CPU clock and enables WDT.
 *****************************************************************************/

#ifdef TEST_MODE
unsigned int  test_flag =0;
int ce_main(void)
#else
int main(void)
#endif
{    uint8_t i;

    // Configure Oscillator to operate the device at 60Mhz
    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
    // Fosc= 8M*60/(2*2)=120Mhz for 8M input clock
    PLLFBD = 58;                // M=60
    CLKDIVbits.PLLPOST = 0;     // N1=2
    CLKDIVbits.PLLPRE = 0;      // N2=2
    OSCTUN = 0;                 // Tune FRC oscillator, if FRC is used

    /* Initiate Clock Switch to Primary Oscillator (XT) with PLL (NOSC=0b011)*/
    __builtin_write_OSCCONH( 0x03 );
    __builtin_write_OSCCONL( OSCCON || 0x01 );
    while( OSCCONbits.COSC != 0b011 );

    /* Wait for Clock switch to occur */
    while( !OSCCONbits.LOCK );

    /*Initialize the Ports */
    TRISA = 0xfff0;
    LATA = 0x0000;
    PORTA = 0x0000;

    if( RCONbits.WDTO )
    {
        LATAbits.LATA3 = 1;     //LED D6 glows if system resets due to Watchdog timeout
    }

    T1CONbits.TON = 0;
    T1CONbits.TCKPS = 0b00;    // Set to 0b01 in order to reset the device due to WDT timeout.
    T1CONbits.TCS = 0;
    TMR1 = 0x0000;              // Set Timer1 for time out period of 2ms.
#ifdef TEST_MODE
    PR1 = 0x0FFF;
#else
    PR1 = 0xFFFF;
#endif


    _T1IE = 1;
    _T1IF = 0;
    T1CONbits.TON = 1;

    _SWDTEN = 1;                // Enable WDT by S/W. It is set to time out in 4ms but has a 75%

    //window which makes it important to clear WDT within 3ms.
    i = _WDTO;

#ifdef  TEST_MODE
  test_flag=0;

    while(1)
  {
      LATAbits.LATA2 = 1;
   
      if(test_flag ==1 ) // Timeout has happened and set WDT, so that device does not get reset
		return 0;

 }

      
#else
    if( i == 1 )
    {

        while( 1 );


        // Device has reset due to the time out of WDT.
    }
  while(1);
           
#endif
}

/******************************************************************************
 * Function:void __attribute__ ((__interrupt__, no_auto_psv)) _T1Interrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        ISR for timer interrupt. Clear WDT.
 *****************************************************************************/
void __attribute__ ( (__interrupt__, no_auto_psv) )_T1Interrupt ( void )
{
    _T1IF = 0;

    //LED D4 toggles if system is running and D3 will be off.
    LATAbits.LATA1 = 1;
    ClrWdt();                   // clear the WDT to inhibit the device reset
#ifdef TEST_MODE
    if (_WDTO == 0)
        test_flag=1;
    else
        test_flag=0;
#endif
}

/*******************************************************************************
 End of File
*/

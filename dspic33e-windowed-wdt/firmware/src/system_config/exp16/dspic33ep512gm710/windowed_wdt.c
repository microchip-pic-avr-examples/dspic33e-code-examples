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
// FICD
#pragma config ICS = PGD1               // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)

// FPOR
#pragma config BOREN = ON               // Brown-out Reset (BOR) Detection Enable bit (BOR is enabled)
#pragma config ALTI2C1 = OFF            // Alternate I2C1 pins (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2 = OFF            // Alternate I2C2 pins (I2C2 mapped to SDA2/SCL2 pins)
#ifdef  TEST_MODE
#pragma config WDTWIN = WIN25           // Watchdog Window Select bits (WDT Window is 25% of WDT period)
#else
#pragma config WDTWIN = WIN75           // Watchdog Window Select bits (WDT Window is 75% of WDT period)
#endif

// FWDT
#pragma config WDTPOST = PS1            //PS1            // Watchdog Timer Postscaler bits (1:1)
#pragma config WDTPRE = PR128           // Watchdog Timer Prescaler bit (1:128)
#pragma config PLLKEN = ON              // PLL Lock Enable bit (Clock switch to PLL source will wait until the PLL lock signal is valid.)
#ifdef  TEST_MODE
#pragma config WINDIS = OFF              // Watchdog Timer Window Enable bit (Watchdog Timer in Window mode)
#else
#pragma config WINDIS = ON              // Watchdog Timer Window Enable bit (Watchdog Timer in Window mode)
#endif
#pragma config FWDTEN = OFF             // Watchdog Timer Enable bit (Watchdog timer enabled/disabled by user software)

// FOSC
#pragma config POSCMD = XT              // Primary Oscillator Mode Select bits (XT Crystal Oscillator Mode)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config IOL1WAY = OFF            // Peripheral pin select configuration (Allow multiple reconfigurations)
#pragma config FCKSM = CSECMD           // Clock Switching Mode bits (Clock switching is enabled,Fail-safe Clock Monitor is disabled)

// FOSCSEL
#pragma config FNOSC = FRC              // Oscillator Source Selection (Internal Fast RC (FRC))
#pragma config PWMLOCK = ON             // PWM Lock Enable bit (Certain PWM registers may only be written after key sequence)
#pragma config IESO = OFF               // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FGS
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)
#pragma config GCP = OFF                // General Segment Code-Protect bit (General Segment Code protect is Disabled)

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
{
    uint8_t i;

    // Configure Oscillator to operate the device at 60Mhz
    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
    // Fosc= 8M*60/(2*2)=120Mhz for 8M input clock
    PLLFBD = 58;                        // M=60
    CLKDIVbits.PLLPOST = 0;             // N1=2
    CLKDIVbits.PLLPRE = 0;              // N2=2
    OSCTUN = 0;                         // Tune FRC oscillator, if FRC is used
    __builtin_write_OSCCONH( 0x03 );    /* Initiate Clock Switch to Primary Oscillator (XT) with PLL (NOSC=0b011)*/
    __builtin_write_OSCCONL( OSCCON || 0x01 );
    while( OSCCONbits.COSC != 0b011 );

    /* Wait for Clock switch to occur */
    while( !OSCCONbits.LOCK );

    /*Initialize the Ports */
    TRISG = 0xf3ff;
    LATG = 0x0000;
    PORTG = 0x0000;
    ANSELGbits.ANSG11 = 0;
    ANSELGbits.ANSG10 = 0;

    if( RCONbits.WDTO )
    {
        LATGbits.LATG10 = 1;            //LED D3 glows if system resets due to Watchdog timeout
    }

    T1CONbits.TON = 0;
    T1CONbits.TCKPS = 0b00;            // Set to 0b01 in order to reset the device due to WDT timeout.
    T1CONbits.TCS = 0;
    TMR1 = 0x0000;                      // Set Timer1 for time out period of 2ms.

#ifdef  TEST_MODE
    PR1 = 0x0FFF;
#else
PR1 = 0xFFFF;
#endif

    _T1IE = 1;
    _T1IF = 0;
    T1CONbits.TON = 1;

    _SWDTEN = 1;                        // Enable WDT by S/W. It is set to time out in 4ms but has a 25% window which makes it important to clear WDT within 3ms.
    i = _WDTO;

#ifdef  TEST_MODE
  test_flag=0;
 while(1)
  {
      LATGbits.LATG10 = 1;
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
void __attribute__ ( (__interrupt__, no_auto_psv) ) _T1Interrupt ( void )
{
    _T1IF = 0;

    //LED D4 toggles if system is running and D3 will be off.
    LATGbits.LATG11 =1;
    ClrWdt();   // clear the WDT to inhibit the device reset
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

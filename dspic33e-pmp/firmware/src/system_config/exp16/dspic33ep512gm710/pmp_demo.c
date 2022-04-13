/*******************************************************************************
  ce452 main file
  
  Company:
    Microchip Technology Inc.

  File Name:
    pmp_demo.c

  Summary:
    This file is used to call the timer, LCD, PMP APIs

  Description:
    The main.c includes the header files that have the LCD, timer API 
    declarations and is used to call initialize LCD, timer,PMP and also
    contineously transfers data over PMP to LCD.
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
#include <system.h>

#if __XC16_VERSION == 1011
#warning "XC16 v1.11 detected. It is recommended that a newer version of XC16 be used."
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
#include <xc.h>

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
#pragma config POSCMD = NONE        // Primary Oscillator Mode Select bits (Primary Oscillator disabled)
#pragma config OSCIOFNC = OFF       // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config IOL1WAY = ON         // Peripheral pin select configuration (Allow only one reconfiguration)
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
uint16_t _display_state;

// *****************************************************************************
// Section: Function Prototype
// *****************************************************************************
// *****************************************************************************
extern void BannerStart( void );
extern void BannerProcessEvents( void );

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
 * Overview:        Sets CPU clock.
 *****************************************************************************/
#ifdef TEST_MODE
unsigned char test_flag;
int ce_main(void)
#else
int main(void)
#endif
{

#ifdef TEST_MODE
    test_flag=0;
#endif
    // Setup IOs Ports as digital
    //AD1PCFGL = 0xffff;
    ANSELA = 0;
    ANSELB = 0;
    ANSELC = 0;
    ANSELD = 0;
    ANSELE = 0;
    ANSELG = 0;

    // Setup the LCD
    mLCDInit();

    // Setup the timer
    TimerInit();

    // Setup the banner processing
    BannerStart();

#ifdef TEST_MODE
    while( 1 )
    {
        LCDProcessEvents();

        if( TimerIsOverflowEvent() )
        {
            BannerProcessEvents();
        }
        if (test_flag == 1)
            return 0;
    }   
#else
    while( 1 )
    {
        LCDProcessEvents();

        if( TimerIsOverflowEvent() )
        {
            BannerProcessEvents();
        }
    }   // End of while(1)...
#endif
}       // End of main()...

/*******************************************************************************
 End of File
*/

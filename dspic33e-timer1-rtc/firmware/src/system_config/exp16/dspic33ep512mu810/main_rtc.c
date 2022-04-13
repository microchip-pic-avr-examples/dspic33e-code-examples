/*******************************************************************************
  CE413 main source file

  Company:
    Microchip Technology Inc.

  File Name:
    main_rtc.c

  Summary:
    main function to emulate the real time clock using Timer1.
    
  Description:
    This source file calls the Explorer 16 board LCD routines or the dsPICDEM 1.1 board 
    LCD routines depending on which one of them is used and handles the Timer1 to emulate the
    Real Time Clock. The LCD screen on the board that is used will display a clock that ticks
    every second. 
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
#include "lcd_exp16brd.h"
#include <stdint.h>
#include "common_rtc.h"
#include "delay.h"

#if __XC16_VERSION < 1011
#warning "Please upgrade to XC16 v1.11 or newer."
#endif

// Select the Board
#define __EXPLORER16_BRD__  1       // Explorer 16 Development Board

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
// FGS
#pragma config GWRP = OFF           // General Segment Write-Protect bit (General Segment may be written)
#pragma config GSS = OFF            // General Segment Code-Protect bit (General Segment Code protect is disabled)
#pragma config GSSK = OFF           // General Segment Key bits (General Segment Write Protection and Code Protection is Disabled)

// FOSCSEL
#pragma config FNOSC = FRC          // Initial Oscillator Source Selection Bits (Internal Fast RC (FRC))
#pragma config IESO = OFF           // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FOSC
#pragma config POSCMD = NONE        // Primary Oscillator Mode Select bits (Primary Oscillator disabled)
#pragma config OSCIOFNC = OFF       // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config IOL1WAY = ON         // Peripheral pin select configuration (Allow only one reconfiguration)
#pragma config FCKSM = CSECMD       // Clock Switching Mode bits (Clock switching is enabled,Fail-safe Clock Monitor is disabled)

// FWDT
#pragma config WDTPOST = PS32768    // Watchdog Timer Postscaler Bits (1:32,768)
#pragma config WDTPRE = PR128       // Watchdog Timer Prescaler bit (1:128)
#pragma config PLLKEN = ON          // PLL Lock Wait Enable bit (Clock switch to PLL source will wait until the PLL lock signal is valid.)
#pragma config WINDIS = OFF         // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF         // Watchdog Timer Enable bit (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR128       // Power-on Reset Timer Value Select bits (128ms)
#pragma config BOREN = ON           // Brown-out Reset (BOR) Detection Enable bit (BOR is enabled)
#pragma config ALTI2C1 = OFF        // Alternate I2C pins for I2C1 (SDA1/SCK1 pins are selected as the I/O pins for I2C1)
#pragma config ALTI2C2 = OFF        // Alternate I2C pins for I2C2 (SDA2/SCK2 pins are selected as the I/O pins for I2C2)

// FICD
#pragma config ICS = PGD1           // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config RSTPRI = PF          // Reset Target Vector Select bit (Device will obtain reset instruction from Primary flash)
#pragma config JTAGEN = OFF         // JTAG Enable bit (JTAG is disabled)

// FAS
#pragma config AWRP = OFF           // Auxiliary Segment Write-protect bit (Auxiliary program memory is not write-protected)
#pragma config APL = OFF            // Auxiliary Segment Code-protect bit (Aux Flash Code protect is disabled)
#pragma config APLK = OFF           // Auxiliary Segment Key bits (Aux Flash Write Protection and Code Protection is Disabled)

// *****************************************************************************
// *****************************************************************************
// Section: static Function declarations
// *****************************************************************************
// *****************************************************************************
void Update_LCD ( void );

// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Constants
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
 * Overview:        Sets the operating clock Frequency. Initializes timer1 and LCD.
 *****************************************************************************/
#ifdef TEST_MODE

unsigned int test_count=0;
int ce_main(void)
#else
int main( void )
#endif

{

#ifdef TEST_MODE
    test_count=0;
#endif
    hours, minutes,
        seconds = 0;

    // Configure Oscillator to operate the device at 60Mhz
    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
    // Fosc= 8M*60/(2*2)=120Mhz for 8M input clock
    PLLFBD = 58;                                // M=60
    CLKDIVbits.PLLPOST = 0;                     // N1=2
    CLKDIVbits.PLLPRE = 0;                      // N2=2
    OSCTUN = 0;                                 // Tune FRC oscillator, if FRC is used

    // Disable Watch Dog Timer
    RCONbits.SWDTEN = 0;

    // Clock Switch to incorporate PLL
    __builtin_write_OSCCONH( 0x01 );            // Initiate Clock Switch to

    // FRC with PLL (NOSC=0b001)
    __builtin_write_OSCCONL( OSCCON || 0x01 );  // Start clock switching
    while( OSCCONbits.COSC != 0b001 );

    // Wait for Clock switch to occur
    // Wait for PLL to lock
    while( OSCCONbits.LOCK != 1 )
    { };

    /* Initialize some general use variables */
    rtc_Lcd_Update = 0;

    /* set LED0 pins as outputs */
    TRISDbits.TRISD1 = 0;

    /* Initialize LCD */
    Init_LCD();
    home_clr();

    Puts_LCD( ( uint8_t * ) "Real Time Clock", 15 );
    line_2();
    Puts_LCD( ( uint8_t * ) "00 : 00 : 00 ", 12 );

    Delay( 1000 );

    /* Initialize Timer 1 for 32KHz real-time clock operation */
    Init_Timer1();

    while( 1 )
    {

#ifdef TEST_MODE
          if( rtc_Lcd_Update )
          {
            test_count++;
            if(test_count == 2)
                return 0;
          }
#endif
        if( rtc_Lcd_Update )
        {

            Update_LCD();
            rtc_Lcd_Update = 0;
        }
    };
    return ( 0 );
}

/******************************************************************************
 * Function:        Update_LCD
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:       Update LCD for real-time clock data
 *****************************************************************************/
void Update_LCD( void )
{
    HexDec( hours );
    line_2();
    LCD_Data( tens + 0x30 );
    LCD_Data( ones + 0x30 );
    Puts_LCD( ( uint8_t * ) " : ", 3 );

    HexDec( minutes );
    LCD_Data( tens + 0x30 );
    LCD_Data( ones + 0x30 );
    Puts_LCD( ( uint8_t * ) " : ", 3 );

    HexDec( seconds );
    LCD_Data( tens + 0x30 );
    LCD_Data( ones + 0x30 );
}

/*******************************************************************************
 End of File
*/

/*******************************************************************************
  ce404 main file

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    Calls the clockSwitch function 

  Description:
    In this main.c file, the clockswitch routine is called if the clock switch enable 
    flag is '1' and the current system frequency at which the controller is running
    is changed to 1/8 of its initial value. So, the port pin toggling frequency also
    gets changed to the new system frequency thus confirming a clean switch.
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
#include "clock_switch.h"
#include "ctglpin.h"
#include <stdint.h>

#if __XC16_VERSION < 1011
#warning "Please upgrade to XC16 v1.11 or newer."
#endif

// DSPIC33EP256GP506 Configuration Bit Settings
#include <xc.h>

// FGS
#pragma config GWRP = OFF           // General Segment Write-Protect bit (General Segment may be written)

// FOSCSEL
#pragma config FNOSC = PRI          // Initial Oscillator Source Selection bits (Primary Oscillator (XT, HS, EC))
#pragma config IESO = OFF           // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FOSC
#pragma config POSCMD = XT          // Primary Oscillator Mode Select bits (XT Crystal Oscillator Mode)
#pragma config OSCIOFNC = OFF       // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config IOL1WAY = OFF        // Peripheral pin select configuration (Allow multiple reconfigurations)
#pragma config FCKSM = CSECMD       // Clock Switching Mode bits (Clock switching is enabled,Fail-safe Clock Monitor is disabled)

// FWDT
#pragma config WDTPOST = PS32768    // Watchdog Timer Postscaler bits (1:32,768)
#pragma config WDTPRE = PR128       // Watchdog Timer Prescaler bit (1:128)
#pragma config PLLKEN = ON          // PLL Lock Wait Enable bit (Clock switch to PLL source will wait until the PLL lock signal is valid.)
#pragma config WINDIS = OFF         // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF         // Watchdog Timer Enable bit (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config ALTI2C1 = OFF        // Alternate I2C pins for I2C1 (SDA1/SCK1 pins are selected as the I/O pins for I2C1)
#pragma config ALTI2C2 = OFF        // Alternate I2C pins for I2C2 (SDA2/SCK2 pins are selected as the I/O pins for I2C2)

// FICD
#pragma config ICS = PGD1           // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config JTAGEN = OFF         // JTAG Enable bit (JTAG is disabled)

// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************
// Clock switching flag
uint16_t clkSwEnable = 1;

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
unsigned char test_flag=0;
// FRC Swithcing  NOSC value
#define FRC_CLOCK_SWITCH_CHECK  0x0700;
unsigned int temp_val=0xFF;
int ce_main(void)
#else
int main(void)
#endif

{
#ifdef TEST_MODE
 test_flag=0;
#endif
    // Disable Watch Dog Timer
    RCONbits.SWDTEN = 0;

    // Wait for PLL to lock
    //    while(OSCCONbits.LOCK!=1) {};
    // Continuous Pin Toggle Init
    CtglPinInit();

    // Clock Switch
    if( clkSwEnable == 0 )
    {
        // Pin will be toggling at 1/8 of system clock frequency.
        CtglPin();                  // No clock switch
    }
    else
    {

        ClockSwitch( NOSC_FRC );    // Clock is switched to internal FRC

#ifdef TEST_MODE
        temp_val = (unsigned int)OSCCON ;
        temp_val = (unsigned int)temp_val & FRC_CLOCK_SWITCH_CHECK;
        if(temp_val == 0)
        {
            test_flag=0;
            return test_flag;
        }
        else
        {
            test_flag=1;
            return test_flag;
        }


#endif
        // Pin will be toggling at 1/8 of system clock frequency.
        CtglPin();
    }

    return ( 0 );
}

/*******************************************************************************
 End of File
 */

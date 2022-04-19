/*******************************************************************************
  ce419 main source file

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    Carries out the read and write operations on EEPROM using I2C.

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
#include "i2c_emem.h"

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
// Instantiate Drive and Data objects
I2CEMEM_DRV i2cmem = I2CSEMEM_DRV_DEFAULTS;
I2CEMEM_DATA    wData;
I2CEMEM_DATA    rData;

uint16_t        wBuff[10], rBuff[10];
uint16_t        enable;
extern uint16_t jDone;

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
 * Overview:       Sets operating clock frequency and initializes I2C.
 *****************************************************************************/
int main( void )
{
    int i = 0;

    // Configure Oscillator to operate the device at 40Mhz
    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
    // Fosc= 8M*40/(2*2)=80Mhz for 8M input clock
    PLLFBD = 38;                        // M=40
    CLKDIVbits.PLLPOST = 0;             // N1=2
    CLKDIVbits.PLLPRE = 0;              // N2=2
    OSCTUN = 0;                         // Tune FRC oscillator, if FRC is used

    // Disable Watch Dog Timer
    RCONbits.SWDTEN = 0;

    // Make all the ANx pins as digital pins
    ANSELC = 0;

    // Clock switch to incorporate PLL
    __builtin_write_OSCCONH( 0x03 );    // Initiate Clock Switch to Primary

    // Oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONL( 0x01 );    // Start clock switching
    while( OSCCONbits.COSC != 0b011 );

    // Wait for Clock switch to occur
    // Wait for PLL to lock
    while( OSCCONbits.LOCK != 1 )
    { };

    // Initialise I2C peripheral and Driver
    i2cmem.init( &i2cmem );

    // Initialise Data to be written to serial EEPROM
    for( i = 0; i < 10; i++ )
    {
        wBuff[i] = i + 54;
    }

    // Initialise I2C Data object for Write operation
    wData.buff = wBuff;
    wData.n = 1;                        //10;
    wData.addr = 0x00;
    wData.csel = 0x00;

    // Initialise I2C Data Object for Read operation
    rData.buff = rBuff;
    rData.n = 1;                        //10;
    rData.addr = 0x00;
    rData.csel = 0x00;

    // Enable data write to I2C serial EEPROM
    enable = 1;

    while( 1 )
    {
        if( enable == 1 )
        {
            enable = 0;

            // Write Data
            i2cmem.oData = &wData;
            i2cmem.cmd = I2C_WRITE;     /* comment this line to omit the write phase */

            while( i2cmem.cmd != I2C_IDLE )
            {
                i2cmem.tick( &i2cmem );
            }

            // Read Data
            i2cmem.oData = &rData;
            i2cmem.cmd = I2C_READ;      /* comment this line to omit the read phase */

            while( i2cmem.cmd != I2C_IDLE )
            {
                i2cmem.tick( &i2cmem );
            }
        }
    };
}

/*******************************************************************************
 End of File
*/

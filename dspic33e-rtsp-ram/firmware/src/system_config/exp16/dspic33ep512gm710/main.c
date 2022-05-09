/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This file is used to call the flash API

  Description:
    The main.c includes the flash API's for Erase and Write operations. The flash write in both compressed and
        uncompressed formats are used as a demonstration.
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

#include <stdint.h>                 /* Includes uint16_t definition                    */
#include <stdbool.h>                /* Includes true/false definition                  */
#ifdef TEST_MODE
#include "../../../rtsp_api.h"
#endif

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
#define NO_ERROR            0
#define WRERR_OR_UERR_ERROR 1

// Function prototypes
uint8_t PageErase( uint32_t address );

uint8_t     RowWrite_Compressed( uint32_t flashAddress, uint16_t ramBuffer[] );

uint8_t     RowWrite_Uncompressed( uint32_t flashAddress, uint16_t ramBuffer[] );

// Four 16-bit RAM locations for every two Instruction Words programmed
// For a TLAK 64-IW row this takes 128, 16-bit data RAM elements
uint16_t    ramArray_Uncompressed[] = { 0xBBCC, /*  <=== 0x{LSW0}   */ 0x00AA, /*  <=== 0x00{MSB0} */ 0xEEFF, /*  <=== 0x{LSW1}   */ 0x00DD, /*  <=== 0x00{MSB1} */ 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD, 0xBBCC, 0x00AA, 0xEEFF, 0x00DD };

// Three 16-bit RAM locations for every two Instruction Words programmed
// For a TLAK 64-IW row this takes 96, 16-bit data RAM elements
uint16_t    ramArray_Compressed[] = { 0x2233, /*  <=== 0x{LSW0}       */ 0x4411, /*  <=== 0x{MSB1}{MSB0} */ 0x5566, /*  <=== 0x{LSW1}       */ 0x2233, 0x4411, 0x5566, 0x2233, 0x4411, 0x5566, 0x2233, 0x4411, 0x5566, 0x2233, 0x4411, 0x5566, 0x2233, 0x4411, 0x5566, 0x2233, 0x4411, 0x5566, 0x2233, 0x4411, 0x5566, 0x2233, 0x4411, 0x5566, 0x2233, 0x4411, 0x5566, 0x2233, 0x4411, 0x5566, 0x2233, 0x4411, 0x5566, 0x2233, 0x4411, 0x5566, 0x2233, 0x4411, 0x5566, 0x2233, 0x4411, 0x5566, 0x2233, 0x4411, 0x5566, 0x2233, 0x4411, 0x5566, 0x2233, 0x4411, 0x5566, 0x2233, 0x4411, 0x5566, 0x2233, 0x4411, 0x5566, 0x2233, 0x4411, 0x5566, 0x2233, 0x4411, 0x5566, 0x2233, 0x4411, 0x5566, 0x2233, 0x4411, 0x5566, 0x2233, 0x4411, 0x5566, 0x2233, 0x4411, 0x5566, 0x2233, 0x4411, 0x5566, 0x2233, 0x4411, 0x5566, 0x2233, 0x4411, 0x5566, 0x2233, 0x4411, 0x5566, 0x2233, 0x4411, 0x5566, 0x2233, 0x4411, 0x5566 };

const uint16_t __attribute__ ( (space(prog), address(0x2000)) ) flashArray1[] =
{
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0001,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0002,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0003,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0004,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0005,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0006,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0007,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0008,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009,
    0x0009
};

#ifdef TEST_MODE
uint16_t    nvmAdr, nvmAdru, nvmAdrPageAligned, nvmRow, nvmSize;
int16_t     pageTestBuff[128];
#endif
/******************************************************************************
 * Function:        int16_t main(void)
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
unsigned char test_flag;
#endif
#ifdef TEST_MODE
int ce_main(void)
#else
int main(void)
#endif
{
    uint32_t    softwareDelay = 0;
    uint32_t    flashAddress = 0;
    uint8_t     returnCode = NO_ERROR;
#ifdef TEST_MODE
    int16_t temp;
    uint8_t i=0;
    test_flag=0;
#endif
    // Software delay
    // Prevent code from running during the programming process if test mode is successively entered/exited.
    for( softwareDelay = 0; softwareDelay < 0x10000; softwareDelay++ )
    {
        ;
    }

    Nop();
    Nop();
    Nop();

    // Erase 1 row at flash address 0x2000
    flashAddress = 0x2000;
    returnCode = PageErase( flashAddress );

    // Loop forever if an error is detected
    if( returnCode != NO_ERROR )
    {
        while( 1 )
        {
            Nop();
            Nop();
            Nop();
        }
    }

    // Erase 1 row at flash address 0x2400
    flashAddress = 0x2400;
    returnCode = PageErase( flashAddress );

    // Loop forever if an error is detected
    if( returnCode != NO_ERROR )
    {
        while( 1 )
        {
            Nop();
            Nop();
            Nop();
        }
    }

    // Perform a 64-IW row write at flash address  0x2000: using COMPRESSED RAM format
    flashAddress = 0x2400;
    returnCode = RowWrite_Compressed( flashAddress, ramArray_Compressed );

    // Loop forever if an error is detected
    if( returnCode != NO_ERROR )
    {
        while( 1 )
        {
            Nop();
            Nop();
            Nop();
        }
    }

    // Perform a 64-IW row write at flash address  0x2080: using UNCOPMPRESSED RAM format
    flashAddress = 0x2080;
    returnCode = RowWrite_Uncompressed( flashAddress, ramArray_Uncompressed );

    // Loop forever if an error is detected
    if( returnCode != NO_ERROR )
    {
        while( 1 )
        {
            Nop();
            Nop();
            Nop();
        }
    }

    Nop();
    Nop();
    Nop();
#ifdef TEST_MODE
    nvmAdru = __builtin_tblpage( flashArray1 );
    nvmAdr = __builtin_tbloffset( flashArray1 );
    nvmAdrPageAligned = nvmAdr & 0xFC00;    // Get the Flash Page Aligned address
    nvmRow = 0; // Specify the Flash Row to be modified within the page
    nvmSize = 128;

          // Read the page and place the data into pageMirrorBuf array
    temp = FlashPageRead( nvmAdru, 0x2080/* Location to read into the buffer*/, pageTestBuff );
        for (i=0;i<128;)
    {
        if (   (pageTestBuff[i]   == 0xBBCC) &&
               (pageTestBuff[i+1] == 0x00AA) &&
               (pageTestBuff[i+2] == 0xEEFF) &&        
               (pageTestBuff[i+3] == 0x00DD) 
             )
            test_flag=1;
        else
            test_flag=0;

        i=i+4;

     }

     while(1)
      {
             if(test_flag ==1)
                 return 0;
      }
#else
    while( 1 )
    {
        Nop();
        Nop();
        Nop();
    }
#endif
}

/******************************************************************************
 * Function:        uint8_t PageErase( uint32_t flashAddress )
 *
 * PreCondition:    None
 *
 * Input:           uint32_t flashAddress
 *
 * Output:          True/False
 *
 * Side Effects:    None
 *
 * Overview:        This function is used to erase one page of program memory
 *                  starting from the address specified in the parameter.
 *****************************************************************************/
uint8_t PageErase( uint32_t flashAddress )
{
    // Set up the NVMADR registers to the starting address of the page
    NVMADR = ( flashAddress & 0xFFFF );
    NVMADRU = ( (flashAddress >> 16) & 0xFFFF );

    // Set up NVMCON to erase one page of Program Memory
    NVMCON = 0x4003;

    // Disable interrupts < priority 7 for next 5 instructions; Assumes no level 7 peripheral interrupts
    __builtin_disi( 6 );

    // Write the key sequence and set the 'WR' bit
    __builtin_write_NVM();

    // This is the code behind '__builtin_write_NVM()':
    //mov #0x55, Wn
    //mov Wn, _NVMKEY
    //mov #0xAA, Wn
    //mov Wn, _NVMKEY
    //bset _NVMCON, #15
    //nop
    //nop
    // Some basic error-checking to bubble-up to the 'CALLER':
    if( (NVMCONbits.WRERR == 1) || (NVMCONbits.URERR == 1) )
    {
        return ( WRERR_OR_UERR_ERROR );
    }
    else
    {
        return ( NO_ERROR );
    }
}

/******************************************************************************
 * Function:        uint8_t RowWrite_Compressed( uint32_t flashAddress,
                             uint16_t ramBuffer[] )
 *
 * PreCondition:    None
 *
 * Input:           uint32_t flashAddress, uint16_t ramBuffer[]
 *
 * Output:          True/False
 *
 * Side Effects:    None
 *
 * Overview:         This function is used to write to one row of flash when the data being supplied from RAM is in
                     the compressed format.
 *****************************************************************************/
uint8_t RowWrite_Compressed( uint32_t flashAddress, uint16_t ramBuffer[] )
{
    // Set up the NVMADR registers to the starting address of the page
    NVMADR = ( flashAddress & 0xFFFF );
    NVMADRU = ( (flashAddress >> 16) & 0xFFFF );

    // Set up the NVMSRCADR registers to the starting
    // address in RAM where the data (to be programmed into flash)is stored
    NVMSRCADRL = ( int16_t ) ramBuffer;
    NVMSRCADRH = 0;

    //NVMSRCADRL = (uint16_t) ( ( &ramBuffer) & 0xFFFF );
    //NVMSRCADRH = (uint16_t) ( ( ( &ramBuffer) >> 16 ) & 0xFFFF );
    // Set up NVMCON to write one row of Program Memory
    // with RAM data in compressed format
    NVMCON = 0x4202;

    // Disable interrupts < priority 7 for next 5 instructions; Assumes no level 7 peripheral interrupts
    __builtin_disi( 6 );

    // Write the key sequence and set the 'WR' bit
    __builtin_write_NVM();

    // This is the code behind '__builtin_write_NVM()':
    //mov #0x55, Wn
    //mov Wn, _NVMKEY
    //mov #0xAA, Wn
    //mov Wn, _NVMKEY
    //bset _NVMCON, #15
    //nop
    //nop
    // Some basic error-checking to bubble-up to the 'CALLER':
    if( (NVMCONbits.WRERR == 1) || (NVMCONbits.URERR == 1) )
    {
        return ( WRERR_OR_UERR_ERROR );
    }
    else
    {
        return ( NO_ERROR );
    }
}

/******************************************************************************
 * Function:        uint8_t RowWrite_Uncompressed( uint32_t flashAddress,
                               uint16_t ramBuffer[] )
 *
 * PreCondition:    None
 *
 * Input:           uint32_t flashAddress, uint16_t ramBuffer[]
 *
 * Output:          True/False
 *
 * Side Effects:    None
 *
 * Overview:        This function is used to write to one page of flash program memory when the data from RAM is
                    being fed in the uncompressed format.
 *****************************************************************************/
uint8_t RowWrite_Uncompressed( uint32_t flashAddress, uint16_t ramBuffer[] )
{
    // Set up the NVMADR registers to the starting address of the page
    NVMADR = ( flashAddress & 0xFFFF );
    NVMADRU = ( (flashAddress >> 16) & 0xFFFF );

    // Set up the NVMSRCADR registers to the starting
    // address in RAM where the data (to be programmed into flash)is stored
    NVMSRCADRL = ( int16_t ) ramBuffer;
    NVMSRCADRH = 0;

    //NVMSRCADRL = (uint16_t) ( ( &ramBuffer) & 0xFFFF );
    //NVMSRCADRH = (uint16_t) ( ( ( &ramBuffer) >> 16 ) & 0xFFFF );
    // Set up NVMCON to write one row of Program Memory
    // with RAM data in uncompressed format
    NVMCON = 0x4002;

    // Disable interrupts < priority 7 for next 5 instructions; Assumes no level 7 peripheral interrupts
    __builtin_disi( 6 );

    // Write the key sequence and set the 'WR' bit
    __builtin_write_NVM();

    // This is the code behind '__builtin_write_NVM()':
    //mov #0x55, Wn
    //mov Wn, _NVMKEY
    //mov #0xAA, Wn
    //mov Wn, _NVMKEY
    //bset _NVMCON, #15
    //nop
    //nop
    // Some basic error-checking to bubble-up to the 'CALLER':
    if( (NVMCONbits.WRERR == 1) || (NVMCONbits.URERR == 1) )
    {
        return ( WRERR_OR_UERR_ERROR );
    }
    else
    {
        return ( NO_ERROR );
    }
}

/*******************************************************************************
 End of File
 */

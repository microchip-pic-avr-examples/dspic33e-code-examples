/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This file is used to call the flash API

  Description:
    The main.c includes the header files that have the flash API declarations
    and is used to call the RTSP flash APIs for Erase, Read and Write operations.
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
#include "rtsp_api.h"
#include "testdata.h"
#include <stdint.h>

#if __XC16_VERSION == 1011
#warning "XC16 v1.11 detected. It is recommended that a newer version of XC16 be used."
#endif

// *****************************************************************************
// *****************************************************************************
// Section: Macros for Configuration Fuse Registers:
// *****************************************************************************
// *****************************************************************************
//Invoke macros to set up  device configuration fuse registers.
//The fuses will select the oscillator source, power-up timers, watch-dog
//timers etc. The macros are defined within the device
//header files. The configuration fuse registers reside in Flash memory.
// DSPIC33EP512MU810 Configuration Bit Settings
// FGS
#pragma config GWRP = OFF           // General Segment Write-Protect bit (General Segment may be written)
#pragma config GSS = OFF            // General Segment Code-Protect bit (General Segment Code protect is disabled)
#pragma config GSSK = OFF           // General Segment Key bits (General Segment Write Protection and Code Protection is Disabled)

// FOSCSEL
#pragma config FNOSC = FRC          // Initial Oscillator Source Selection Bits (Internal Fast RC (FRC))
#pragma config IESO = OFF           // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FOSC
#pragma config POSCMD = XT          // Primary Oscillator Mode Select bits (XT Crystal Oscillator Mode)
#pragma config OSCIOFNC = OFF       // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config IOL1WAY = OFF        // Peripheral pin select configuration (Allow multiple reconfigurations)
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
#pragma config ALTI2C1 = ON         // Alternate I2C pins for I2C1 (ASDA1/ASCK1 pins are selected as the I/O pins for I2C1)
#pragma config ALTI2C2 = ON         // Alternate I2C pins for I2C2 (ASDA2/ASCK2 pins are selected as the I/O pins for I2C2)

// FICD
#pragma config ICS = PGD1           // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config RSTPRI = PF          // Reset Target Vector Select bit (Device will obtain reset instruction from Primary flash)
#pragma config JTAGEN = OFF         // JTAG Enable bit (JTAG is disabled)

// FAS
#pragma config AWRP = OFF           // Auxiliary Segment Write-protect bit (Auxiliary program memory is not write-protected)
#pragma config APL = OFF            // Auxiliary Segment Code-protect bit (Aux Flash Code protect is disabled)
#pragma config APLK = OFF           // Auxiliary Segment Key bits (Aux Flash Write Protection and Code Protection is Disabled)

//-----------------------------------------------------------------------------
// Intermediate buffer to check the result before and after writing on to Aux flash
int16_t pageMirrorBuff[128 * 8];

// RTSP variable
uint16_t    nvmAdr, nvmAdru, nvmAdrPageAligned, nvmRow, nvmSize;

// Data to be written to Flash
int16_t     MyRowDataInRam[128] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127 };
#ifdef TEST_MODE
int16_t testPageMirrorBuff[128] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96, 97, 98, 99, 100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127 };
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

#ifdef TEST_MODE
    int16_t j;
    test_flag=0;
#endif

    int16_t i;

    int16_t temp;

    // Configure Oscillator to operate the device at 60Mhz
    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
    // Fosc= 8M*60/(2*2)=120Mhz for 8M input clock
    PLLFBD = 58;                            // M=60
    CLKDIVbits.PLLPOST = 0;                 // N1=2
    CLKDIVbits.PLLPRE = 0;                  // N2=2
    OSCTUN = 0;                             // Tune FRC oscillator, if FRC is used

    // Disable Watch Dog Timer
    RCONbits.SWDTEN = 0;

    // Clock switch to incorporate PLL
    __builtin_write_OSCCONH( 0x03 );        // Initiate Clock Switch to Primary Oscillator with PLL
    __builtin_write_OSCCONL( 0x01 );        // Start clock switching
    while( OSCCONbits.COSC != 0b011 );

    // Wait for Clock switch to occur
    // Wait for PLL to lock
    while( OSCCONbits.LOCK != 1 )
    { };

    nvmAdru = __builtin_tblpage( myRowDataInFlash ) & 0x00FF;
    nvmAdr = __builtin_tbloffset( myRowDataInFlash );
    nvmAdrPageAligned = nvmAdr & 0xF800;    // Get the Flash Page Aligned address
    nvmRow = 7; // Specify the Flash Row to be modified within the page
    nvmSize = 128;

    // Read the page and place the data into pageMirrorBuf array
    temp = FlashPageRead( nvmAdru, nvmAdrPageAligned, pageMirrorBuff );

    // Initialize data buffer
    for( i = 0; i < 128; i++ )
    {
        MyRowDataInRam[i] = i;
    }

    // Modify the pageMirrorBuf array
    temp = FlashPageModify( nvmRow, nvmSize, MyRowDataInRam, pageMirrorBuff );

    // Erase the page in Auxiliary Flash
    temp = FlashPageErase( nvmAdru, nvmAdrPageAligned );

    // Program the page with modified data
    temp = FlashPageWrite( nvmAdru, nvmAdrPageAligned, pageMirrorBuff );

    // Clear Page Mirror Buffer
    for( i = 0; i < (128 * 8); i++ )
    {
        pageMirrorBuff[i] = 0;
    }

    // Read the page and place the data into pageMirrorBuf array
    temp = FlashPageRead( nvmAdru, nvmAdrPageAligned, pageMirrorBuff );

  #ifdef TEST_MODE

    for( i=0,j=896; i<128; )
    {
        if(pageMirrorBuff[j++] == testPageMirrorBuff[i++])
            test_flag=1;
        else
        { test_flag=0;
            break;
        }
    }
    if(test_flag ==1)
        return 0;
    else
        return 1;

#else
    while( 1 );
#endif
}

/*******************************************************************************
 End of File
 */

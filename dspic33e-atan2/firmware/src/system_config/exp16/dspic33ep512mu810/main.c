/*******************************************************************************
  ce440 main file

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    Calls Atan2CORDIC function

  Description:
    This file has the main function that calls atan2cordic function to perform inverse tangent
    operation on a range of angles.
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
#include <math.h>
#include <dsp.h>

#if __XC16_VERSION == 1011
#warning "XC16 v1.11 detected. It is recommended that a newer version of XC16 be used."
#endif

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

// *****************************************************************************
// *****************************************************************************
// Section: Macros for Configuration Fuse Registers:
// *****************************************************************************
// *****************************************************************************

/// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************
// angular increment in radians
#define DELTA_ANGLE ( 0.01 )

// definition of assembly function in Atan2CORDIC.s
fractional Atan2CORDIC( fractional y, fractional x );

///////////////////////////////////////////////////////////////////
//
//  main() test rig harness for the assembly CORDIC routine
//  For a range of input angles calculate the x and y fractional
//	values corresponding to this angle. Then calculate the
//	return value from the C standard atan2 function and also
//	from the modifiy atan2CORDIC routine, print the result and error
//
///////////////////////////////////////////////////////////////////
double      x, y;                   //variables used to hold the cosine and sine values for a given theta
double      cFunc;                  //variable used to hold the value of atan2 function
double      theta;                  // given angle in radian
double      theta1;                 // angle calculated by the assembly function
double      err;                    // Difference between atan2 and assembly function calculation of inverse tan
fractional  fx, fy;
fractional  fRes;

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
 ************************************************************************/
#ifdef TEST_MODE
int ce_main(void)
{
#else
int main(void)
{
#endif

    for( theta = -PI; theta < PI; theta += DELTA_ANGLE )
    {
        //printf("%f, ", theta);
        // calculate the x and y values for this angle
        x = cos( theta );
        y = sin( theta );

        //printf("%f, %f, ", x, y);
        // calculate the angle using the library routine
        cFunc = atan2( y, x );

        //printf("%f, ", cFunc);
        // Atan2CORDIC only accepts dsPIC fractionals
        fx = Float2Fract( x );
        fy = Float2Fract( y );
        fRes = Atan2CORDIC( fy, fx );

        // the return value is a dsPIC fractional from -1.0 to 0.9999
        // representing the range -PI to PI, so scale the result
        theta1 = Fract2Float( fRes );
        theta1 *= PI;

        //printf("%f, ", theta1);
        // calculate the error in degrees
        err = ( 180.0 / PI ) * ( cFunc - theta1 );
    
    }
#ifdef TEST_MODE
        if((signed int)cFunc == (signed int)theta1)
            return 0;
        else
            return 1;
#endif
    
#ifndef TEST_MODE
    while( 1 );
    return ( 0 );
#endif
 }

/*******************************************************************************
 End of File
 */

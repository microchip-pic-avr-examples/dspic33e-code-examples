/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    edc_manual.c

  Summary:
    This file is used demonstrate how automatically(compiler) and manually EDS registers can be managed.

  Description:
                The file defines 3 buffers in EDS, and then compute vector dot product of two array saved.
                'm' has been defined as a compiler-managed EDS variable,  i.e. the EDS registers are automatically
                managed by the compiler without requiring the user code to modify or save/restore the contents of
                the EDS registers. 'm1' and 'm2' have been defined as EDS variables that the user application needs
                to manage manually  i.e. the compiler will not modify or save/restore the value of the DSRPAG
                registers.
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

#if __XC16_VERSION == 1011
#warning "XC16 v1.11 detected. It is recommended that a newer version of XC16 be used."
#endif

// DSPIC33EP512GM710 Configuration Bit Settings
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
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************
//3 data arrays are defined to be stored in Extended Data Space (EDS)
//'x' is a non-EDS array
__eds__ int m[5] = { 1, 2, 3, 4, 5 };
int __attribute__ ( (space(xmemory), eds) ) m1[5] = { 2, 4, 6, 8, 10 };
int __attribute__ ( (space(ymemory), eds) ) m2[5] = { 3, 6, 9, 12, 15 };
int         x[5] = { 10, 20, 30, 40, 50 };
__eds__ int sum[5];
int __attribute__ ( (space(xmemory), eds) ) sum1[5];
int __attribute__ ( (space(ymemory), eds) ) sum2[5];

#ifdef TEST_MODE

// The below values are calculated as per the above array declaration. These values are reference values to check
// whether the proper multiplication happens by accessing from the EDS memory.
// The below values should be re-calculated if the above array values are modified/updated.
unsigned int testSum1 = 1100; // sum1 reference value after actual calculation
unsigned int testSum2 = 1650; // sum2 reference value after actual calculation
unsigned int testSum = 550; // sum reference value after actual calculation
unsigned int tempsum=0;

#endif
//Function prototype
void    vectorMultiply( __eds__ int *, int *, __eds__ int * );

// *****************************************************************************
// main function
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
 * Overview:        Main function
 *****************************************************************************/
#ifdef TEST_MODE
unsigned int  test_flag1 = 0, test_flag2=0, test_flag3=0;
int ce_main(void)
#else
int main(void)
#endif
{

#ifdef TEST_MODE
    test_flag1=0; // First calculation tracking flag
    test_flag2=0; // Second calculation tracking flag
    test_flag3=0; // Third calculation tracking flag
    tempsum=0;
#endif
    int temp1;

    int temp2;

    // Configure Oscillator to operate the device at 60Mhz
    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
    // Fosc= 8M*60/(2*2)=120Mhz for 8M input clock
    PLLFBD = 58;                                // M=60
    CLKDIVbits.PLLPOST = 0;                     // N1=2
    CLKDIVbits.PLLPRE = 0;                      // N2=2
    OSCTUN = 0;                                 // Tune FRC oscillator, if FRC is used

    // Disable Watch Dog Timer
    RCONbits.SWDTEN = 0;

    // Clock switch to incorporate PLL
    __builtin_write_OSCCONH( 0x03 );            // Initiate Clock Switch to Primary Oscillator with PLL
    __builtin_write_OSCCONL( OSCCON || 0x01 );  // Start clock switching
    while( OSCCONbits.COSC != 0b011 );

    // Wait for Clock switch to occur
    // Wait for PLL to lock
    while( OSCCONbits.LOCK != 1 )
    { };

    // Save original EDS page values
    temp1 = DSRPAG;
    temp2 = DSWPAG;

    DSRPAG = __builtin_edspage( m1 );
    DSWPAG = __builtin_edspage( sum1 );
    vectorMultiply( ( int * ) m1, x, ( int * ) sum1 );
#ifdef TEST_MODE
    if(tempsum == testSum1)
        test_flag1=1;
    else
        test_flag1=0;

#endif
    DSRPAG = __builtin_edspage( m2 );
    DSWPAG = __builtin_edspage( sum2 );
    vectorMultiply( ( int * ) m2, x, ( int * ) sum2 );
#ifdef TEST_MODE

    if(tempsum == testSum2)
        test_flag2=1;
    else
        test_flag2=0;

#endif
    // Restore original EDS page values
    DSRPAG = temp1;
    DSWPAG = temp2;

    vectorMultiply( (__eds__ int *) m, x, (__eds__ int *) sum );
#ifdef TEST_MODE

    if(tempsum == testSum)
        test_flag3=1;
    else
        test_flag3=0;
// Make sure that all the access of the variables at EDS and calculations are rightly done by checking the flags
   if(test_flag1 && test_flag2 && test_flag3)
       return 0;
   else
       return 1;

#else
    while( 1 );
#endif
}

/******************************************************************************
 * Function:        void vectorMultiply(__eds__ int *m, int *x, __eds__ int *sum)
 *
 * PreCondition:    None
  *
 * Input:           Two operands for addition m and x and resultant storage 'sum' variable
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is used to add __eds and normal variable and result is stored
 *                  at location through __eds manner
 *****************************************************************************/
void vectorMultiply( __eds__ int *m, int *x, __eds__ int *sum )
{
    int i;
#ifdef TEST_MODE
    tempsum=0;
#endif
    for( i = 0; i < 5; i++ )
    {
#ifdef TEST_MODE
        tempsum = tempsum + ((*m) *(*x));
#endif

        ( *sum++ ) = ( *m++ ) * ( *x++ );
    }
}

/*******************************************************************************
 End of File
 */

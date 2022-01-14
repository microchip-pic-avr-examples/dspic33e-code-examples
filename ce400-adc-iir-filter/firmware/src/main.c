/*******************************************************************************
  ce400 main file

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    Calls all the ADC and DSP functions

  Description:
    This file has the main function that calls the DSP filter functions
    and the ADC init and DMA init and adcdrv1 function that configures the ADC
    to a particular mode of operation. The main file also has macros to configure
    the clock source for the device.
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
#include <dsp.h>
#include"adcdrv1.h"

#if __XC16_VERSION < 1011
#warning "Please upgrade to XC16 v1.11 or newer."
#endif

// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************


extern IIRTransposedStruct ExampleHPFFilter;
extern void IIRTransposedInit(IIRTransposedStruct* filter);
extern fractional* IIRTransposed(int numSamps,
        fractional* dstSamps,
        fractional* srcSamps,
        IIRTransposedStruct* filter
        );

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
unsigned char test_flag;
#endif
#ifdef TEST_MODE
int ce_main(void)
#else
int main(void)
#endif
{
    // Configure Oscillator to operate the device at 60Mhz
    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
    // Fosc= 8M*60/(2*2)=120Mhz for 8M input clock
    PLLFBD = 58; // M=60
    CLKDIVbits.PLLPOST = 0; // N1=2
    CLKDIVbits.PLLPRE = 0; // N2=2
    OSCTUN = 0; // Tune FRC oscillator, if FRC is used

    // Disable Watch Dog Timer
    RCONbits.SWDTEN = 0;

    // Clock switching to incorporate PLL
    __builtin_write_OSCCONH(0x03); // Initiate Clock Switch to Primary
    // Oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONL(OSCCON || 0x01); // Start clock switching
    while (OSCCONbits.COSC != 0b011); // Wait for Clock switch to occur

    // Wait for PLL to lock
    while (OSCCONbits.LOCK != 1) {
    };


    // IIR module initialisation
    IIRTransposedInit(&ExampleHPFFilter);


    // Peripheral Initialisation
    InitAdc1(); // Initialize the A/D converter to convert Channel 5
    InitDma0(); // Initialise the DMA controller to buffer ADC data in conversion order
    InitTmr3(); // Initialise the Timer to generate sampling event to ADC @ 8Khz rate

#ifdef TEST_MODE
    while(1)
    {
        if(test_flag ==1)
        return 0;
    }
#else

    while (1) //Loop Endlessly - Execution is interrupt driven
    { //from this point on.

    }

    return 0;
#endif

}

/*******************************************************************************
 End of File
 */


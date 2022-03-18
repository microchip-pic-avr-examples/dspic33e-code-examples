/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    main_fft_example.c

  Summary:
    This file is used to call the FFT related functions.

  Description:
    This file includes the function calls for FFTComplexIP, BitReverseComplex,
    SquareMagnitudeCplx functions that are central to the computation of the FFT
    of an input signal and then the peak frequency is calculated.
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
#include "fft.h"

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
#pragma config ICS = PGD1               // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)

// FPOR
#pragma config BOREN = ON               // Brown-out Reset (BOR) Detection Enable bit (BOR is enabled)
#pragma config ALTI2C1 = OFF            // Alternate I2C1 pins (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2 = OFF            // Alternate I2C2 pins (I2C2 mapped to SDA2/SCL2 pins)
#pragma config WDTWIN = WIN25           // Watchdog Window Select bits (WDT Window is 25% of WDT period)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler bits (1:32,768)
#pragma config WDTPRE = PR128           // Watchdog Timer Prescaler bit (1:128)
#pragma config PLLKEN = ON              // PLL Lock Enable bit (Clock switch to PLL source will wait until the PLL lock signal is valid.)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
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
// *****************************************************************************
// Section: /* Extern definitions */
// *****************************************************************************
// *****************************************************************************
 // Typically, the input to an FFT  is a complex array containing samples
 // of an input signal. For this example, we will provide the input signal in an
 // array declared in Y-data space.
 extern fractcomplex sigCmpx[FFT_BLOCK_LENGTH]
__attribute__ ((eds, space(ymemory), aligned (FFT_BLOCK_LENGTH * 2 *2)));	      		/*  */

 /* Declare Twiddle Factor array in X-space*/
 #ifndef FFTTWIDCOEFFS_IN_PROGMEM
 fractcomplex twiddleFactors[FFT_BLOCK_LENGTH/2] 	
 __attribute__ ((section (".xbss, bss, xmemory"), aligned (FFT_BLOCK_LENGTH*2)));
 #else
 extern const fractcomplex twiddleFactors[FFT_BLOCK_LENGTH/2]	/* Twiddle Factor array in Program memory */
 __attribute__ ((space(prog), aligned (FFT_BLOCK_LENGTH*2)));
 #endif
 extern fractional input[FFT_BLOCK_LENGTH];
// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************
fractional output[FFT_BLOCK_LENGTH/2];
int16_t	peakFrequencyBin = 0; // Declare post-FFT variables to compute the
uint32_t peakFrequency = 0; // frequency of the largest spectral component


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
 * Overview:        Sets CPU clock and initializes Ports and timer.
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
    test_flag=0;
#endif

    int i = 0;
#ifndef FFTTWIDCOEFFS_IN_PROGMEM					/* Generate TwiddleFactor Coefficients */
    TwidFactorInit (LOG2_BLOCK_LENGTH, &twiddleFactors[0], 0);	/* We need to do this only once at start-up */
#endif

    for ( i = 0; i < FFT_BLOCK_LENGTH; i++ )/* The FFT function requires input data */
    {					/* to be in the fractional fixed-point range [-0.5, +0.5]*/
            sigCmpx[i].real = input[i] >> 1 ;		/* So, we shift all data samples by 1 bit to the right. */
            sigCmpx[i].imag = 0;			/* Should you desire to optimize this process, perform */
    }

	/* Perform FFT operation */
#ifndef FFTTWIDCOEFFS_IN_PROGMEM
    FFTComplexIP (LOG2_BLOCK_LENGTH, &sigCmpx[0], &twiddleFactors[0], COEFFS_IN_DATA);
#else
    FFTComplexIP (LOG2_BLOCK_LENGTH, &sigCmpx[0], (fractcomplex *) __builtin_psvoffset(&twiddleFactors[0]), (int) __builtin_psvpage(&twiddleFactors[0]));
#endif

    /* Store output samples in bit-reversed order of their addresses */
    BitReverseComplex (LOG2_BLOCK_LENGTH, &sigCmpx[0]);

    /* Compute the square magnitude of the complex FFT output array so we have a Real output vetor */
    SquareMagnitudeCplx(FFT_BLOCK_LENGTH/2, &sigCmpx[0], output);

    /* Find the frequency Bin ( = index into the SigCmpx[] array) that has the largest energy*/
    /* i.e., the largest spectral component */
    VectorMax(FFT_BLOCK_LENGTH/2, output, &peakFrequencyBin);

    /* Compute the frequency (in Hz) of the largest spectral component */
    peakFrequency = peakFrequencyBin*(SAMPLING_RATE/FFT_BLOCK_LENGTH);

#ifdef TEST_MODE
  while(1)
  {
    if ((peakFrequency >= 950) && (peakFrequency <= 1050))
    {
            test_flag=1;
           return 0;
    }
    else
    {
        test_flag=0;
        return 1;
    }
  }
#else

    while (1);	/* Place a breakpoint here and observe the watch window variables */
#endif

}
/*******************************************************************************
 End of File
*/
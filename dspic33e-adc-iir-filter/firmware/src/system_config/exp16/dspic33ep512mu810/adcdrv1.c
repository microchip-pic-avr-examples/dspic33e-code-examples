/*******************************************************************************
  ADC driver functions source file

  Company:
    Microchip Technology Inc.

  File Name:
    adcdrv1.c

  Summary:
    ADC function definitions

  Description:
    This file has the DSP function declarations derived from the dsp.h file
    and also has the initialization routines for the ADC and DMA modules. 
    Also, the ADC is configured for auto sampling and produces results 
    which are in the signed fractional format and puts the result in the DMA/x-memory
    buffers. These buffers are then used as inputs to perform filtering of the input
    analog signal. 
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
#include <dsp.h>
#include "adcdrv1.h"
#include "tglpin.h"

// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************


// DSPIC33EP512MU810 Configuration Bit Settings

// FICD
#pragma config ICS = PGD1               // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)

// FPOR

#pragma config ALTI2C1 = OFF            // Alternate I2C1 pins (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2 = OFF            // Alternate I2C2 pins (I2C2 mapped to SDA2/SCL2 pins)

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
#pragma config IESO = OFF               // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FGS
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)



/* This section defines the input and the output buffers having a fractional data type.
The input buffers are stored in the x-memory area.
The output is also of fractional data type.*/


fractional bufferA[NUMSAMP] __attribute__((space(xmemory)));
fractional bufferB[NUMSAMP] __attribute__((space(xmemory)));


fractional outputSignal[NUMSAMP];

extern IIRTransposedStruct ExampleHPFFilter;
extern void IIRTransposedInit(IIRTransposedStruct* filter);
extern fractional* IIRTransposed(int numSamps,
        fractional* dstSamps,
        fractional* srcSamps,
        IIRTransposedStruct* filter
        );


/******************************************************************************
 * Function:        void InitAdc1(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is used to configure A/D to convert channel 5 on Timer event.
                    It generates event to DMA on every sample/convert sequence. ADC clock is configured at 625Khz.


 *****************************************************************************/

void InitAdc1(void) {

    AD1CON1bits.FORM = 3; // Data Output Format: Signed Fraction (Q15 format)
    AD1CON1bits.SSRC = 2; // Sample Clock Source: GP Timer starts conversion
    AD1CON1bits.ASAM = 1; // ADC Sample Control: Sampling begins immediately after conversion
    AD1CON1bits.AD12B = 1; // 12-bit ADC operation
    AD1CON1bits.ADDMABM = 1; // DMA buffers are built in conversion order mode

    AD1CON2bits.CHPS = 0; // Converts CH0

    AD1CON3bits.ADRC = 0; // ADC Clock is derived from Systems Clock
    AD1CON3bits.ADCS = 63; // ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/40M)*64 = 1.6us (625Khz)
    // ADC Conversion Time for 12-bit Tc=14*Tad = 22.4us

    AD1CON1bits.ADDMABM = 1; // DMA buffers are built in conversion order mode
    AD1CON2bits.SMPI = 0; // SMPI must be 0


    //AD1CHS0: A/D Input Select Register
    AD1CHS0bits.CH0SA = 5; // MUXA +ve input selection (AN5) for CH0
    AD1CHS0bits.CH0NA = 0; // MUXA -ve input selection (Vref-) for CH0

    IFS0bits.AD1IF = 0; // Clear the A/D interrupt flag bit
    IEC0bits.AD1IE = 0; // Do Not Enable A/D interrupt
    AD1CON1bits.ADON = 1; // Turn on the A/D converter

    TglPinInit();

}

/******************************************************************************
 * Function:        void InitTmr3(void) 
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Timer3 is setup to time-out every 125 microseconds (8Khz Rate). As a result, the module
                    will stop sampling and trigger a conversion on every Timer3 time-out, i.e., Ts=125us.
                    At that time, the conversion process starts and completes Tc=14*Tad periods later.
                    When the conversion completes, the module starts sampling again. However, since Timer3
                    is already on and counting, about (Ts-Tc)us later, Timer3 will expire again and trigger
                    next conversion.

 *****************************************************************************/
void InitTmr3(void) {
    TMR3 = 0x0000;
    PR3 = SAMPPRD;
    IFS0bits.T3IF = 0;
    IEC0bits.T3IE = 0;

    //Start Timer 3
    T3CONbits.TON = 1;

}


/******************************************************************************
 * Function:        void InitDma0(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        1. DMA0 configuration function.
                    2. Direction: Read from peripheral address 0-x300 (ADC1BUF0) and write to DMA RAM
                       AMODE: Register indirect with post increment
                       MODE: Continuous, Ping-Pong Mode
                       IRQ: ADC Interrupt
                       ADC stores results stored alternatively between DMA_BASE[0]/DMA_BASE[16] on every
                       16th DMA request

 *****************************************************************************/
void InitDma0(void) {
    DMA0CONbits.AMODE = 0; // Configure DMA for Register indirect with post increment
    DMA0CONbits.MODE = 2; // Configure DMA for Continuous Ping-Pong mode

    DMA0PAD = (int) &ADC1BUF0;
    DMA0CNT = (NUMSAMP - 1);

    DMA0REQ = 13;

    DMA0STAL = (unsigned int) &bufferA;
    DMA0STAH = (unsigned int) &bufferA;
    DMA0STBL = (unsigned int) &bufferB;
    DMA0STBH = (unsigned int) &bufferB;

    IFS0bits.DMA0IF = 0; //Clear the DMA interrupt flag bit
    IEC0bits.DMA0IE = 1; //Set the DMA interrupt enable bit

    DMA0CONbits.CHEN = 1;

}

/******************************************************************************
 * Function:        void __attribute__((interrupt, auto_psv)) _DMA0Interrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Depending on the dmaBuffer, the data in bufferA or bufferB is passed
 *****************************************************************************/

unsigned int dmaBuffer = 0;

void __attribute__((interrupt, auto_psv)) _DMA0Interrupt(void) {

    if (dmaBuffer == 0)
    {
         IIRTransposed(NUMSAMP, &outputSignal[0], (fractional*)bufferA, &ExampleHPFFilter);
    }
    else
    {
	    IIRTransposed(NUMSAMP, &outputSignal[0], (fractional*)bufferB, &ExampleHPFFilter);
    }

    dmaBuffer ^= 1;
#ifdef TEST_MODE
// Testing Reference Value consideration: Keep the POT(arrow mark of POT to extreme right/maximum)
// on expl16 board for testing
// Here we are checking for one of the samples to be correct
     if((outputSignal[0] >= (-10820)) &&
       (outputSignal[0] <= (18500))
      )
        test_flag=1;
#endif
    TglPin(); // Toggle RA4
    IFS0bits.DMA0IF = 0; //Clear the DMA0 Interrupt Flag
}

/*******************************************************************************
 End of File
 */







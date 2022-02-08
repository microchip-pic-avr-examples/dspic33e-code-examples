/*******************************************************************************
  ADC driver functions source file

  Company:
    Microchip Technology Inc.

  File Name:
    adcdrv2.c

  Summary:
    ADC function definitions

  Description:
    This file has the initialization routines for the ADC and DMA modules. 
    Also, the ADC is configured for auto sampling and produces results 
    which are in the signed fractional format and puts the result in the DMA/x-memory
    buffers.  
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
#include "adcdrv2.h"
#include "tglpin.h"

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

// User Defines
#define FCY         40000000        // User must calculate and enter FCY here
#define Dly_Time    ( 20E-6 * FCY ) // ADC Off-to-On delay

//uncomment next line if __eds__ is supported
//#define _HAS_DMA_
#define NUMSAMP 256

// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************
/* This section defines the input and the output buffers having a integer data type.
The input buffers are stored in either the extended data space or the x-memory area
depending on whether the extended data space (EDS) is available in the given dsPIC33E device.
 */
#ifdef _HAS_DMA_
__eds__ int bufferA[NUMSAMP] __attribute__( (eds, space(dma)) );
__eds__ int bufferB[NUMSAMP] __attribute__( (eds, space(dma)) );
#else
int         bufferA[NUMSAMP] __attribute__( (space(xmemory)) );
int         bufferB[NUMSAMP] __attribute__( (space(xmemory)) );
#endif
void        ProcessADCSamples( __eds__ int *adcBuffer );

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
void InitAdc1( void )
{
    AD1CON1bits.FORM = 3;       // Data Output Format: Signed Fraction (Q15 format)
    AD1CON1bits.SSRC = 7;       // Interan Counter (SAMC) ends sampling and starts convertion
    AD1CON1bits.ASAM = 1;       // ADC Sample Control: Sampling begins immediately after conversion
    AD1CON1bits.AD12B = 0;      // 10-bit ADC operation
    AD1CON2bits.CHPS = 1;       // Converts CH0/CH1
    AD1CON3bits.ADRC = 0;       // ADC Clock is derived from Systems Clock
    AD1CON3bits.SAMC = 0;       // Auto Sample Time = 0*Tad
    AD1CON3bits.ADCS = 2;       // ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/40M)*3 = 75ns (13.3Mhz)

    // ADC Conversion Time for 10-bit Tc=12*Tab =  900ns (1.1MHz)
    AD1CON1bits.ADDMABM = 1;    // DMA buffers are built in conversion order mode
    AD1CON2bits.SMPI = 0;       // SMPI must be 0
    AD1CON4bits.ADDMAEN = 1;    // Converts in ADC1BUF0

    //AD1CHS0/AD1CHS123: A/D Input Select Register
    AD1CHS0bits.CH0SA = 5;      // MUXA +ve input selection (AIN20) for CH0
    AD1CHS0bits.CH0NA = 0;      // MUXA -ve input selection (Vref-) for CH0
    AD1CHS123bits.CH123NA = 0;  // MUXA -ve input selection (Vref-) for CH1
    IFS0bits.AD1IF = 0;         // Clear the A/D interrupt flag bit
    IEC0bits.AD1IE = 0;         // Do Not Enable A/D interrupt
    AD1CON1bits.ADON = 1;       // Turn on the A/D converter
    TglPinInit();

    //        Dly_Time();            // Delay for 20uS to allow ADC to settle (25nS * 0x320 = 20uS)
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
 * Overview:        DMA0 configuration function.
                    Direction: Read from peripheral address 0-x300 (ADC1BUF0) and write to DMA RAM
                    AMODE: Register indirect with post increment
                    MODE: Continuous, Ping-Pong Mode
                    IRQ: ADC Interrupt
                    ADC stores results stored alternatively between DMA_BASE[0]/DMA_BASE[16] on every
                    16th DMA request
 *****************************************************************************/
void InitDma0( void )
{
    DMA0CONbits.AMODE = 0;  // Configure DMA for Register indirect with post increment
    DMA0CONbits.MODE = 2;   // Configure DMA for Continuous Ping-Pong mode
    DMA0PAD = ( int ) &ADC1BUF0;
    DMA0CNT = ( NUMSAMP - 1 );

    DMA0REQ = 13;

    #ifdef _HAS_DMA_
    DMA0STAL = __builtin_dmaoffset( &bufferA );
    DMA0STAH = __builtin_dmapage( &bufferA );

    DMA0STBL = __builtin_dmaoffset( &bufferB );
    DMA0STBH = __builtin_dmapage( &bufferB );

    #else
    DMA0STAL = ( unsigned int ) &bufferA;
    DMA0STAH = ( unsigned int ) &bufferA;

    DMA0STBL = ( unsigned int ) &bufferB;
    DMA0STBH = ( unsigned int ) &bufferB;
    #endif
    IFS0bits.DMA0IF = 0;    //Clear the DMA interrupt flag bit
    IEC0bits.DMA0IE = 1;    //Set the DMA interrupt enable bit
    DMA0CONbits.CHEN = 1;
}

unsigned int    dmaBuffer = 0;
#ifdef TEST_MODE
        unsigned char test_flag_buffA=0;
        unsigned char test_flag_buffB=0;
#endif
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
 * Overview:        Depending on the dmaBuffer value, the data in bufferA or bufferB is passed
 *****************************************************************************/
void __attribute__ ( (interrupt, auto_psv) ) _DMA0Interrupt( void )
{
    if( dmaBuffer == 0 )
    {
#ifdef TEST_MODE
        test_flag_buffA=1;
#endif
        ProcessADCSamples( bufferA );
    }
    else
    {
#ifdef TEST_MODE
        test_flag_buffB=1;
#endif 
        ProcessADCSamples( bufferB );
    }

    dmaBuffer ^= 1;
    TglPin();               // Toggle RA4
    IFS0bits.DMA0IF = 0;    //Clear the DMA0 Interrupt Flag
}

/******************************************************************************
 * Function:        void ProcessADCSamples(__eds__ int16_t * adcBuffer)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is just a prototype which can be used to perform
 *                  some activity on the samples taken by the ADC.
 *****************************************************************************/
void ProcessADCSamples( __eds__ int *adcBuffer )
{
    /* Do something with ADC Samples */
}

/*******************************************************************************
 End of File
 */

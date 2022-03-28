/*******************************************************************************
  ADC driver functions source file

  Company:
    Microchip Technology Inc.

  File Name:
    adcdrv1.c

  Summary:
    ADC function definitions

  Description:
    This source file configures the ADC for channel scanning and 10-bit operating mode is used. 
    The ADC works with the DMA and the result is stored in the DMA buffers. The timer 3 is used 
    to provide trigger for the conversion operations once every 125us or 8 kHz rate. 
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

#include "adcdrv1.h"
#include "tglpin.h"

// DSPIC33EP256GP506 Configuration Bit Settings
// FICD
#pragma config ICS = PGD1           // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config JTAGEN = OFF         // JTAG Enable bit (JTAG is disabled)

// FPOR
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

//uncomment next line if __eds__ is supported
//#define _HAS_DMA_
// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************
// Define Message Buffer Length for ECAN1/ECAN2
#define MAX_CHNUM       13          // Highest Analog input number in Channel Scan
#define SAMP_BUFF_SIZE  8           // Size of the input buffer per analog input
#define NUM_CHS2SCAN    4           // Number of channels enabled for channel scan

// Number of locations for ADC buffer = 14 (AN0 to AN13) x 8 = 112 words
// Align the buffer to 128 words or 256 bytes. This is needed for peripheral indirect mode
#ifdef _HAS_DMA_
__eds__ int16_t bufferA[MAX_CHNUM + 1][SAMP_BUFF_SIZE] __attribute__( (eds, space(dma), aligned(256)) );
__eds__ int16_t bufferB[MAX_CHNUM + 1][SAMP_BUFF_SIZE] __attribute__( (eds, space(dma), aligned(256)) );
#else
int16_t         bufferA[MAX_CHNUM + 1][SAMP_BUFF_SIZE] __attribute__( (space(xmemory), aligned(256)) );
int16_t         bufferB[MAX_CHNUM + 1][SAMP_BUFF_SIZE] __attribute__( (space(xmemory), aligned(256)) );
#endif

#ifdef TEST_MODE
unsigned char  test_dma0_int_flag =0;
unsigned char test_process_adc_samp_flag=0;
#endif


//Function Prototype
void            ProcessADCSamples( __eds__ int16_t *adcBuffer );

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
 * Overview:        ADC initialization/configuration function.
 *                  This function is used to configure the ADC for channel scanning of 4 channels. 
 *****************************************************************************/
void InitAdc1( void )
{
    AD1CON1bits.FORM = 3;                       // Data Output Format: Signed Fraction (Q15 format)
    AD1CON1bits.SSRC = 2;                       // Sample Clock Source: GP Timer starts conversion
    AD1CON1bits.ASAM = 1;                       // ADC Sample Control: Sampling begins immediately after conversion
    AD1CON1bits.AD12B = 0;                      // 10-bit ADC operation
    AD1CON2bits.CSCNA = 1;                      // Scan Input Selections for CH0+ during Sample A bit
    AD1CON2bits.CHPS = 0;                       // Converts CH0
    AD1CON3bits.ADRC = 0;                       // ADC Clock is derived from Systems Clock
    AD1CON3bits.ADCS = 63;                      // ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/40M)*64 = 1.6us (625Khz)

    // ADC Conversion Time for 10-bit Tc=12*Tab = 19.2us
    AD1CON1bits.ADDMABM = 0;                    // DMA buffers are built in scatter/gather mode
    AD1CON2bits.SMPI = ( NUM_CHS2SCAN - 1 );    // 4 ADC Channel is scanned
    AD1CON4bits.DMABL = 3;                      // Each buffer contains 8 words
    AD1CON4bits.ADDMAEN = 1;                    // Conversion results stored in ADCxBUF0 register

    //AD1CSSH/AD1CSSL: A/D Input Scan Selection Register
    AD1CSSH = 0x0000;
    AD1CSSLbits.CSS4 = 1;                       // Enable AN4 for channel scan
    AD1CSSLbits.CSS5 = 1;                       // Enable AN5 for channel scan
    AD1CSSLbits.CSS10 = 1;                      // Enable AN10 for channel scan
    AD1CSSLbits.CSS13 = 1;                      // Enable AN13 for channel scan
    IFS0bits.AD1IF = 0;     // Clear the A/D interrupt flag bit
    IEC0bits.AD1IE = 0;     // Do Not Enable A/D interrupt
    AD1CON1bits.ADON = 1;   // Turn on the A/D converter
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
 * Overview:        Timer initialization/configuration function.
 *                  Timer 3 is setup to time-out every 125 microseconds (8Khz Rate).
 *                  As a result, the module  will stop sampling and trigger a conversion on every Timer3
 *                  time-out, i.e., Ts=125us.
 *****************************************************************************/
void InitTmr3( void )
{
    TMR3 = 0x0000;
    PR3 = 4999;         // Trigger ADC1 every 125usec
    IFS0bits.T3IF = 0;  // Clear Timer 3 interrupt
    IEC0bits.T3IE = 0;  // Disable Timer 3 interrupt
    T3CONbits.TON = 1;  //Start Timer 3
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
 * Overview:         This function is used to configure the DMA0 module to read from peripheral
 *                  address 0x300 (ADC1BUF0)  and write to the RAM continuously using peripheral
 *                  indirect addressing in ping pong mode.
 *****************************************************************************/
void InitDma0( void )
{
    DMA0CONbits.AMODE = 2;  // Configure DMA for Peripheral indirect mode
    DMA0CONbits.MODE = 2;   // Configure DMA for Continuous Ping-Pong mode
    DMA0PAD = ( int ) &ADC1BUF0;
    DMA0CNT = ( SAMP_BUFF_SIZE * NUM_CHS2SCAN ) - 1;
    DMA0REQ = 13;           // Select ADC1 as DMA Request source
    #ifdef _HAS_DMA_
    DMA0STAL = __builtin_dmaoffset( &bufferA );
    DMA0STAH = __builtin_dmapage( &bufferA );

    DMA0STBL = __builtin_dmaoffset( &bufferB );
    DMA0STBH = __builtin_dmapage( &bufferB );

    #else
    DMA0STAL = ( uint16_t ) ( &bufferA );
    DMA0STAH = ( uint16_t ) ( &bufferA );

    DMA0STBL = ( uint16_t ) ( &bufferB );
    DMA0STBH = ( uint16_t ) ( &bufferB );
    #endif
    IFS0bits.DMA0IF = 0;    //Clear the DMA interrupt flag bit
    IEC0bits.DMA0IE = 1;    //Set the DMA interrupt enable bit
    DMA0CONbits.CHEN = 1;   // Enable DMA
}

/*=============================================================================
_DMA0Interrupt(): ISR name is chosen from the device linker script.
=============================================================================*/
// dmaBuffer variable is used to toggle the buffer between A and B for copying.
uint16_t    dmaBuffer = 0;

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
 * Overview:        Process the data in buffer A or in buffer B by invoking ProcessADCSamples function
 *****************************************************************************/
void __attribute__ ( (interrupt, auto_psv) ) _DMA0Interrupt( void )
{


#ifdef TEST_MODE
    test_dma0_int_flag=1;
#endif
    if( dmaBuffer == 0 )
    {
        ProcessADCSamples( &bufferA[4][0] );
        ProcessADCSamples( &bufferA[5][0] );
        ProcessADCSamples( &bufferA[10][0] );
        ProcessADCSamples( &bufferA[13][0] );
    }
    else
    {
        ProcessADCSamples( &bufferB[4][0] );
        ProcessADCSamples( &bufferB[5][0] );
        ProcessADCSamples( &bufferB[10][0] );
        ProcessADCSamples( &bufferB[13][0] );
    }

    dmaBuffer ^= 1;

    TglPin();               // Toggle RA4
    IFS0bits.DMA0IF = 0;    // Clear the DMA0 Interrupt Flag
}

/******************************************************************************
 * Function:        void ProcessADCSamples(__eds__ int16_t * AdcBuffer)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Dummy function, does nothing.
 *****************************************************************************/
void ProcessADCSamples( __eds__ int16_t *adcBuffer )
{
    /* Do something with ADC Samples */
#ifdef TEST_MODE
    test_process_adc_samp_flag=1;
#endif
}

/*******************************************************************************
 End of File
*/

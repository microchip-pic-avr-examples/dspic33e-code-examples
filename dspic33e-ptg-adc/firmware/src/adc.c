/*******************************************************************************
    ADC Configuration Code Source file

  Company:
    Microchip Technology Inc.

  File Name:
    adc.c

  Summary:
    This code example generates multiple ADC triggers in synchronisation with
    PWM time base
  Description:
    This file contains ADC Initialisation routine
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
#ifdef TEST_MODE
#include "common.h"
#endif
// *****************************************************************************
// *****************************************************************************
// Section: Helper Macros
// *****************************************************************************
// *****************************************************************************
#define SAMP_BUFF_SIZE  10  // Size of the input buffer per analog input

// Variables
int             ain0Buff[SAMP_BUFF_SIZE];
int             ain1Buff[SAMP_BUFF_SIZE];
int             ain2Buff[SAMP_BUFF_SIZE];
int             ain3Buff[SAMP_BUFF_SIZE];
unsigned int    sampleCounter = 0;

// Functions
void            Init_ADC( void );

/******************************************************************************
 * Function:        void Init_ADC(void)
 *
 * PreCondition:    None.
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function intialises ADC in 10 bit mode and to sample CHO,CH1,CH2 and
                    CH3 simultaneously
 *****************************************************************************/
void Init_ADC( void )
{
    // ============= ADC - Measure Current & Pot ======================
    // ADC setup for simultanous sampling on
    // CH0=AN3, CH1=AN0, CH2=AN1, CH3=AN2.
    // Sampling triggered by PTG TRIG 12
    AD1CON1bits.FORM = 0;

    // PTG Trigger 12 ends sampling and starts conversion
    AD1CON1bits.SSRC = 3;
    AD1CON1bits.SSRCG = 1;

    // Simultaneous Sample Select bit (only applicable when CHPS = 01 or 1x)
    // Samples CH0, CH1, CH2, CH3 simultaneously (when CHPS = 1x)
    // Samples CH0 and CH1 simultaneously (when CHPS = 01)
    AD1CON1bits.SIMSAM = 1;

    // Sampling begins immediately after last conversion completes.
    // SAMP bit is auto set.
    AD1CON1bits.ASAM = 1;
    AD1CON1bits.AD12B = 0;              // 10-bit ADC operation
    AD1CON4bits.ADDMAEN = 0;            // Conversion results stored in ADCxBUF0 register
    AD1CON2 = 0;
    AD1CON2bits.CHPS = 2;               // Samples CH0, CH1, CH2, CH3 simultaneously (when CHPS = 1x)
    AD1CON2bits.SMPI = 0;               // 4 ADC Channel is scanned
    AD1CON3 = 0;
    AD1CON3bits.ADRC = 0;               // ADC Clock is derived from Systems Clock
    AD1CON3bits.ADCS = 3;               // A/D Conversion Clock Select bits = 4 * Tcy

    // ADCHS: ADC Input Channel Select Register
    AD1CHS0 = 0;
    AD1CHS0bits.CH0SA = 3;              // CH0 is AN3

    // CH1 positive input is AN0, CH2 positive input is AN1, CH3 positive input is AN2
    // ADCSSL: ADC Input Scan Select Register
    AD1CSSL = 0;

    IFS0bits.AD1IF = 0;                 // Clear the A/D interrupt flag bit
    IEC0bits.AD1IE = 1;                 // Enable A/D interrupt
    _AD1IP = 4;

    // Turn on A/D module
    AD1CON1bits.ADON = 1;
}

/******************************************************************************
 * Function:        void __attribute__((__interrupt__, no_auto_psv)) _AD1Interrupt(void) 
 *
 * PreCondition:    None.
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        ADC interrupt service routine stores converted results to rescpective array
 *****************************************************************************/
void __attribute__ ( (__interrupt__, no_auto_psv) ) _AD1Interrupt( void )
{
    _RC6 = ~_RC6;                       //Toggles port pin RC6 after conversion
    ain0Buff[sampleCounter] = ADC1BUF0; //AN3
    ain1Buff[sampleCounter] = ADC1BUF1; //AN0
    ain2Buff[sampleCounter] = ADC1BUF2; //AN1
    ain3Buff[sampleCounter] = ADC1BUF3; //AN2
    sampleCounter++;
    if( sampleCounter == SAMP_BUFF_SIZE )
    {
        sampleCounter = 0;
    }

    _AD1IF = 0;
#ifdef TEST_MODE
    test_flag=1;
#endif
}

/*******************************************************************************
 End of File
 */

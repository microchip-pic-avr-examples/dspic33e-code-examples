/*******************************************************************************
  ADC driver functions source file

  Company:
    Microchip Technology Inc.

  File Name:
    adcdrv3.c

  Summary:
    ADC function definitions

  Description:
    This file has the ADC initialization routine that configures the ADC for
    auto sampling with output in signed fractional format. The result is available 
    in the ADC result buffer. 
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
#include "adcdrv3.h"
#include "tglpin.h"
#include <stdint.h>

// DSPIC33EP512GM710 Configuration Bit Settings
// FGS
#pragma config GWRP = OFF           // General Segment Write-Protect bit (General Segment may be written)

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
#pragma config BOREN = ON           // Brown-out Reset (BOR) Detection Enable bit (BOR is enabled)
#pragma config ALTI2C1 = OFF        // Alternate I2C pins for I2C1 (SDA1/SCK1 pins are selected as the I/O pins for I2C1)
#pragma config ALTI2C2 = OFF        // Alternate I2C pins for I2C2 (SDA2/SCK2 pins are selected as the I/O pins for I2C2)

// FICD
#pragma config ICS = PGD1           // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config JTAGEN = OFF         // JTAG Enable bit (JTAG is disabled)

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
void InitAdc1 ( void )
{
    AD1CON1bits.FORM = 3;           // Data Output Format: Signed Fraction (Q15 format)
    AD1CON1bits.SSRC = 0;           // Clear SAMP bit ends sampling and starts conversion
    AD1CON1bits.SSRCG = 0;
    AD1CON1bits.ASAM = 1;           // ADC Sample Control: Sampling begins immediately after conversion
    AD1CON1bits.AD12B = 0;          // 10-bit ADC operation
    AD1CON2bits.CHPS = 0;           // Converts CH0

    // Set up A/D conversion clock to ADC Internal RC so that it will work in SLEEP mode
    AD1CON3 = ( AD1CON3 | 0x8000 );
    AD1CON3bits.SAMC = 1;           // Auto Sample Time = 1*Tad
    AD1CON3bits.ADCS = 0;           // ADC Conversion Clock Tad=Trc
    AD1CON2bits.SMPI = 0;           // SMPI must be 0

    //AD1CHS0: A/D Input Select Register
    AD1CHS0bits.CH0SA = 5;          // MUXA +ve input selection (AIN5) for CH0
    AD1CHS0bits.CH0NA = 0;          // MUXA -ve input selection (Vref-) for CH0
    IFS0bits.AD1IF = 0;             // Clear the A/D interrupt flag bit
    IEC0bits.AD1IE = 1;             // Do Not Enable A/D interrupt
    AD1CON1bits.ADON = 1;           // Turn on the A/D converter
    TglPinInit();
}

// Variable to hold the buffered data
uint16_t result;

/******************************************************************************
 * Function:        void __attribute__((interrupt, no_auto_psv)) _AD1Interrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Depending on the scancounter, the data at each ADC buffer is copied to various buffer
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _AD1Interrupt( void )
{
    result = ADC1BUF0;
#ifdef TEST_MODE
    test_flag=1;
#endif
    TglPin();                       // Toggle RA4
    IFS0bits.AD1IF = 0;             // Clear the ADC1 Interrupt Flag
}

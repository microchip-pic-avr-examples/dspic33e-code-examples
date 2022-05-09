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
#pragma config IESO = OFF           // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FGS
#pragma config GWRP = OFF           // General Segment Write-Protect bit (General Segment may be written)
#pragma config GCP = OFF            // General Segment Code-Protect bit (General Segment Code protect is Disabled)
#define SAMP_BUFF_SIZE  8           // Size of the input buffer per analog input
#define NUM_CHS2SCAN    4           // Number of channels enabled for channel scan

// Buffer declaration to collect data from various ADC buffers
int ain4Buff[SAMP_BUFF_SIZE];
int ain5Buff[SAMP_BUFF_SIZE];
int ain10Buff[SAMP_BUFF_SIZE];
int ain13Buff[SAMP_BUFF_SIZE];
int scanCounter = 0;
int sampleCounter = 0;
#ifdef TEST_MODE
unsigned char test_scan_ch1_flag=0;
unsigned char test_scan_ch2_flag=0;
unsigned char test_scan_ch3_flag=0;
unsigned char test_scan_ch4_flag=0;
#endif
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
 *                  This function is used to configure the ADC for channel scanning
 *****************************************************************************/
void InitAdc1( void )
{
    AD1CON1bits.FORM = 3;                       // Data Output Format: Signed Fraction (Q15 format)
    AD1CON1bits.SSRC = 2;                       // Sample Clock Source: GP Timer3 starts conversion
    AD1CON1bits.SSRCG = 0;                      // Sample Clock Source: GP Timer3 starts conversion
    AD1CON1bits.ASAM = 1;                       // ADC Sample Control: Sampling begins immediately after conversion
    AD1CON1bits.AD12B = 0;                      // 10-bit ADC operation
    AD1CON2bits.CSCNA = 1;                      // Scan Input Selections for CH0+ during Sample A bit
    AD1CON2bits.CHPS = 0;                       // Converts CH0
    AD1CON3bits.ADRC = 0;                       // ADC Clock is derived from Systems Clock
    AD1CON3bits.ADCS = 63;                      // ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/40M)*64 = 1.6us (625Khz)

    // ADC Conversion Time for 10-bit Tc=12*Tab = 19.2us
    AD1CON2bits.SMPI = ( NUM_CHS2SCAN - 1 );    // 4 ADC Channel is scanned
    AD1CON4bits.ADDMAEN = 1;                    // Conversion results stored in ADCxBUF0 register

    //AD1CSSH/AD1CSSL: A/D Input Scan Selection Register
    AD1CSSH = 0x0000;
    AD1CSSLbits.CSS4 = 1;                       // Enable AN4 for channel scan
    AD1CSSLbits.CSS5 = 1;                       // Enable AN5 for channel scan
    AD1CSSLbits.CSS10 = 1;                      // Enable AN10 for channel scan
    AD1CSSLbits.CSS13 = 1;                      // Enable AN13 for channel scan
    ANSELB = 0xffff;
    TRISBbits.TRISB4 = 1;
#ifdef TEST_MODE
    TRISBbits.TRISB7 = 1;
#else
    TRISBbits.TRISB5 = 1;
#endif

    TRISBbits.TRISB10 = 1;
    TRISBbits.TRISB13 = 1;

    IFS0bits.AD1IF = 0;     // Clear the A/D interrupt flag bit
    IEC0bits.AD1IE = 1;     // Enable A/D interrupt
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
 * Overview:        Timer 3 is setup to time-out every 125 microseconds (8Khz Rate). As a result, the module
                    will stop sampling and trigger a conversion on every Timer3 time-out, i.e., Ts=125us.
 *****************************************************************************/
void InitTmr3( void )
{
    TMR3 = 0x0000;
    PR3 = 4999;         // Trigger ADC1 every 125usec
    IFS0bits.T3IF = 0;  // Clear Timer 3 interrupt
    IEC0bits.T3IE = 0;  // Disable Timer 3 interrupt

    //Start Timer 3
    T3CONbits.TON = 1;
}

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
    switch( scanCounter )
    {
        case 0:
            ain4Buff[sampleCounter] = ADC1BUF0;
#ifdef TEST_MODE
            test_scan_ch1_flag=1;
#endif
            break;

        case 1:
            ain5Buff[sampleCounter] = ADC1BUF0;
#ifdef TEST_MODE
            test_scan_ch2_flag=1;
#endif
            break;

        case 2:
            ain10Buff[sampleCounter] = ADC1BUF0;
#ifdef TEST_MODE
            test_scan_ch3_flag=1;
#endif
            break;

        case 3:
            ain13Buff[sampleCounter] = ADC1BUF0;
#ifdef TEST_MODE
            test_scan_ch4_flag=1;
#endif
            break;

        default:
            break;
    }

    scanCounter++;
    if( scanCounter == NUM_CHS2SCAN )
    {
        scanCounter = 0;
        sampleCounter++;
    }

    if( sampleCounter == SAMP_BUFF_SIZE )
    {
        sampleCounter = 0;
    }

    TglPin();           // Toggle RA6
    IFS0bits.AD1IF = 0; // Clear the ADC1 Interrupt Flag
}

/*******************************************************************************
 End of File
*/

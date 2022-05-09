/*******************************************************************************
  SPI & DMA Configuration source file

  Company:
    Microchip Technology Inc.

  File Name:
    spi1drv.c

  Summary:
    Configures the SPI1 and DMA modules for loopback operation.

  Description:
    This source file configures the SPI1 module to act as a master and send data in the loopback mode
    using the DMA and the DMA is configured to be operating in the ping pong mode. Either the DMA RAM
    or the normal RAM can be used to store the received data depending on whether the device has a
    dedicated DMA RAM or not.
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

// DSPIC33EP512GM710 Configuration Bit Settings
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
#pragma config POSCMD = NONE        // Primary Oscillator Mode Select bits (Primary Oscillator disabled)
#pragma config OSCIOFNC = OFF       // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config IOL1WAY = OFF        // Peripheral pin select configuration (Allow multiple reconfigurations)
#pragma config FCKSM = CSECMD       // Clock Switching Mode bits (Clock switching is enabled,Fail-safe Clock Monitor is disabled)

// FOSCSEL
#pragma config FNOSC = FRC          // Oscillator Source Selection (Internal Fast RC (FRC))
#pragma config IESO = OFF           // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FGS
#pragma config GWRP = OFF           // General Segment Write-Protect bit (General Segment may be written)
#pragma config GCP = OFF            // General Segment Code-Protect bit (General Segment Code protect is Disabled)

//Prototype of functions definitions
void CfgDma0SpiTx ( void );
void                CfgDma1SpiRx( void );
void                CfgSPI1Master( void );
void                ProcessSpiRxSamples( __eds__ uint16_t *SpiRxBuffer );

// used to switch the receiving buffer
uint16_t            rxDmaBuffer = 0;
#ifdef _HAS_DMA_
__eds__ uint16_t    SPI1RxBuffA[16] __attribute__( (eds, space(dma)) );
__eds__ uint16_t    SPI1RxBuffB[16] __attribute__( (eds, space(dma)) );
__eds__ uint16_t    SPI1TxBuffA[16] __attribute__( (eds, space(dma)) );
__eds__ uint16_t    SPI1TxBuffB[16] __attribute__( (eds, space(dma)) );

#else
uint16_t            SPI1RxBuffA[16] __attribute__( (space(xmemory)) );
uint16_t            SPI1RxBuffB[16] __attribute__( (space(xmemory)) );
uint16_t            SPI1TxBuffA[16] __attribute__( (space(xmemory)) );
uint16_t            SPI1TxBuffB[16] __attribute__( (space(xmemory)) );
#endif

/******************************************************************************
 * Function:       void CfgDma0SpiTx(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        1.This function is used to configure the DMA0 module to operate with the SPI1 transmitter and
 *                  transfer the contents from DMA RAM to the SPI module in continuous ping pong mode with
 *                  Register indirect addressing.
 *                  2. DMA0 initialization/configuration function.
 *****************************************************************************/
void CfgDma0SpiTx( void )
{
    DMA0CON = 0x2002;
    DMA0CNT = 15;
    DMA0REQ = 10;

    DMA0PAD = ( volatile uint16_t ) &SPI1BUF;
    #ifdef _HAS_DMA_
    DMA0STAL = ( uint16_t ) __builtin_dmaoffset( &SPI1TxBuffA );
    DMA0STAH = ( uint16_t ) __builtin_dmapage( &SPI1TxBuffA );

    DMA0STBL = ( uint16_t ) __builtin_dmaoffset( &SPI1TxBuffB );
    DMA0STBH = ( uint16_t ) __builtin_dmapage( &SPI1TxBuffB );

    #else
    DMA0STAL = ( uint16_t ) & SPI1TxBuffA;
    DMA0STAH = ( uint16_t ) & SPI1TxBuffA;

    DMA0STBL = ( uint16_t ) & SPI1TxBuffB;
    DMA0STBH = ( uint16_t ) & SPI1TxBuffB;
    #endif
    IFS0bits.DMA0IF = 0;    // Clear DMA interrupt
    IEC0bits.DMA0IE = 1;    // Enable DMA interrupt
    DMA0CONbits.CHEN = 1;   // Enable DMA Channel
}

/******************************************************************************
 * Function:       void CfgDma1SpiRx(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        1.This function is used to configure the DMA1 module to operate with the SPI1 receiver and
 *                    transfer the contents from SPI module to the DMA RAM in continuous ping pong mode
 *                    with Register indirect addressing.
 *                  2.DMA1 initialization/configuration function.
 *****************************************************************************/
void CfgDma1SpiRx( void )
{
    DMA1CON = 0x0002;
    DMA1CNT = 15;
    DMA1REQ = 10;
    DMA1PAD = ( volatile uint16_t ) &SPI1BUF;
    #ifdef _HAS_DMA_
    DMA1STAL = __builtin_dmaoffset( &SPI1RxBuffA );
    DMA1STAH = __builtin_dmapage( &SPI1RxBuffA );

    DMA1STBL = __builtin_dmaoffset( &SPI1RxBuffB );
    DMA1STBH = __builtin_dmapage( &SPI1RxBuffB );

    #else
    DMA1STAL = ( uint16_t ) & SPI1RxBuffA;
    DMA1STAH = ( uint16_t ) & SPI1RxBuffA;

    DMA1STBL = ( uint16_t ) & SPI1RxBuffB;
    DMA1STBH = ( uint16_t ) & SPI1RxBuffB;
    #endif
    IFS0bits.DMA1IF = 0;    // Clear DMA interrupt
    IEC0bits.DMA1IE = 1;    // Enable DMA interrupt
    DMA1CONbits.CHEN = 1;   // Enable DMA Channel
}

/******************************************************************************
 * Function:        void CfgSpi1Master(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        1.This function is used to configure the SPI1 module to operate in a loopback.
 *                  2.SPI1 initialization/configuration function.
 *****************************************************************************/
void CfgSpi1Master( void )
{
    /* Configure SPI1CON register to the following
      Idle state for clock is a low level (SPI1CON1bits.CKP=?)
 •    Data out on Active to Idle Edge (SPI1CON1bits.CKE=?)
 •    16-bit data transfer mode (SPI1CON1bits.MODE16=?)
 •    Enable Master mode (SPI1CON1bits.MSTEN=?)
 •    Set Primary Pre-scalar for 4:1 ratio (SPI1CON1bits.PPRE=?)
 •    Set Secondary Pre-scalar for 2:1 ratio (SPI1CON1bits.SPRE=?)
 •    Enable SDO output (SPI1CON1bits.DISSDO=?)
 •    Enable SCK output (SPI1CON1bits.DISSCK=?)
        
*/
    ANSELAbits.ANSA4 = 0;
    ANSELAbits.ANSA9 = 0;
    ANSELCbits.ANSC3 = 0;
    ANSELEbits.ANSE1 = 0;

    SPI1CON1bits.CKP = 0;
    SPI1CON1bits.CKE = 0;
    SPI1CON1bits.MODE16 = 1;
    SPI1CON1bits.MSTEN = 1;
    SPI1CON1bits.SPRE = 0;
    SPI1CON1bits.PPRE = 0;
    SPI1CON1bits.DISSDO = 0;
    SPI1CON1bits.DISSCK = 0;

    // •    Enable SPI module (SPI1STATbits.SPIEN=?)
    SPI1STATbits.SPIEN = 1;

    // Force First word after Enabling SPI
    DMA0REQbits.FORCE = 1;
    while( DMA0REQbits.FORCE == 1 );
}

/******************************************************************************
 * Function:        void InitSPIBuff (void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        1.This function is used to initialize two separate buffers to store the incoming data
 *                    to the SPI and also to initialize two buffers for the SPI to transmit data.
 *                  2.Buffer initialization function.
 *****************************************************************************/
void InitSPIBuff( void )
{
    uint16_t    i;
    for( i = 0; i < 16; i++ )
    {
        SPI1TxBuffA[i] = i;
    }

    for( i = 0; i < 16; i++ )
    {
        SPI1TxBuffB[i] = 16 + i;
    }

    for( i = 0; i < 16; i++ )
    {
        SPI1RxBuffA[i] = 0xDEED;
        SPI1RxBuffB[i] = 0xDEED;
    }
}

/*=============================================================================
Interrupt Service Routines.
=============================================================================*/
/******************************************************************************
 * Function:        void  __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Clears the DMA0 Interrupt flag
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _DMA0Interrupt( void )
{
    IFS0bits.DMA0IF = 0;    // Clear the DMA0 Interrupt Flag;
    DMA0CONbits.CHEN = 1;   // Enable DMA Channel
}

/******************************************************************************
 * Function:        void __attribute__((interrupt, auto_psv)) _DMA1Interrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        1.Clears the DMA1 Interrupt flag
 *                  2.Switches the Receiving buffer between  BuffA and BuffB
 *****************************************************************************/
void __attribute__ ( (interrupt, auto_psv) ) _DMA1Interrupt( void )
{
    if( rxDmaBuffer == 0 )
    {
        ProcessSpiRxSamples( SPI1RxBuffA );
    }
    else
    {
        ProcessSpiRxSamples( SPI1RxBuffB );
    }

    rxDmaBuffer ^= 1;

    IFS0bits.DMA1IF = 0;    // Clear the DMA1 Interrupt Flag
}

/******************************************************************************
 * Function:        void ProcessSpiRxSamples(__eds__ uint16_t * SpiRxBuffer)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Does nothing with the data.
 *  *****************************************************************************/
void ProcessSpiRxSamples( __eds__ uint16_t *SpiRxBuffer )
{
    /* Do something with SPI Samples */
}

/*******************************************************************************
 End of File
*/

/*******************************************************************************
  UART and DMA configuration source file

  Company:
    Microchip Technology Inc.

  File Name:
    uart1drv.c

  Summary:
    Configures UART1 for operation with DMA.

  Description:
    This source file configures the UART1 module to operate in a loopback mode and also configures
    the DMA module to store the received data and also to provide the data for transmit by the
    UART module. For devices that have a dedicated DMA RAM, the DMA RAM buffers are used for transmit
    and receive else the normal RAM itself is used. 
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
#include <uart1drv.h>

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
#pragma config POSCMD = NONE        // Primary Oscillator Mode Select bits (Primary Oscillator disabled)
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
#define FCY         60000000
#define BAUDRATE    9600
#define BRGVAL      ( (FCY / BAUDRATE) / 16 ) - 1

//uncomment next line if __eds__ is supported
//#define _HAS_DMA_
// Prototype declaration of function
void ProcessUartRxSamples( __eds__ uint16_t * UartRxBuffer );
#ifdef _HAS_DMA_
__eds__ unsigned int    uart1RxBuffA[16] __attribute__( (eds, space(dma)) );
__eds__ unsigned int    uart1RxBuffB[16] __attribute__( (eds, space(dma)) );
__eds__ unsigned int    uart1TxBuffA[16] __attribute__( (eds, space(dma)) );
__eds__ unsigned int    uart1TxBuffB[16] __attribute__( (eds, space(dma)) );

#else
uint16_t                uart1RxBuffA[16] __attribute__( (space(xmemory)) );
uint16_t                uart1RxBuffB[16] __attribute__( (space(xmemory)) );
uint16_t                uart1TxBuffA[16] __attribute__( (space(xmemory)) );
uint16_t                uart1TxBuffB[16] __attribute__( (space(xmemory)) );
#endif

/******************************************************************************
 * Function:        void CfgDma0UartTx(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        1.DMA0 initialization/configuration function
 *                  2.This function is used to configure the DMA0 module to operate with the UART1 transmitter
 *                    and transfer the contents from RAM to the UART module in continuous ping pong mode with
 *                    Register indirect addressing.
 *****************************************************************************/
void CfgDma0UartTx( void )
{
    DMA0CON = 0x2002;
    DMA0CNT = 15;
    DMA0REQ = 0x00C;

    DMA0PAD = ( volatile uint16_t ) &U1TXREG;

    #ifdef _HAS_DMA_
    DMA0STAL = __builtin_dmaoffset( &uart1TxBuffA );
    DMA0STAH = __builtin_dmapage( &uart1TxBuffA );

    DMA0STBL = __builtin_dmaoffset( &uart1TxBuffB );
    DMA0STBH = __builtin_dmapage( &uart1TxBuffB );
    #else
    DMA0STAL = ( uint16_t ) & uart1TxBuffA;
    DMA0STAH = ( uint16_t ) & uart1TxBuffA;

    DMA0STBL = ( uint16_t ) & uart1TxBuffB;
    DMA0STBH = ( uint16_t ) & uart1TxBuffB;
    #endif
    IFS0bits.DMA0IF = 0;    // Clear DMA interrupt
    IEC0bits.DMA0IE = 1;    // Enable DMA interrupt
    DMA0CONbits.CHEN = 1;   // Enable DMA Channel
}

/******************************************************************************
 * Function:        void CfgDma1UartRx(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        1.DMA1 initialization/configuration function.
 *                  2.This function is used to configure the DMA1 module to operate with the UART1 receiver and
 *                    transfer the contents from UART module to the RAM in continuous ping pong mode
 *                    with Register indirect addressing.
 *****************************************************************************/
void CfgDma1UartRx( void )
{
    DMA1CON = 0x0002;
    DMA1CNT = 15;
    DMA1REQ = 0x00B;

    DMA1PAD = ( volatile uint16_t ) &U1RXREG;
    #ifdef _HAS_DMA_
    DMA1STAL = __builtin_dmaoffset( &uart1RxBuffA );
    DMA1STAH = __builtin_dmapage( &uart1RxBuffA );

    DMA1STBL = __builtin_dmaoffset( &uart1RxBuffB );
    DMA1STBH = __builtin_dmapage( &uart1RxBuffB );

    #else
    DMA1STAL = ( uint16_t ) & uart1RxBuffA;
    DMA1STAH = ( uint16_t ) & uart1RxBuffA;

    DMA1STBL = ( uint16_t ) & uart1RxBuffB;
    DMA1STBH = ( uint16_t ) & uart1RxBuffB;
    #endif
    IFS0bits.DMA1IF = 0;    // Clear DMA interrupt
    IEC0bits.DMA1IE = 1;    // Enable DMA interrupt
    DMA1CONbits.CHEN = 1;   // Enable DMA Channel
}

/******************************************************************************
 * Function:        void CfgUart1(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        1.UART1 initialization/configuration function.
 *                  2.This function is used to configure the UART1 module to operate in the loopback mode at 9600 baud.
 *****************************************************************************/
void CfgUart1( void )
{
    // Enable Loop Back Mode
    U1MODEbits.LPBACK = 1;

    // •    Configure U2BRG register for 57600 bit rate, note Fcy=30Mhz (U2BRG=?)
    //      U1BRD=((FCY/BAUDRATE)/16)-1
    U1BRG = BRGVAL;

    // Configure U1MODE register to the following
    // •    No Parity and 8-bit data (U1MODEbits.PDSEL=?)
    // •    1-Stop Bit (U1MODEbits.STSEL=?)
    // •    Enable UART (U1MODEbits.UARTEN=?)
    U1MODEbits.PDSEL = 0;
    U1MODEbits.STSEL = 0;
    U1MODEbits.UARTEN = 1;

    // Configure U1STA register to the following
    // •    Interrupt on data transfer to the Transmit Shift register (U1STAbits.UTXISEL=?)
    // •    Interrupt on data transfer to the Receive register (U1STAbits.URXISEL=?)
    // •    Enable UART Transmit Module (U2STAbits.UTXEN=?)
    U1STAbits.UTXISEL0 = 0;
    U1STAbits.UTXISEL1 = 0;
    U1STAbits.URXISEL = 0;
    U1STAbits.UTXEN = 1;
}

/******************************************************************************
 * Function:        void InitUartBuff(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        1.Buffer initialization function.
 *                  2.This function is used to initialize two separate buffers to store the incoming data to the UART
 *                    and also to initialize two buffers for the UART to transmit data.
 *****************************************************************************/
void InitUartBuff( void )
{
    uint16_t    i;
    for( i = 0; i < 16; i++ )
    {
        uart1TxBuffA[i] = i;
    }

    for( i = 0; i < 16; i++ )
    {
        uart1TxBuffB[i] = 16 + i;
    }

    for( i = 0; i < 16; i++ )
    {
        uart1RxBuffA[i] = 0xDEED;
        uart1RxBuffB[i] = 0xDEED;
    }
}

/*=============================================================================
Interrupt Service Routines.
=============================================================================*/
// Variable used to swith the Rx buffer A and B
uint16_t    rxDmaBuffer = 0;

/******************************************************************************
 * Function:        void __attribute__ ((interrupt, no_auto_psv)) _DMA0Interrupt(void)
 *
 * PreCondition:    None.
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Clears the trap flag if DMA0IF bit is set.
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _DMA0Interrupt( void )
{
    IFS0bits.DMA0IF = 0;    //Clear the DMA0 Interrupt Flag;
}

/******************************************************************************
 * Function:        void __attribute__ ((interrupt, no_auto_psv)) _DMA1Interrupt(void)
 *
 * PreCondition:    None.
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Clears the trap flag if DMA1IF bit is set.
 *****************************************************************************/
void __attribute__ ( (interrupt, auto_psv) ) _DMA1Interrupt( void )
{
    if( rxDmaBuffer == 0 )
    {
        ProcessUartRxSamples( uart1RxBuffA );
    }
    else
    {
        ProcessUartRxSamples( uart1RxBuffB );
    }

    rxDmaBuffer ^= 1;

    IFS0bits.DMA1IF = 0;    // Clear the DMA0 Interrupt Flag
}

/******************************************************************************
 * Function:        void __attribute__ ((interrupt, no_auto_psv)) ProcessUartRxSamples(void)
 *
 * PreCondition:    None.
 *
 * Input:           UartRxBuffer
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Just a Dummy function, does nothing
 *****************************************************************************/
void ProcessUartRxSamples( __eds__ uint16_t *UartRxBuffer )
{
    /* Do something with SPI Samples */
}

/*
 *  End of File
*/

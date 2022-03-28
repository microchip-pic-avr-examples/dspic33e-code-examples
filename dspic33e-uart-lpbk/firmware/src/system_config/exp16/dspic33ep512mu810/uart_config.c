/*******************************************************************************
  UART Configuration source file

  Company:
    Microchip Technology Inc.

  File Name:
    uart_config.c

  Summary:
    Configures the UART2 for operation with DMA.

  Description:
    This source file configures the UART2 module acting as a Receiver
    and transmitter.2 DMA channels are configured; one for transmit
    and other for receive bytes.
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

// DSPIC33EP512MU810 Configuration Bit Settings
// FICD
#pragma config ICS = PGD1           // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config JTAGEN = OFF         // JTAG Enable bit (JTAG is disabled)

// FPOR
#pragma config BOREN = ON           // Brown-out Reset (BOR) Detection Enable bit (BOR is enabled)
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

// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************
#define FCY         60000000
#define BAUDRATE    9600
#define BRGVAL      ( (FCY / BAUDRATE) / 16 ) - 1

//uncomment next line if __eds__ is supported
//#define _HAS_DMA_
//  Allocate two buffers for DMA transfers
#ifdef _HAS_DMA_
__eds__ uint16_t bufferA[8] __attribute__( (eds, space(dma)) );
__eds__ uint16_t    bufferB[8] __attribute__( (eds, space(dma)) );
#else
uint16_t            bufferA[8] __attribute__( (space(xmemory)) );
uint16_t            bufferB[8] __attribute__( (space(xmemory)) );
#endif

/******************************************************************************
 * Function:        void CfgUart2(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        UART initialization/configuration function.
 *                  This function is used to configure the UART module to
 *                  operate at 9600 baud with 1-stop bit,no parity and 8-bit data.
 *****************************************************************************/
void CfgUart2( void )
{
    U2MODEbits.STSEL = 0;   // 1-stop bit
    U2MODEbits.PDSEL = 0;   // No Parity, 8-data bits
    U2MODEbits.ABAUD = 0;   // Autobaud Disabled
    U2BRG = BRGVAL;         // BAUD Rate Setting for 9600

    //  Configure UART for DMA transfers
    U2STAbits.UTXISEL0 = 0; // Interrupt after one Tx character is transmitted
    U2STAbits.UTXISEL1 = 0;
    U2STAbits.URXISEL = 0;  // Interrupt after one RX character is received

    //  Enable UART Rx and Tx
    U2MODEbits.UARTEN = 1;  // Enable UART
    U2STAbits.UTXEN = 1;    // Enable UART Tx

    //configure pins
    //R2TX
    // The PPS configuration varies from device to device. Refer the datasheet of the device being used and
    // use the appropriate values for the RPINR/RPOR registers.
    RPOR9bits.RP101R = 3;   //RF5 as U2TX
    _U2RXR = 100;           //RF4 as U2RX
    IEC4bits.U2EIE = 0;
}

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
 * Overview:        DMA0 initialization/configuration function for transmitter.
 *                  This function is used to configure the DMA0 module to associate
 *                  itself with the UART2 transmitter and generate interrupts after
 *                  8 transfers from the DMA buffer.
 *****************************************************************************/
void CfgDma0UartTx( void )
{
    //  Associate DMA Channel 0 with UART Tx
    //********************************************************************************/
    DMA0REQ = 0x001F;       // Select UART2 Transmitter
    DMA0PAD = ( volatile uint16_t ) &U2TXREG;

    //  Configure DMA Channel 0 to:
    //  Transfer data from RAM to UART
    //  One-Shot mode
    //  Register Indirect with Post-Increment
    //  Using single buffer
    //  8 transfers per buffer
    //  Transfer words
    DMA0CONbits.AMODE = 0;
    DMA0CONbits.MODE = 1;
    DMA0CONbits.DIR = 1;
    DMA0CONbits.SIZE = 0;
    DMA0CNT = 7;            // 8 DMA requests

    // Associate one buffer with Channel 0 for one-shot operation
    #ifdef _HAS_DMA_
    DMA0STAL = __builtin_dmaoffset( &bufferA );
    DMA0STAH = __builtin_dmapage( &bufferA );

    #else
    DMA0STAL = ( uint16_t ) & bufferA;
    DMA0STAH = ( uint16_t ) & bufferA;
    #endif

    //    Enable DMA Interrupts
    IFS0bits.DMA0IF = 0;    // Clear DMA Interrupt Flag
    IEC0bits.DMA0IE = 1;    // Enable DMA interrupt
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
 * Overview:        DMA1 initialization/configuration function for receive.
 *                  This function is used to configure the DMA1 module to
 *                  operate with the UART2 receiver and transfer data
 *                  continuously from the UART to the DMA buffer..
 *****************************************************************************/
void CfgDma1UartRx( void )
{
    //  Associate DMA Channel 1 with UART Rx
    DMA1REQ = 0x001E;                   // Select UART2 Receiver
    DMA1PAD = ( volatile uint16_t ) &U2RXREG;

    //  Configure DMA Channel 1 to:
    //  Transfer data from UART to RAM Continuously
    //  Register Indirect with Post-Increment
    //  Using two ‘ping-pong’ buffers
    //  8 transfers per buffer
    //  Transfer words
    DMA1CONbits.AMODE = 0;
    DMA1CONbits.MODE = 2;
    DMA1CONbits.DIR = 0;
    DMA1CONbits.SIZE = 0;
    DMA1CNT = 7;                        // 8 DMA requests

    //  Associate two buffers with Channel 1 for ‘Ping-Pong’ operation
    #ifdef _HAS_DMA_
    DMA1STAL = __builtin_dmaoffset( &bufferA );
    DMA1STAH = __builtin_dmapage( &bufferA );

    DMA1STBL = __builtin_dmaoffset( &bufferB );
    DMA1STBH = __builtin_dmapage( &bufferB );

    #else
    DMA1STAL = ( uint16_t ) & bufferA;
    DMA1STAH = ( uint16_t ) & bufferA;

    DMA1STBL = ( uint16_t ) & bufferB;
    DMA1STBH = ( uint16_t ) & bufferB;
    #endif

    //    Enable DMA Interrupts
    IFS0bits.DMA1IF = 0;                // Clear DMA interrupt
    IEC0bits.DMA1IE = 1;                // Enable DMA interrupt

    //  Enable DMA Channel 1 to receive UART data
    DMA1CONbits.CHEN = 1;               // Enable DMA Channel
}

//    Setup DMA interrupt handlers
//    Force transmit after 8 words are received
/******************************************************************************
 * Function:        void __attribute__ ((interrupt, no_auto_psv)) _DMA0Interrupt(void)
 *
 * PreCondition:    UART Module must be Initialized with transmit interrupt enabled.
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Clear the DMA0 Interrupt Flag
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _DMA0Interrupt( void )
{
    IFS0bits.DMA0IF = 0;                // Clear the DMA0 Interrupt Flag;
}

/******************************************************************************
 * Function:        void __attribute__ ((interrupt, no_auto_psv)) _DMA1Interrupt(void)
 *
 * PreCondition:    UART Module must be Initialized with transmit interrupt enabled.
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        check to see if buffer contains 8 characters, if it's more thatn
 *                  8-character transmit over to Hyper terminal
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _DMA1Interrupt( void )
{
    static uint16_t bufferCount = 0;    // Keep record of which buffer contains Rx Data
    if( bufferCount == 0 )
    {
        // Point DMA 0 to data to be transmitted
        #ifdef _HAS_DMA_
        DMA0STAL = __builtin_dmaoffset( &bufferA );
        DMA0STAH = __builtin_dmapage( &bufferA );

        #else
        DMA0STAL = ( uint16_t ) & bufferA;
        DMA0STAH = ( uint16_t ) & bufferA;
        #endif
    }
    else
    {
        // Point DMA 0 to data to be transmitted
        #ifdef _HAS_DMA_
        DMA0STAL = __builtin_dmaoffset( &bufferB );
        DMA0STAH = __builtin_dmapage( &bufferB );

        #else
        DMA0STAL = ( uint16_t ) & bufferB;
        DMA0STAH = ( uint16_t ) & bufferB;
        #endif
    }

    DMA0CONbits.CHEN = 1;               // Re-enable DMA0 Channel
    DMA0REQbits.FORCE = 1;              // Manual mode: Kick-start the first transfer
    bufferCount ^= 1;
    IFS0bits.DMA1IF = 0;                // Clear the DMA1 Interrupt Flag
}

/******************************************************************************
 * Function:        void __attribute__ ((interrupt, no_auto_psv)) _U2ErrInterrupt(void)
 *
 * PreCondition:    UART Module must be Initialized with transmit interrupt enabled.
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Clear the UART2 Error Interrupt Flag
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _U2ErrInterrupt( void )
{
    IFS4bits.U2EIF = 0;                 // Clear the UART2 Error Interrupt Flag
}

/*******************************************************************************
 End of File
*/

/*******************************************************************************
  ECAN Configuration source file

  Company:
    Microchip Technology Inc.

  File Name:
    ecan2_config.c

  Summary:
    Initializes and configures the ECAN1 and DMA modules.

  Description:
    This source file initializes the DMA and configures two DMA channels one
    each for transmission and reception. The ECAN is also initialized, its clock
    configured to be the system clock itself and the filters are also configured
    to accept a particular message.
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
#include "ecan2_config.h"

/******************************************************************************
 * Function:      void DMA1Init(void)
 *
 * PreCondition:  None
 *
 * Input:         None
 *
 * Output:        None
 *
 * Side Effects:  None
 *
 * Overview:      DMA1 initialization/configuration function.
 *                Direction: Read from RAM and write to the C1TXD register
 *                AMODE: Register indirect with post increment
 *                MODE: Continuous, Ping-Pong Mode
 *                IRQ: ECAN2 Transmit Interrupt
 *****************************************************************************/
void DMA1Init( void )
{
    DMAPWC = 0;
    DMARQC = 0;
    DMA1CON = 0x2020;
    DMA1PAD = ( int16_t ) & C2TXD;  /* ECAN 2 (C2TXD) */
    DMA1CNT = 0x0007;
    DMA1REQ = 0x0047;               /* ECAN 2 Transmit */

    #ifdef _HAS_DMA_
    DMA1STAL = __builtin_dmaoffset( ecan2msgBuf );
    DMA1STAH = __builtin_dmapage( ecan2msgBuf );
    #else
    DMA1STAL = (uint16_t)(int_least24_t)(&ecan2msgBuf);
    DMA1STAH = 0;
    #endif
    DMA1CONbits.CHEN = 1;
}

/******************************************************************************
 * Function:      void DMA3Init(void)
 *
 * PreCondition:  None
 *
 * Input:         None
 *
 * Output:        None
 *
 * Side Effects:  None
 *
 * Overview:      DMA3 initialization/configuration function.
 *                Direction: Read from RAM and write to the C1RXD register
 *                AMODE: Register indirect with post increment
 *                MODE: Continuous, Ping-Pong Mode
 *                IRQ: ECAN2 Transmit Interrupt
 *****************************************************************************/
void DMA3Init( void )
{
    //	 DMACS0=0;
    DMAPWC = 0;
    DMARQC = 0;
    DMA3CON = 0x0020;
    DMA3PAD = ( int16_t ) & C2RXD;  /* ECAN 2 (C2RXD) */
    DMA3CNT = 0x0007;
    DMA3REQ = 0x0037;               /* ECAN 2 Receive */

    #ifdef _HAS_DMA_
    DMA3STAL = __builtin_dmaoffset( ecan2msgBuf );
    DMA3STAH = __builtin_dmapage( ecan2msgBuf );
    #else
    DMA3STAL = (uint16_t)(int_least24_t)(&ecan2msgBuf);
    DMA3STAH = 0;
    #endif
    DMA3CONbits.CHEN = 1;
}

/******************************************************************************
 * Function:      void Ecan2ClkInit(void)
 *
 * PreCondition:  None
 *
 * Input:         None
 *
 * Output:        None
 *
 * Side Effects:  None
 *
 * Overview:      ECAN2 clock initialization function
 *                This function is used to configure the clock used for the
 *                ECAN2 module during transmission/reception.
 *****************************************************************************/
void Ecan2ClkInit( void )
{
    /* FCAN is selected to be FCY*/
    // FCAN = FCY = 40MHz
    //	C2CTRL1bits.CANCKS = 0x1;
    /*
Bit Time = (Sync Segment + Propagation Delay + Phase Segment 1 + Phase Segment 2)=20*TQ
Phase Segment 1 = 8TQ
Phase Segment 2 = 6Tq
Propagation Delay = 5Tq
Sync Segment = 1TQ
CiCFG1<BRP> =(FCAN /(2 ×N×FBAUD))– 1
*/
    /* Synchronization Jump Width set to 4 TQ */
    C2CFG1bits.SJW = 0x3;

    /* Baud Rate Prescaler */
    C2CFG1bits.BRP = BRP_VAL;

    /* Phase Segment 1 time is 8 TQ */
    C2CFG2bits.SEG1PH = 0x7;

    /* Phase Segment 2 time is set to be programmable */
    C2CFG2bits.SEG2PHTS = 0x1;

    /* Phase Segment 2 time is 6 TQ */
    C2CFG2bits.SEG2PH = 0x5;

    /* Propagation Segment time is 5 TQ */
    C2CFG2bits.PRSEG = 0x4;

    /* Bus line is sampled three times at the sample point */
    C2CFG2bits.SAM = 0x1;
}

/******************************************************************************
 * Function:     void Ecan2Init(void)
 *
 * PreCondition:  None
 *
 * Input:         None
 *
 * Output:        None
 *
 * Side Effects:  None
 *
 * Overview:      ECAN2 initialization function.This function is used to
 *                initialize the ECAN1 module by configuring the message
 *                buffers, and the acceptance filters and
 *                setting appropriate masks for the same.
 *****************************************************************************/
void Ecan2Init( void )
{
    /* Request Configuration Mode */
    C2CTRL1bits.REQOP = 4;
    while( C2CTRL1bits.OPMODE != 4 );

    Ecan2ClkInit();

    C2FCTRLbits.DMABS = 0b000;     /* 4 CAN Message Buffers in DMA RAM */

    /*	Filter Configuration

	Ecan2WriteRxAcptFilter(int16_t n, long identifier, unsigned int16_t exide,unsigned int16_t bufPnt,unsigned int16_t maskSel);

	n = 0 to 15 -> Filter number

	identifier -> SID <10:0> : EID <17:0> 

	exide = 0 -> Match messages with standard identifier addresses 
	exide = 1 -> Match messages with extended identifier addresses 

	bufPnt = 0 to 14  -> RX Buffer 0 to 14
	bufPnt = 15 -> RX FIFO Buffer

	maskSel = 0	->	Acceptance Mask 0 register contains mask
	maskSel = 1	->	Acceptance Mask 1 register contains mask
	maskSel = 2	->	Acceptance Mask 2 register contains mask
	maskSel = 3	->	No Mask Selection
	
*/
    Ecan2WriteRxAcptFilter( 1, 0x1FFEFFFF, 1, 0, 0 );

    /*	Mask Configuration

	Ecan2WriteRxAcptMask(int16_t m, long identifierMask, unsigned int16_t mide, unsigned int16_t exide);

	m = 0 to 2 -> Mask Number

	identifier -> SID <10:0> : EID <17:0> 

	mide = 0 -> Match either standard or extended address message if filters match 
	mide = 1 -> Match only message types that correpond to 'exide' bit in filter

	exide = 0 -> Match messages with standard identifier addresses 
	exide = 1 -> Match messages with extended identifier addresses
	
*/
    Ecan2WriteRxAcptMask( 1, 0x1FFFFFFF, 1, 1 );

    /* Enter Normal Mode */
    C2CTRL1bits.REQOP = 0;
    while( C2CTRL1bits.OPMODE != 0 );

    /* ECAN transmit/receive message control */
    C2RXFUL1 = C2RXFUL2 = C2RXOVF1 = C2RXOVF2 = 0x0000;
    C2TR01CONbits.TXEN0 = 1;        /* ECAN2, Buffer 0 is a Transmit Buffer */
    C2TR01CONbits.TX0PRI = 0b11;   /* Message Buffer 0 Priority Level */

    /* Setup I/O pins */
    // The PPS configuration varies from device to device. Refer the datasheet of the device being used and
    // use the appropriate values for the RPINR/RPOR registers.
    RPINR26bits.C2RXR = 112;        //set CAN2 RX to RP112		(90)
    RPOR13bits.RP113R = 15;         //set CAN2TX to RP113		(89)
}

/*******************************************************************************
 End of File
*/

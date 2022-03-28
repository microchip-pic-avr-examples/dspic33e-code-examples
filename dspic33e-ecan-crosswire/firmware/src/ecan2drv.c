/*******************************************************************************
  ECAN buffer and filter configuration source file

  Company:
    Microchip Technology Inc.

  File Name:
    ecan2drv.c

  Summary:
    Configures the message acceptance filters and the data buffers.

  Description:
    This source file sets the mask for the acceptance filters for the incoming
    data and also configures the data buffers by specifying identifiers for them.
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
#include "ecan2_config.h"

/******************************************************************************
 * Function:     void Ecan2WriteRxAcptFilter(int16_t n, int32_t identifier,
 *               uint16_t exide,uint16_t bufPnt,uint16_t maskSel)
 *
 * PreCondition:  None
 *
 * Input:         n-> Filter number [0-15]
 *                identifier-> Bit ordering is given below
 *                Filter Identifier (29-bits) :
 *                0b000f ffff ffff ffff ffff ffff ffff ffff
 *                     |____________|_____________________|
 *                        SID10:0          EID17:0
 *
 *               Filter Identifier (11-bits) :
 *               0b0000 0000 0000 0000 0000 0fff ffff ffff
 *                                           |___________|
 *                                             SID10:
 *               exide -> "0" for standard identifier
 *                        "1" for Extended identifier
 *               bufPnt -> Message buffer to store filtered message [0-15]
 *               maskSel -> Optinal Masking of identifier bits [0-3]
 *
 * Output:        None
 *
 * Side Effects:  None
 *
 * Overview:      Configures Acceptance filter "n" for ECAN2.
 *****************************************************************************/
void Ecan2WriteRxAcptFilter( int16_t n, int32_t identifier, uint16_t exide, uint16_t bufPnt, uint16_t maskSel )
{
    unsigned long   sid10_0 = 0;

    unsigned long   eid15_0 = 0;

    unsigned long   eid17_16 = 0;
    unsigned int    *sidRegAddr;
    unsigned int    *bufPntRegAddr;
    unsigned int    *maskSelRegAddr;
    unsigned int    *fltEnRegAddr;

    C2CTRL1bits.WIN = 1;

    // Obtain the Address of CiRXFnSID, CiBUFPNTn, CiFMSKSELn and CiFEN register for a given filter number "n"
    sidRegAddr = ( unsigned int * ) ( &C2RXF0SID + (n << 1) );
    bufPntRegAddr = ( unsigned int * ) ( &C2BUFPNT1 + (n >> 2) );
    maskSelRegAddr = ( unsigned int * ) ( &C2FMSKSEL1 + (n >> 3) );
    fltEnRegAddr = ( unsigned int * ) ( &C2FEN1 );

    // Bit-filed manupulation to write to Filter identifier register
    if( exide == 1 )
    {   // Filter Extended Identifier
        eid15_0 = ( identifier & 0xFFFF );
        eid17_16 = ( identifier >> 16 ) & 0x3;
        sid10_0 = ( identifier >> 18 ) & 0x7FF;

        *sidRegAddr = ( ((sid10_0) << 5) + 0x8 ) + eid17_16;    // Write to CiRXFnSID Register
        *( sidRegAddr + 1 ) = eid15_0;  // Write to CiRXFnEID Register
    }
    else
    {   // Filter Standard Identifier
        sid10_0 = ( identifier & 0x7FF );
        *sidRegAddr = ( sid10_0 ) << 5; // Write to CiRXFnSID Register
        *( sidRegAddr + 1 ) = 0;        // Write to CiRXFnEID Register
    }

    *bufPntRegAddr = ( *bufPntRegAddr ) & ( 0xFFFF - (0xF << (4 * (n & 3))) );      // clear nibble
    *bufPntRegAddr = ( (bufPnt << (4 * (n & 3))) | (*bufPntRegAddr) );              // Write to C1BUFPNTn Register
    *maskSelRegAddr = ( *maskSelRegAddr ) & ( 0xFFFF - (0x3 << ((n & 7) * 2)) );    // clear 2 bits
    *maskSelRegAddr = ( (maskSel << (2 * (n & 7))) | (*maskSelRegAddr) );           // Write to C1FMSKSELn Register
    *fltEnRegAddr = ( (0x1 << n) | (*fltEnRegAddr) );   // Write to C1FEN1 Register
    C2CTRL1bits.WIN = 0;
}

/******************************************************************************
 * Function:     void Ecan2WriteRxAcptMask(int16_t m, int32_t identifier,
 *               uint16_t mide, uint16_t exide)
 *
 * PreCondition:  None
 *
 * Input:        m-> Mask number [0-2]
                 identifier-> Bit ordering is given below n-> Filter number [0-15]
 *                identifier-> Bit ordering is given below
 *                Filter mask Identifier (29-bits) :
 *                0b000f ffff ffff ffff ffff ffff ffff ffff
 *                     |____________|_____________________|
 *                        SID10:0          EID17:0
 *
 *               Filter mask Identifier (11-bits) :
 *               0b0000 0000 0000 0000 0000 0fff ffff ffff
 *                                           |___________|
 *                                             SID10:
 *               mide ->  "0"  Match either standard or extended address message
 *                             if filters match
 *                        "1"  Match only message types that correpond to
 *                             'exide' bit in filter
 *
 * Output:        None
 *
 * Side Effects:  None
 *
 * Overview:      Configures Acceptance filter "n" for ECAN2.
 *****************************************************************************/
void Ecan2WriteRxAcptMask( int16_t m, int32_t identifier, uint16_t mide, uint16_t exide )
{
    unsigned long   sid10_0 = 0;

    unsigned long   eid15_0 = 0;

    unsigned long   eid17_16 = 0;
    unsigned int    *maskRegAddr;

    C2CTRL1bits.WIN = 1;

    // Obtain the Address of CiRXMmSID register for given Mask number "m"
    maskRegAddr = ( unsigned int * ) ( &C2RXM0SID + (m << 1) );

    // Bit-filed manupulation to write to Filter Mask register
    if( exide == 1 )
    {   // Filter Extended Identifier
        eid15_0 = ( identifier & 0xFFFF );
        eid17_16 = ( identifier >> 16 ) & 0x3;
        sid10_0 = ( identifier >> 18 ) & 0x7FF;

        if( mide == 1 )
        {
            *maskRegAddr = ( (sid10_0) << 5 ) + 0x0008 + eid17_16;  // Write to CiRXMnSID Register
        }
        else
        {
            *maskRegAddr = ( (sid10_0) << 5 ) + eid17_16;           // Write to CiRXMnSID Register
        }

        *( maskRegAddr + 1 ) = eid15_0; // Write to CiRXMnEID Register
    }
    else
    {   // Filter Standard Identifier
        sid10_0 = ( identifier & 0x7FF );
        if( mide == 1 )
        {
            *maskRegAddr = ( (sid10_0) << 5 ) + 0x0008; // Write to CiRXMnSID Register
        }
        else
        {
            *maskRegAddr = ( sid10_0 ) << 5;            // Write to CiRXMnSID Register
        }

        *( maskRegAddr + 1 ) = 0;                       // Write to CiRXMnEID Register
    }

    C2CTRL1bits.WIN = 0;
}

/******************************************************************************
 * Function:     void Ecan2WriteTxMsgBufId(uint16_t buf, int32_t txIdentifier, uint16_t ide,
 *               uint16_t remoteTransmit)
 *
 * PreCondition:  None
 *
 * Input:        buf    -> Transmit Buffer Number
 *               txIdentifier ->
 *               Extended Identifier (29-bits):
 *                0b000f ffff ffff ffff ffff ffff ffff ffff
 *                     |____________|_____________________|
 *                        SID10:0          EID17:0
 *
 *               Standard Identifier (11-bits) :
 *               0b0000 0000 0000 0000 0000 0fff ffff ffff
 *                                           |___________|
 *                                             SID10:
 *                 Standard Message Format:
 *                                             Word0 : 0b000f ffff ffff ffff
 *                                                          |____________|||___
 *                                                             SID10:0   SRR   IDE
 *                                             Word1 : 0b0000 0000 0000 0000
 *                                                            |____________|
 *                                                               EID17:6
 *                                             Word2 : 0b0000 00f0 0000 ffff
 *                                                       |_____||           |__|
 *                                                       EID5:0 RTR         DLC
 *                Extended Message Format:
 *                                          Word0 : 0b000f ffff ffff ffff
 *                                                       |____________|||___
 *                                                          SID10:0   SRR   IDE
 *                                          Word1 : 0b0000 ffff ffff ffff
 *                                                         |____________|
 *                                                               EID17:6
 *                                          Word2 : 0bffff fff0 0000 ffff
 *                                                    |_____||           |__|
 *                                                    EID5:0 RTR         DLC
 *             ide -> "0"  Message will transmit standard identifier
 *                    "1"  Message will transmit extended identifier
 *
 *            remoteTransmit -> "0" Message transmitted is a normal message
 *                              "1" Message transmitted is a remote message
 *            Standard Message Format:
 *                                          Word0 : 0b000f ffff ffff ff1f
 *                                                       |____________|||___
 *                                                          SID10:0   SRR   IDE
 *                                          Word1 : 0b0000 0000 0000 0000
 *                                                         |____________|
 *                                                            EID17:6
 *                                          Word2 : 0b0000 0010 0000 ffff
 *                                                  |_____||           |__|
 *                                                  EID5:0 RTR         DLC
 *
 *         Extended Message Format:
 *                                         Word0 : 0b000f ffff ffff ff1f
 *                                                      |____________|||___
 *                                                        SID10:0   SRR   IDE
 *                                         Word1 : 0b0000 ffff ffff ffff
 *                                                        |____________|
 *                                                          EID17:6
 *                                         Word2 : 0bffff ff10 0000 ffff
 *                                                   |_____||           |__|
 *                                                   EID5:0 RTR         DLC
 *
 * Output:        None
 *
 * Side Effects:  None
 *
 * Overview:      This function configures ECAN2 message buffer.
 *****************************************************************************/
void Ecan2WriteTxMsgBufId( uint16_t buf, int32_t txIdentifier, uint16_t ide, uint16_t remoteTransmit )
{
    unsigned long   word0 = 0;

    unsigned long   word1 = 0;

    unsigned long   word2 = 0;
    unsigned long   sid10_0 = 0;
    unsigned long   eid5_0 = 0;
    unsigned long   eid17_6 = 0;

    if( ide )
    {
        eid5_0 = ( txIdentifier & 0x3F );
        eid17_6 = ( txIdentifier >> 6 ) & 0xFFF;
        sid10_0 = ( txIdentifier >> 18 ) & 0x7FF;
        word1 = eid17_6;
    }
    else
    {
        sid10_0 = ( txIdentifier & 0x7FF );
    }

    if( remoteTransmit == 1 )
    {   // Transmit Remote Frame
        word0 = ( (sid10_0 << 2) | ide | 0x2 );
        word2 = ( (eid5_0 << 10) | 0x0200 );
    }
    else
    {
        word0 = ( (sid10_0 << 2) | ide );
        word2 = ( eid5_0 << 10 );
    }

    // Obtain the Address of Transmit Buffer in DMA RAM for a given Transmit Buffer number
    if( ide )
    {
        ecan2msgBuf[buf][0] = ( word0 | 0x0002 );
    }
    else
    {
        ecan2msgBuf[buf][0] = word0;
    }

    ecan2msgBuf[buf][1] = word1;
    ecan2msgBuf[buf][2] = word2;
}

/******************************************************************************
 * Function:     void Ecan2WriteTxMsgBufData(uint16_t buf, uint16_t dataLength,
 *    uint16_t data1, uint16_t data2, uint16_t data3, uint16_t data4)
 *
 * PreCondition:  None
 *
 * Input:            buf    -> Transmit Buffer Number
 *              dataLength  -> data length in bytes.
 *    	        actual data -> data1, data2, data3, data4
 *
 * Output:        None
 *
 * Side Effects:  None
 *
 * Overview:      This function transmits ECAN data.
 *****************************************************************************/
void Ecan2WriteTxMsgBufData( uint16_t buf, uint16_t dataLength, uint16_t data1, uint16_t data2, uint16_t data3,
                             uint16_t data4 )
{
    ecan2msgBuf[buf][2] = ( (ecan2msgBuf[buf][2] & 0xFFF0) + dataLength );

    ecan2msgBuf[buf][3] = data1;
    ecan2msgBuf[buf][4] = data2;
    ecan2msgBuf[buf][5] = data3;
    ecan2msgBuf[buf][6] = data4;
}

/******************************************************************************
 * Function:     void Ecan1DisableRXFilter(int16_t n)
 *
 * PreCondition:  None
 *
 * Input:          n -> Filter number [0-15]
 *
 * Output:        None
 *
 * Side Effects:  None
 *
 * Overview:          Disables RX Acceptance Filter.
 *****************************************************************************/
void Ecan2DisableRXFilter( int16_t n )
{
    unsigned int    *fltEnRegAddr;
    C2CTRL1bits.WIN = 1;
    fltEnRegAddr = ( unsigned int * ) ( &C2FEN1 );
    *fltEnRegAddr = ( *fltEnRegAddr ) & ( 0xFFFF - (0x1 << n) );
    C2CTRL1bits.WIN = 0;
}

/*******************************************************************************
 End of File
*/

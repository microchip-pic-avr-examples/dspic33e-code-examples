/*******************************************************************************
  ECAN2 Driver Header file. 

  Company:
    Microchip Technology Inc.

  File Name:
    ecan2drv.h

  Summary:
    This file consists of Interface routines used to initiate ECAN2 transmission/reception.

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
#include <stdint.h>
#ifndef __ECAN2_DRV_H__
    #define __ECAN2_DRV_H__

    #ifdef __cplusplus      // Provide C++ Compatability
extern "C"
{
        #endif

    // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Routines
    // *****************************************************************************
    // *****************************************************************************
    extern void Ecan2WriteRxAcptFilter( int16_t n, int32_t identifier, uint16_t exide, uint16_t bufPnt, uint16_t maskSel );
    extern void Ecan2WriteRxAcptMask( int16_t m, int32_t identifierMask, uint16_t mide, uint16_t exide );

    extern void Ecan2WriteTxMsgBufId( uint16_t buf, int32_t txIdentifier, uint16_t ide, uint16_t remoteTransmit );
    extern void Ecan2WriteTxMsgBufData( uint16_t buf, uint16_t dataLength, uint16_t data1, uint16_t data2,
                                        uint16_t data3, uint16_t data4 );

    extern void Ecan2DisableRXFilter( int16_t n );

        #ifdef __cplusplus  // Provide C++ Compatibility
}

    #endif
#endif

/*******************************************************************************
 End of File
*/

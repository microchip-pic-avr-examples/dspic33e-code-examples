/*******************************************************************************
  ECAN Configuration header file

  Company:
    Microchip Technology Inc.

  File Name:
    ecan2_config.h

  Summary:
    Contains the prototypes for ECAN and DMA initialization and configuration functions.

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
#ifndef __ECAN2_CONFIG_H__
    #define __ECAN2_CONFIG_H__

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
    #include "ecan2drv.h"

    #ifdef __cplusplus      // Provide C++ Compatability
extern "C"
{
        #endif

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
  
  /* CAN Baud Rate Configuration         */
   #define FCAN    40000000
   #define BITRATE 1000000
   #define NTQ     20  // 20 Time Quanta in a Bit Time
   #define BRP_VAL ( (FCAN / (2 * NTQ * BITRATE)) - 1 )

    //#define _HAS_DMA_ //Uncomment this line if DMA is present in EDS in the selected device.
    /* CAN Message Buffer Configuration */
    #define ECAN2_MSG_BUF_LENGTH    32
// *****************************************************************************
// *****************************************************************************
// Section: Data Types
// *****************************************************************************
// *****************************************************************************
   typedef uint16_t                        ECAN2MSGBUF[ECAN2_MSG_BUF_LENGTH][8];

    
   #ifdef _HAS_DMA_
   __eds__ extern ECAN2MSGBUF ecan2msgBuf  __attribute__( (eds, space(dma), aligned(ECAN2_MSG_BUF_LENGTH * 16)) );
   #else
   __eds__ extern ECAN2MSGBUF ecan2msgBuf  __attribute__( (eds, space(xmemory), aligned(ECAN2_MSG_BUF_LENGTH * 16)) );
   #endif
// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************

    
    /* Function Prototype     */
    extern void Ecan2Init( void );
    extern void DMA1Init( void );
    extern void DMA3Init( void );

        #ifdef __cplusplus  // Provide C++ Compatibility
}

    #endif
#endif

/*******************************************************************************
 End of File
*/

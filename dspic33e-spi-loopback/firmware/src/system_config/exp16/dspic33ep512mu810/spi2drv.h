/*******************************************************************************
  SPI & DMA Configuration Header file

  Company:
    Microchip Technology Inc.

  File Name:
    spi2drv.h

  Summary:
    Contains prototypes for the SPI2 Configuration on the DMA Configuration functions.

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
#ifndef __SPI2DRV_H__
    #define __SPI2DRV_H__

    #ifdef __cplusplus      // Provide C++ Compatability
extern "C"
{
        #endif

    // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Routines
    // *****************************************************************************
    // *****************************************************************************
    // External Functions
    extern void CfgDma0SpiTx( void );
    extern void CfgDma1SpiRx( void );
    extern void CfgSpi2Master( void );
    extern void InitSPIBuff( void );

    extern void __attribute__ ( (__interrupt__) )   _DMA0Interrupt( void );
    extern void __attribute__ ( (__interrupt__) )   _DMA1Interrupt( void );
#ifdef TEST_MODE
extern uint16_t            SPI2RxBuffA[16] __attribute__( (space(xmemory)) );
extern uint16_t            SPI2RxBuffB[16] __attribute__( (space(xmemory)) );
extern uint16_t            SPI2TxBuffA[16] __attribute__( (space(xmemory)) );
extern uint16_t            SPI2TxBuffB[16] __attribute__( (space(xmemory)) );
#endif
        #ifdef __cplusplus  // Provide C++ Compatibility
}

    #endif
#endif

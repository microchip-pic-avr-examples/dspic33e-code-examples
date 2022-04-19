/*******************************************************************************
 
  Company:
    Microchip Technology Inc.

  File Name:
    rtdm.h

  Summary:
    RTDM API function definitions.

  Description:
    This file consists of the definitions for the RTDM process, 
    RTDM start, RTDM close fucntions that are called in user program.
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
#ifndef RTDM_H
    #define RTDM_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
    #include <stdint.h>
    #include <string.h>
    #include "uart.h"
    #include "rtdmuser.h"

    #ifdef __cplusplus      // Provide C++ Compatability
extern "C"
{
        #endif

    // *****************************************************************************
    // *****************************************************************************
    // Section: File Scope or Global Constants
    // *****************************************************************************
    // *****************************************************************************
        #if defined( __dsPIC33F__ ) || defined( __PIC24H__ ) || defined( __dsPIC33E__ ) || defined( __PIC24E__ ) || defined \
            ( __dsPIC30F1010__ ) || defined( __dsPIC30F2020__ ) || defined( __dsPIC30F2023__ )
            #define RTDM_UART_V2
        #elif defined( __dsPIC30F__ )
            #define RTDM_UART_V1
        #endif
        #if defined RTDM_FCY
            #if defined RTDM_BAUDRATE
                #define RTDM_BRG    ( RTDM_FCY / (16 * RTDM_BAUDRATE) ) - 1
            #else
                #error Cannot calculate BRG value. Please define RTDM_BAUDRATE in RTDMUSER.h file
            #endif
        #else
            #error Cannot calculate RTDM_BRG value. Please define RTDM_FCY in RTDMUSER.h file
        #endif
        #define RTDM_BAUDRATE_ACTUAL    ( RTDM_FCY / (16 * (RTDM_BRG + 1)) )
        #define RTDM_BAUD_ERROR         ( (RTDM_BAUDRATE_ACTUAL > RTDM_BAUDRATE) ? RTDM_BAUDRATE_ACTUAL - \
                              RTDM_BAUDRATE : RTDM_BAUDRATE - RTDM_BAUDRATE_ACTUAL )
        #define RTDM_BAUD_ERROR_PERCENT ( ((RTDM_BAUD_ERROR * 100) + (RTDM_BAUDRATE / 2)) / RTDM_BAUDRATE )
        #if ( RTDM_BAUD_ERROR_PERCENT > 2 )
            #error The value loaded to the BRG register produces a baud rate error higher than 2%
        #endif

    // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Routines
    // *****************************************************************************
    // *****************************************************************************
    int             RTDM_ProcessMsgs( void );
    int             RTDM_Close( void );
    int             RTDM_Start( void );
    unsigned int    RTDM_CumulativeCrc16( unsigned char *buf, unsigned int u16Length, unsigned int u16CRC );

        #ifdef __cplusplus  // Provide C++ Compatibility
}

    #endif
#endif

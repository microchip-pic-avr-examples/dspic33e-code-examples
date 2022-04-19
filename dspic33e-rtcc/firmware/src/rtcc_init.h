/*******************************************************************************
  ce409 RTSP API header file

  Company:
    Microchip Technology Inc.

  File Name:
    rtcc_init.h

  Summary:
    RTCC API function definitions.

  Description:
    This file consists of the definitions for RTCC initialization, read and RTCC unlock function definations.
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
#ifndef __RTCCINIT_H__
    #define __RTCCINIT_H__

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
    #include <stdint.h>

    #ifdef __cplusplus  // Provide C++ Compatability
extern "C"
{
        #endif

    // *****************************************************************************
    // *****************************************************************************
    // Section: File Scope or Global Constants
    // *****************************************************************************

    // *****************************************************************************
        #define RTCCLock()      RCFGCALbits.RTCWREN = 0;
        #define RTCCOn()        RCFGCALbits.RTCEN = 1;
        #define RTCCOff()       RCFGCALbits.RTCEN = 0;
        #define EnableSecOsc()  __builtin_write_OSCCONL( 0x02 );

    // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Routines
    // *****************************************************************************
    // *****************************************************************************
    extern void RtccInit( void );
    extern void RtccRead( void );
    extern void RTCCUnlock( void );
    extern void __attribute__ ( (interrupt, no_auto_psv) )  _RTCCInterrupt( void );
#ifdef TEST_MODE
    extern unsigned char test_flag;
#endif
        #ifdef __cplusplus  // Provide C++ Compatibility
}

    #endif
#endif

/*******************************************************************************
 End of File
*/

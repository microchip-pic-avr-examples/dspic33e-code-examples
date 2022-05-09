/*******************************************************************************
  Header file to set the Doze Ratio.

  Company:
    Microchip Technology Inc.

  File Name:
    set_doze_ratio.h

  Summary:
    Sets a proper Doze ratio.

  Description:
    This file contains function that sets the proper doze ratio
    and puts the device in the Doze mode.
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
#ifndef __SetDozeRatio_H__
    #define __SetDozeRatio_H__

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
    #include <stdint.h>

    #ifdef __cplusplus      // Provide C++ Compatability
extern "C"
{
        #endif

    // *****************************************************************************
    // *****************************************************************************
    // Section: Helper Macros
    // *****************************************************************************
    // *****************************************************************************
        #define DOZE1       0
        #define DOZE2       1
        #define DOZE4       2
        #define DOZE8       3
        #define DOZE16      4
        #define DOZE32      5
        #define DOZE64      6
        #define DOZE128     7

        #define DOZE1_ROI   8
        #define DOZE2_ROI   9
        #define DOZE4_ROI   10
        #define DOZE8_ROI   11
        #define DOZE16_ROI  12
        #define DOZE32_ROI  13
        #define DOZE64_ROI  14
        #define DOZE128_ROI 15

    // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Routines
    // *****************************************************************************
    // *****************************************************************************
    extern void SetDozeRatio( uint16_t r );

        #ifdef __cplusplus  // Provide C++ Compatibility
}

    #endif
#endif

/*******************************************************************************
 End of File
*/

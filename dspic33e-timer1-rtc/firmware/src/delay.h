/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    delay.h

  Summary:
    Defines various delay times.

  Description:
    This header file declares two delay functions that give can cause a delay in terms of
    milli seconds or micro seconds. Different counts have also been declared in this file.
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
#ifndef __DELAY_H__
    #define __DELAY_H__

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
    // Section: Constants
    // *****************************************************************************
    // *****************************************************************************
        #define Delay200uS_count    200
        #define Delay_1mS_Cnt       1
        #define Delay_2mS_Cnt       2
        #define Delay_5mS_Cnt       5
        #define Delay_15mS_Cnt      15
        #define Delay_1S_Cnt        1000

    // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Routines
    // *****************************************************************************
    // *****************************************************************************
    void    Delay( uint16_t delay_count );
    void    Delay_Us( uint16_t delayUs_count );

        #ifdef __cplusplus  // Provide C++ Compatibility
}

    #endif
#endif

/*******************************************************************************
 End of File
*/

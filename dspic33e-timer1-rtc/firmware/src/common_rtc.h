/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    common.h

  Summary:
    General variable linkage information.

  Description:
    This file contains definitions commonly used in this project like some general
    variables and a function declaration.
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
#include <stdint.h>
#ifdef __cplusplus      // Provide C++ Compatability
extern "C"
{
    #endif

    /* variables used in Timer 1 ISR */
    extern volatile uint8_t hours;
    extern volatile uint8_t minutes;
    extern volatile uint8_t seconds;
    extern volatile uint8_t rtc_Lcd_Update;

    /* variables used in hex to decimal routine */
    extern volatile uint8_t hunds;
    extern volatile uint8_t tens;
    extern volatile uint8_t ones;

    // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Routines
    // *****************************************************************************
    // *****************************************************************************
    extern void             HexDec( uint8_t count );
    extern void             Init_Timer1( void );
    extern void             Init_INTpin( void );

    #ifdef __cplusplus  // Provide C++ Compatibility
}

#endif

/*******************************************************************************
 End of File
*/

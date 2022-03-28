/*******************************************************************************
  ce452 timer header file

  Company:
    Microchip Technology Inc.

  File Name:
    timer.h

  Summary:
    timer API function definitions.

  Description:
    This file consists of the definitions for timer APIs
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
#ifdef __cplusplus      // Provide C++ Compatability
extern "C"
{
    #endif

    /*********************************************************************
 * Function:        TimerInit
 *
 * PreCondition:    None.
 *
 * Input:        None.
 *                  
 * Output:       None.
 *
 * Overview:        Initializes Timer0 for use.
 *
 ********************************************************************/
    extern void             TimerInit( void );

    /*********************************************************************
 * Function:        TimerIsOverflowEvent
 *
 * PreCondition:    None.
 *
 * Input:        None. 
 *                  
 * Output:       Status.
 *
 * Overview:        Checks for an overflow event, returns TRUE if 
 *     an overflow occured.
 *
 * Note:            This function should be checked at least twice
 *     per overflow period.
 ********************************************************************/
    extern unsigned char    TimerIsOverflowEvent( void );

    #ifdef __cplusplus  // Provide C++ Compatibility
}

#endif

/*********************************************************************
 * EOF
 ********************************************************************/

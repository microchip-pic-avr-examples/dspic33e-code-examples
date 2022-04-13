/*******************************************************************************
  ce452 timer function
  
  Company:
    Microchip Technology Inc.

  File Name:
    timer.c

  Summary:
    Function to initialize timer

  Description:
    Timer0 is initialized and checks for an overflow event.
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
#include <system.h>
#include "stdint.h"

/*********************************************************************
 * Function:       void TimerInit(void)
 *
 * PreCondition:   None.
 *
 * Input:          None.
 *                  
 * Output:         None.
 *
 * Overview:       Initializes Timer0 for use.
 *
 ********************************************************************/
void TimerInit( void )
{
    PR1 = 0x2FF;

    IPC0bits.T1IP = 5;
    T1CON = 0b1000000000010000;
    IFS0bits.T1IF = 0;
}

/*********************************************************************
 * Function:       unsigned char TimerIsOverflowEvent(void)
 *
 * PreCondition:   None.
 *
 * Input:          None.
 *                  
 * Output:         Status.
 *
 * Overview:       Checks for an overflow event, returns TRUE if 
 *                 an overflow occured.
 *
 * Note:           This function should be checked at least twice
 *                 per overflow period.
 ********************************************************************/
unsigned char TimerIsOverflowEvent( void )
{
    if( _T1IF )
    {
        _T1IF = 0;
        return ( 1 );
    }

    return ( 0 );
}

/*********************************************************************
 * EOF
 ********************************************************************/

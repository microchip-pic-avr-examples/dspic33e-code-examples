/*******************************************************************************
  ce479 timer function
  
  Company:
    Microchip Technology Inc.

  File Name:
    init_timer1.c

  Summary:
    This file is used to initialize timer1.

  Description:
    Timer 1 is initailzed to generate interrrupt after a certain period.
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
#include <stdint.h>

/******************************************************************************
 * Function:        void Init_Timer1( void )
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Initialize Timer1 for Period Interrupts
 *****************************************************************************/
void Init_Timer1( void )
{
    T1CON = 0;          // Timer reset
    IFS0bits.T1IF = 0;  // Reset Timer1 interrupt flag
    IPC0bits.T1IP = 6;  // Timer1 Interrupt priority level=4
    IEC0bits.T1IE = 1;  // Enable Timer1 interrupt
    TMR1 = 0x0000;
    PR1 = 0xFFFF;       // Timer1 period register = ?????
    T1CONbits.TON = 1;  // Enable Timer1 and start the counter
}

/*******************************************************************************
 End of File
 */

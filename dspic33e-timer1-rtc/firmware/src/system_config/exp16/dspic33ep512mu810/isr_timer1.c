/*******************************************************************************
  Timer1 Interrupt Handler Source file

  Company:
    Microchip Technology Inc.

  File Name:
    isr_timer1.c

  Summary:
    Interrupt Handler for Timer1.

  Description:
    This source file handles the Timer1 Interrupt service routine where the seconds,
    minutes and the hours are updated accordingly and a port pin is toggled at a rate 
    of 1 second.
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

// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************
volatile uint8_t    hours;
volatile uint8_t    minutes;
volatile uint8_t    seconds;
volatile uint8_t    rtc_Lcd_Update;

/******************************************************************************
 * Function:        _T1Interrupt
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Timer1 Interrupt Handler.
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _T1Interrupt( void )
{
    if( seconds < 59 )          // is cummulative seconds < 59?
    {
        seconds++;              // yes, so increment seconds
    }
    else                        // else seconds => 59
    {
        seconds = 0x00;         // reset seconds
        if( minutes < 59 )      // is cummulative minutes < 59?
        {
            minutes++;          // yes, so updates minutes
        }
        else                    // else minutes => 59
        {
            minutes = 0x00;     // reset minutes
            if( hours < 23 )    // is cummulative hours < 23
            {
                hours++;        // yes, so update hours
            }
            else
            {
                hours = 0x00;   // reset time
            }
        }
    }

    /* set flag to update LCD */
    rtc_Lcd_Update = 1;

    /* Toggle LED0 at 1 Hz rate */
    LATD = ( PORTD ^ 0x1 );

    /* reset Timer 1 interrupt flag */
    IFS0bits.T1IF = 0;
}

/*******************************************************************************
 End of File

 */

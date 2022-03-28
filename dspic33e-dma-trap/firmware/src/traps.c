/*******************************************************************************
  Source file for trap routines

  Company:
    Microchip Technology Inc.

  File Name:
    traps.c

  Summary:
    This file has different kinds of trap routines.

  Description:
     1. This file contains trap service routines (handlers) for hardware
    exceptions generated by the dsPIC33E device.
     2. All trap service routines in this file simply ensure that device
    continuously executes code within the trap service routine. Users
    may modify the basic framework provided here to suit to the needs
    of their application.
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
#ifdef TEST_MODE
#include "common.h"
#endif
/* Prototype function declaration for functions in the file */
void __attribute__ ( (__interrupt__) )  _OscillatorFail( void );
void __attribute__ ( (__interrupt__) )  _AddressError( void );
void __attribute__ ( (__interrupt__) )  _StackError( void );
void __attribute__ ( (__interrupt__) )  _MathError( void );
void __attribute__ ( (__interrupt__) )  _DMACError( void );


/*
Primary Exception Vector handlers:
These routines are used if INTCON2bits.ALTIVT = 0.
All trap service routines in this file simply ensure that device
continuously executes code within the trap service routine. Users
may modify the basic framework provided here to suit to the needs
of their application.
*/
/******************************************************************************
 * Function:        void __attribute__ ((interrupt, no_auto_psv)) _OscillatorFail(void)
 *
 * PreCondition:    None.
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Clears the trap flag if OSCFAIL bit is set.
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _OscillatorFail( void )
{
    INTCON1bits.OSCFAIL = 0;    //Clear the trap flag
    while( 1 );
}

/******************************************************************************
 * Function:        void __attribute__ ((interrupt, no_auto_psv)) _AddressError(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Clears the trap flag if AddressErrr bit is set.
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _AddressError( void )
{
    INTCON1bits.ADDRERR = 0;    //Clear the trap flag
    while( 1 );
}

/******************************************************************************
 * Function:        void __attribute__ ((interrupt, no_auto_psv)) _StackError(void)
 *
 * PreCondition:    None.
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Clears the trap flag if _StackError bit is set.
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _StackError( void )
{
    INTCON1bits.STKERR = 0;     //Clear the trap flag
    while( 1 );
}

/******************************************************************************
 * Function:        void __attribute__ ((interrupt, no_auto_psv)) _MathError(void)
 *
 * PreCondition:    None.
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Clears the trap flag if _MathError bit is set.
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _MathError( void )
{
    INTCON1bits.MATHERR = 0;    //Clear the trap flag
    while( 1 );
}

/******************************************************************************
 * Function:        void __attribute__ ((interrupt, no_auto_psv)) _DMACError(void)
 *
 * PreCondition:    None.
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Clears the trap flag if _DMACError bit is set.
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _DMACError( void )
{
    INTCON1bits.DMACERR = 0;    //Clear the trap flag
#ifdef TEST_MODE
    test_flag=1;
#else
    while( 1 );
#endif
}

/*******************************************************************************
 End of File
*/

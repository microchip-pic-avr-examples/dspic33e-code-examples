/*******************************************************************************
   PWM Configuration Code Source file

  Company:
    Microchip Technology Inc.

  File Name:
    pwm.c

  Summary:
    This file contains PWM Initialisation routine.

  Description:
    This code example generates multiple ADC triggers in synchronisation with
    PWM time base.
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

// *****************************************************************************
// *****************************************************************************
// Section: Helper Macros
// *****************************************************************************
// *****************************************************************************
#define FCY_HZ          40000000            // Instruction cycle frequency (Hz)
#define TCY_SEC         ( 1.0 / FCY_HZ )    // Instruction cycle period (sec)
#define FSW_HZ          20000               // PWM Switchng Frequency (Hz)
#define LOOPTIME_SEC    ( 1.0 / FSW_HZ )    // PWM Period
#define DEADTIME_SEC    0.000002            // Dead time in seconds - 2us
#define LOOPTIME_TCY    ( unsigned int ) ( LOOPTIME_SEC / TCY_SEC ) // Basic loop period in units of Tcy
#define DDEADTIME       ( unsigned int ) ( DEADTIME_SEC * FCY_HZ )  // Dead time in dTcys

// Functions
void    Init_PWMSync( void );

/******************************************************************************
 * Function:        void Init_PWMSync(void)
 *
 * PreCondition:    None.
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Function initialises PWM1 in edge aligned complementary mode
 *****************************************************************************/
void Init_PWMSync( void )
{
    _RP55R = 0x2D;          //SYNCO mapped to RP55

    // PWM Primary Master Clock Divider Select Register
    PTCON2bits.PCLKDIV = 0;

    // Configure PTPER register for PWM frequency
    PTPER = 2 * LOOPTIME_TCY;

    PWMCON1 = 0x0000;       // Enable PWM output pins and configure them

    // Load ALTDTR1 register with preset dead time value
    ALTDTR1 = DDEADTIME;    // Deadtime setting

    // Load DTR1 register with preset dead time value
    DTR1 = DDEADTIME;       // Deadtime setting

    // Load PDC1 with initial Duty Cycle value
    PDC1 = LOOPTIME_TCY;

    // PWM1 Fault and Current Limit Control register
    FCLCON1bits.FLTMOD = 3;

    // PWM1 I/O Control register
    IOCON1bits.PENH = 1;    // PWM1H is controlled by PWM module
    IOCON1bits.PENL = 1;    // PWM1L is controlled by PWM module
    PTCONbits.SYNCPOL = 0;
}

/*******************************************************************************
 End of File
 */

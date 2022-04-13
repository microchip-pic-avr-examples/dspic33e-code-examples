/*******************************************************************************
    PTG Configuration Code Source file

  Company:
    Microchip Technology Inc.

  File Name:
    ptg.c

  Summary:
    This code example generates phase shifted waveforms using OC and PTG modules.
    
  Description:
    This file contains PTG Initialisation routine and PTG Step Command 
    configuration routine to generate phase shifted waveforms.
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
#include "ptg_definition.h"

// Functions
void    Write_PTG_Sequence( void );
void    Init_PTG( void );

/******************************************************************************
 * Function:        void Init_PTG(void)
 *
 * PreCondition:    None.
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is used to configure the PTG module to select the Peripheral clock as the
                    clock source and generate an output on every PTGTRIG command.

 *****************************************************************************/
void Init_PTG( void )
{
    PTGT0LIM = 1400;        // Specify required phase shift ( 20us * 70 = 1400)

    // This is use to create the required phase shift between the waveforms
    PTGCSTbits.PTGEN = 0;   // PTG module is disabled
    PTGCSTbits.PTGIVIS = 0; // Reading PTGSDLIM, PTGCxLIM or PTGTxLIM registers return the value of these limit registers
    PTGCSTbits.PTGTOGL = 0; // Each execution of the PTGTRIG command will generate a single PTGOx pulse
    PTGCSTbits.PTGITM = 0;  // Continuous edge detect with Step delay executed on exit of command
    PTGCONbits.PTGCLK = 0;  // PTG Clock source is Peripheral clock
    PTGCONbits.PTGDIV = 0;  // PTG Clock is divided by 1
    PTGQPTR = 0;            // Assigning Queue pointer to start of the queue
}

/******************************************************************************
 * Function:        void Write_PTG_Sequence(void)
 *
 * PreCondition:    None.
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Defines PTG Sequencer steps by writing to step queue registers. 

 *****************************************************************************/
void Write_PTG_Sequence( void )
{
    //Step Queue
    _STEP0 = PTGWHI | PTGWOPT7;     // Wait for OC1 input trigger event
    _STEP1 = PTGCTRL | CTRLOPT8;    // Starts PTGTO timer and waits for time out (1400/70 = 20us)
    _STEP2 = PTGTRIG | TRIGOPT1;    // Trigger PTG02 (trig/sync for OC2
#ifdef TEST_MODE
    _STEP3 = PTGIRQ | 0x00;
    _STEP4 = PTGJMP | 0x0;          // Repeat Loop from Step 0
#else
    _STEP3 = PTGJMP | 0x0;          // Repeat Loop from Step 0
#endif
}

/*******************************************************************************
 End of File
 */

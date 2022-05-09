/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    en_sec_osc.s

  Summary:
    Enables the secondary oscillator.

  Description:
    This source file defines a routine that enables the secondary oscillator to be
    used by the Timer module as its clock which is used in a real time clock application.
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
; *****************************************************************************
; *****************************************************************************
; Section: Included Files
; *****************************************************************************
; *****************************************************************************

.include "xc.inc"

; *****************************************************************************
; *****************************************************************************
; Section: File Scope or Global Constants
; *****************************************************************************
; *****************************************************************************

    .global     _EnSecOsc

    .section .text

/******************************************************************************
  Enable Secondary Osc
*******************************************************************************/
_EnSecOsc:

        ;OSCCONL(Low byte) Unlock Sequence
        mov     #OSCCONL, w1
        mov.b     #0x02, w0
        mov     #0x46, w2
        mov     #0x57, w3
        mov.b     w2, [w1]
        mov.b     w3, [w1]

        ; Enable Sec Osc
        mov.b     w0, [w1]
    

           return

.end

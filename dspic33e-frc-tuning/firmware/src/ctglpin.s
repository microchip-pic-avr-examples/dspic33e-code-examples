/*******************************************************************************
  ce410 port pin toggle file

  Company:
    Microchip Technology Inc.

  File Name:
    ctglpin.s

  Summary:
    Toggles RF6 port pin.

  Description:
    This file consists of the port initialization and pin toggling functions.
    The RF6 pin is first initialized to function as an output pin
    and then a small function is written to toggle the RF6 pin between 1 and 0.
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

    .global _CtglPin
    .global _CtglPinInit

; Init RF6 for toggling
_CtglPinInit:
    bclr ODCF,#6
    bclr TRISF,#6
    return

; Toggle RF6
; Toggle Freq is 1/4 of system clock frequency
_CtglPin:
loop:
    btg    LATF,#6
    btg    LATF,#6
    bra loop

    return
.end


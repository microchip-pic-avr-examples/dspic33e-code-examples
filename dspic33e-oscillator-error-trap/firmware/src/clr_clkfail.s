/*******************************************************************************
  Oscillator Failure source file

  Company:
    Microchip Technology Inc.

  File Name:
    clr_clkfail.s

  Summary:
    Clears the Oscillator failure status flag and Clock fail bit.

  Description:
    This source file clears the clock fail bit in OSCCON register and then
    clears the Oscillator failure status bit if an Oscillator failure has 
    taken place. An unlock code is written first and only then can the clock 
    fail bit be modified (cleared in this case).
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

    .global _ClrClkFail
 
;
_ClrClkFail:
        mov     OSCCON, w0              ;Read OSCCON in to w0
        bclr    w0, #CF                 ;Before clearing the OSCFAIL trap status flag
                                        ;we should clear the Clock Fail(CF) bit in
                                        ;OSCCONL

                                        ;Load w0 with a byte value to write to
                                        ;OSCCONL so that the CF bit is cleared
        mov     #OSCCONL, w1            ;Point w1 to OSCCONL byte
        disi    #6                      ;Disable interrupts for the next
                                        ;5 instruction cycles

        mov     #0x46, w2               ;Move first OSCCONL unlock code byte to w2
        mov     #0x57, w3               ;Move second OSCCONL unlock code byte to w3
        mov.b   w2, [w1]                ;Perform byte-wide move of w2 to OSCCONL
        mov.b   w3, [w1]                ;Perform byte-wide move of w3 to OSCCONL
        mov.b   w0, [w1]                ;Write parameter in w0 (lower byte)to
                                        ;OSCCONL within one instruction cycle
                                        ;of unlocking OSCCONL using MOV instruction
        return

.end
/* End of File */

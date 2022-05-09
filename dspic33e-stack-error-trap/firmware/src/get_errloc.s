/*******************************************************************************
  Error Location Source file

  Company:
    Microchip Technology Inc.

  File Name:
    get_errloc.s

  Summary:
    Gives the error location of the trap.

  Description:
    This source file consists of a routine that provides the exact location
    where the Address Error Trap occured. 
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

    .global     GetErrLoc

    .section .text

; Stack Growth from Trap Error
;1. PC[15:0]            <--- Trap Address
;2. SR[7:0]:IPL3:PC[22:16]
;3. RCOUNT
;4. W0
;5. W1
;6. W2
;7. W3
;8. W4
;9. W5
;10. W6
;11. W7
;12. OLD FRAME POINTER [W14]
;13. PC[15:0]           <---- W14 
;14. 0:PC[22:16]
;15.                    <---- W15

_GetErrLoc:
        mov    w14,w2
        sub    w2,#24,w2
        mov    [w2++],w0
        mov    [w2++],w1 
        mov    #0x7f,w3     ; Mask off non-address bits
        and    w1,w3,w1

        mov    #2,w2        ; Decrement the address by 2
        sub    w0,w2,w0
        clr    w2
        subb   w1,w2,w1
        return

.end

/* End of file */


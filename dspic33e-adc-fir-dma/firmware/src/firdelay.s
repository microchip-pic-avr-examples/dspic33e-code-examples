/*******************************************************************************
  FIR API

  Company:
    Microchip Technology Inc.

  File Name:
    firdelay.s

  Summary:
    This file has FIR delay routine

  Description:
    This file consists of FIR function which initializes the delay values in the FIR filter structure to zeros.
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

/*****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************/


 ; Local inclusions.
 .nolist
 .include "dspcommon.inc"  ; FIR filter structure
 .list

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

# .section .libdsp, code

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; _FIRDelayInit: initialization to zero of FIR filter delay.
;
; Operation:
; firfilter->delayBase[m] = 0, 0 <= m < firfilter->numCoeffs (= M)
;
; Input:
; w0 = h, ptr FIR filter structure (see included file)
; Return:
; (void)
;
; System resources usage:
; {w0..w2} used, not restored
;
; DO and REPEAT instruction usage.
; no DO instructions
; 1 level REPEAT intruction
;
; Program words (24-bit instructions):
; 7
;
; Cycles (including C-function call and return overheads):
; 11 + M
;............................................................................

 .global _FIRDelayInit ; export
_FIRDelayInit:

;............................................................................

 ; Prepare operation.
 mov [w0+oNumCoeffs],w1  ; w1 = M
 dec w1,w2    ; w2 = M-1
 mov [w0+oDelayBase],w1  ; w1-> delayBase[0]
 mov #0,w0    ; w0 = 0

;............................................................................

 ; Perform operation.
 repeat w2    ; do (M-1)+1 times
 mov w0,[w1++]   ; delayBase[m] = 0
      ; w1-> delayBase[m+1]

;............................................................................

 return 

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

 .end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; OEF

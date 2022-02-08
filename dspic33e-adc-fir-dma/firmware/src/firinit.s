/*******************************************************************************
  FIR API

  Company:
    Microchip Technology Inc.

  File Name:
    firinit.s

  Summary:
    This file has FIR initialization routine

  Description:
    This file consists of FIR function which initializes the FIR filter structure parameters.
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

 #.section .libdsp, code

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;
; _FIRStructInit: initialization of FIR filter structure.
;
; Operation:
; firfilter->numCoeffs = numCoeffs;
; firfilter->coeffsBase = coeffsBase;
; firfilter->coeffsEnd = coeffsBase+numCoeffs-(1 byte);
; firfilter->coeffsPage = coeffsPage;
; firfilter->delayBase = delayBase;
; firfilter->delayEnd = delayBase+numCoeffs-(1 byte);
; firfilter->delay = delayBase;
;
; Input:
; w0 = h, ptr FIR filter structure (see included file)
; w1 = numCoeffs;
; w2 = coeffsBase;
; w3 = coeffsPage;
; w4 = delayBase;
; Return:
; (void)
;
; System resources usage:
; {w0..w5} used, not restored
;
; DO and REPEAT instruction usage.
; no DO intructions
; no REPEAT intruction
;
; Program words (24-bit instructions):
; 10
;
; Cycles (including C-function call and return overheads):
; 19
;............................................................................

 .global _FIRStructInit ; export
_FIRStructInit:

;............................................................................

 ; Prepare for initialization.
 sl w1,w5    ; w5 = numCoeffs*sizeof(coeffs)
 dec w5,w5    ; w5 =
      ;   numCoeffs*sizeof(coeffs)-1

;............................................................................

 ; Set up filter structure.
 mov w1,[w0++]  ; firfilter->numCoeffs = numCoeffs
     ; w0 =&(firfilter->coeffsBase)
 mov w2,[w0++]  ; firfilter->coeffsBase = coeffsBase
     ; w0 =&(firfilter->coeffsEnd)
 add w2,w5,[w0++]  ; firfilter->coeffsEnd initialized
     ; w0 =&(firfilter->coeffsPage)
 mov w3,[w0++]  ; firfilter->coeffsPage = coeffsPage
     ; w0 =&(firfilter->delayBase)
 mov w4,[w0++]  ; firfilter->delayBase = delayBase
     ; w0 =&(firfilter->delayEnd)
 add w4,w5,[w0++]  ; firfilter->delayEnd initialized
     ; w0 =&(firfilter->delay)
 mov w4,[w0]   ; firfilter->delay initialized

;............................................................................

 return 

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

 .end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; OEF

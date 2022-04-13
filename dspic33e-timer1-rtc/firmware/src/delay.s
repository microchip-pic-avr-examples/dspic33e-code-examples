/*******************************************************************************
  Delay Routine Source file

  Company:
    Microchip Technology Inc.

  File Name:
    delay.s

  Summary:
    Generates different delays.

  Description:
    This source file has different routines to generate delays of different magnitudes 
    ranging from milli seconds to microseconds. The main function calls the appropriate
    delay routine to implement a particular delay.
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

.set Fcy,        40000000

.set US_K,       Fcy/1000000
.set MS_K,       Fcy/10000

    .global _Delay
    .global _Delay_Us

;===============================================
; ms Delay Function
;===============================================
_Delay:

ms_oloop:
    mov #MS_K,w1
ms_iloop:
    nop
    nop
    nop
    nop
    nop

    nop
    nop


    dec     w1, w1
    bra     nz, ms_iloop    
    
    dec     w0,w0
    bra     nz,ms_oloop

    return

;===============================================
; us Delay Function
;===============================================

_Delay_Us:

us_oloop:
    
    .rept (US_K-3)
    nop
    .endr
 
    
    dec     w0,w0
    bra     nz,us_oloop

    return

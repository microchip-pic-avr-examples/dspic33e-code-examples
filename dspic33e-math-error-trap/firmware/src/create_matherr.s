/*******************************************************************************
  Math Error Generator Source file

  Company:
    Microchip Technology Inc.

  File Name:
    create_matherr.s

  Summary:
    Creates different kinds of Math errors.

  Description:
    This source file consists of different routines that generate a Math error such
    as divide by 0, accumulator overflow and accumulator shif error which are all sources
    for generating a Math error trap.
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

    .global     _CreateAccShtError
    .global     _CreateDiv0Error
    .global     _CreateAccaOflow
    .global     _CreateAccbOflow
    .global     _CreateAccaCoflow
    .global     _CreateAccbCoflow


    .section .text
/******************************************************************************
;  ACC Shift Error
*******************************************************************************/

_CreateAccShtError:
        mov     #0x7FFF,w0  
        lac     w0,#0,a
        mov     #17,w1              ; Invalid Shift Count
        sftac   A,w1

        nop
        nop
        nop
        nop
        nop

        return
        


/******************************************************************************
;  DIVIDE BY 0 ERROR
*******************************************************************************/
_CreateDiv0Error:
        clr     SR
        clr     w2                  ; Divisor=0
        mov     #1,w0               ; Dividend=1

        repeat  #17
        div.s   w0,w2               ; Perform 1/0
 
  
        nop
        nop
        nop
        nop
        nop

        return


    .section .text
/******************************************************************************
;  ACCA Overflow
*******************************************************************************/
_CreateAccaOflow:
        clr     SR
        bclr    CORCON,#ACCSAT      ; 1:31 Saturation Mode

        bclr    CORCON,#SATA        ; Comment this line for Saturation
;       bset    CORCON,#SATA        ; Comment this line for Overflow
        mov     #0x7FFF,w0          ; Max POS value
        mov     #0x1,w1             ; + 1
        lac     w0,#0,a
        lac     w1,#0,b
        add     a                   ; a=acca+accb   
  
        nop
        nop
        nop
        nop
        nop

        return
     
/******************************************************************************
;  ACCB Overflow
*******************************************************************************/
_CreateAccbOflow:
        clr     SR
        bclr    CORCON,#ACCSAT      ; 1.31 Saturation Mode

        bclr    CORCON,#SATB        ; Comment this line for Saturation
;       bset    CORCON,#SATB        ; Comment this line for Overflow
        mov     #0x8000,w0          ; Max NEG Value
        mov     #0xFFFF,w1          ; -1
        lac     w0,#0,a
        lac     w1,#0,b
        add     b                   ; b=acca+accb    

        nop
        nop
        nop
        nop
        nop
 
        return



/******************************************************************************
;  ACCA Catastrophic Overflow
*******************************************************************************/
_CreateAccaCoflow:
        clr     SR
        bset    CORCON,#ACCSAT      ; 9.31 Saturation Mode
        bclr    CORCON,#SATA        


        mov     #0x7FFF,w0  
        mov     #0x1,w1  
        lac     w0,#0,a
        lac     w1,#0,b

        sftac   a,#-8       ; Close to Max POS value
        sftac   b,#-8       ; Value to create overflow

        add     a           ; a=acca+accb   

        nop
        nop
        nop
        nop
        nop

        return
     
/******************************************************************************
;  ACCB Catastrophic Overflow
*******************************************************************************/
_CreateAccbCoflow:
        clr     SR
        bset    CORCON,#ACCSAT  ; 9.31 Saturation Mode
        bclr    CORCON,#SATB        


        mov     #0x8000,w0          
        mov     #0xFFFF,w1  
        lac     w0,#0,a
        lac     w1,#0,b

        sftac   a,#-8           ; Max NEG value
        sftac   b,#-8           ; Value to create Overflow

        add     b           ; b=acca+accb   

        nop
        nop
        nop
        nop
        nop

        return

     

.end

/* End Of File */
/*******************************************************************************
  Stack Error Generator Source file

  Company:
    Microchip Technology Inc.

  File Name:
    create_stkerr.s

  Summary:
    Creates different kinds of Stack errors.

  Description:
    This source file consists of two routines that generate a Stack error trap namely
    Stack Overflow and Stack Underflow. The stack pointer is made to either go beyond the 
    SPLIM value to create an Overflow or made to go below the SPLIM value to create
    a Stack Unerflow thus resulting in a trap.
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

    .global     _CreateStkUflow
    .global     _CreateStkOflow

    .section .text
/******************************************************************************
  Stack Overflow
*******************************************************************************/
_CreateStkOflow:
; Save Stack and SPLIM register
        mov w15,w1
        mov SPLIM,w2

; Create Stack Overflow trap condition
        mov w15,w0
        add w0,#20,w0
        mov w0,SPLIM
            
        mov #12,w0

oflow_loop:
        push    w0
           dec     w0, w0
        bra     nz, oflow_loop 

; Restore Stack pointers and SPLIM values
        mov     w1,w15
        mov     w2,SPLIM

        return
     
/******************************************************************************
  Stack Underflow
*******************************************************************************/
_CreateStkUflow:
; Save Stack and SPLIM register
        mov w15,w1
        mov SPLIM,w2

; Create Stack Underflow trap condition
        mov #0xFFF,w0               ; Lowest Stack Address
        add w0,#20,w0
        mov w0,w15
            
        mov #11,w0

uflow_loop:
        pop     w3
           dec     w0, w0
        bra     nz, uflow_loop 

; Restore Stack pointers and SPLIM values
        mov     w1,w15
        mov     w2,SPLIM

        return
.end

/* End of File */
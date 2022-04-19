/*******************************************************************************
  DMA Trap creation source file

  Company:
    Microchip Technology Inc.

  File Name:
    create_dmatrap.c

  Summary:
    Creates a DMA trap condition.

  Description:
    This source file has two different routines that create two different kinds of DMA trap conditions.
    One of the routines creates a DMA write collision event and the other creates a peripheral write
    collision event. 
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

.global _CreatePerWriteCol
.global _CreateDmaWriteCol
.global _uart1RxBuffA


    .section .text

/******************************************************************************
; Peripheral Write Collision
*******************************************************************************/

_CreatePerWriteCol:
        mov #0xDEAD,w1

perWrColl:
        mov w1,U1TXREG
        bra perWrColl
        return
        
/******************************************************************************
;  DMA Write Collision
*******************************************************************************/
        
_CreateDmaWriteCol:

        mov #0xDEAD,w0
        mov #_uart1RxBuffA,w2

; Write to DMA RAM receive buffer
dmaWrColl:
        mov w2,w1
        mov w0,[w1++]
        mov w0,[w1++]
        mov w0,[w1++]
        mov w0,[w1++]

        mov w0,[w1++]
        mov w0,[w1++]
        mov w0,[w1++]
        mov w0,[w1++]

        bra dmaWrColl
        return
.end

/* End of File */
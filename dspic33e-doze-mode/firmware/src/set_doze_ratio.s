/*******************************************************************************
  Source file to set the Doze Ratio.

  Company:
    Microchip Technology Inc.

  File Name:
    set_doze_ratio.s

  Summary:
    Sets the proper Doze Ratio

  Description:
    This file consists of the routine that enables setting a particular
    doze ratio and enables the doze mode to operate the device at a lesser
    speed than set by the configuration fuses.
*******************************************************************************/
/*******************************************************************************
Copyright (c) <2012> released Microchip Technology Inc.  All rights reserved.

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

    .global     _SetDozeRatio

    .section .text

/******************************************************************************
  SET DOZE RATIO 
*******************************************************************************/
_SetDozeRatio:
        sl         w0,#12,w0 
        disi     #10                ; Block all interrupt with priority <7 for next 5 instructions    
        mov        CLKDIV,w1
        mov        #0x7FF,w2
        and     w1,w2,w1
        ior        w0,w1,w1
        mov     w0, CLKDIV        ; Write Doze ratio and ROI bit

        bset    w0,#DOZEN    
        mov     w0, CLKDIV        ; Enable Doze

           return

.end

/* End of File  */
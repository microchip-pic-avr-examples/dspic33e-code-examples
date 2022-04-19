/*******************************************************************************
  ISR in AUX flash.

  Company:
    Microchip Technology Inc.

  File Name:
    aux_int.s

  Summary:
    This file has timer1 ISR in AUX flash.

  Description:
    This file defines Interrupt service routin in auxiliary flash. 
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
 *****************************************************************************
 Section: Included Files
 *****************************************************************************
 *****************************************************************************/

.include "xc.inc"

/****************************************************************************
 *****************************************************************************
 Section: File Scope or Global Constants
 *****************************************************************************
 *****************************************************************************/


.equ AUX_IVEC, 0x7FFFFA ; auxilliary interrupt vector for RSTPRI = 0
.equ AUX_IVEC_SERVICE, 0x7FFFC4 ; auxilliary interrupt service address for fake vector
.equ AUX_SPACE ,0x7FCA00 ;

.section *, address(AUX_SPACE), code   ; 
.global _Fun_in_aux
_Fun_in_aux:bra _Fun_in_aux ; wait here

.section *, address(AUX_IVEC_SERVICE), code ;ISR in aux flash
.global Aux_interrupt
Aux_interrupt:
PUSH w0 ; save w0 
PUSH _TBLPAG ; save TBLPAG 
push.d W8
CLR w0 
MOV w0, _TBLPAG ; vectors are in page 0
sl.b INTTREG, WREG ; *2 for word offset, ignore high byte 
ADD #4, w0 ; add 4 to get over reset instruction
tblrdl [w0], w0 ; get the vector
PUSH SR 
CALL w0 ; execute the ISR
POP SR 
pop.d W8    
POP _TBLPAG ; restore TBLPAG 
POP w0 ; restore w0 
RETFIE ; 

.section *, address(AUX_IVEC), code   ; 
.pword AUX_IVEC_SERVICE    ;Store the address of ISR in the aux IVT


.end

/* End of File */

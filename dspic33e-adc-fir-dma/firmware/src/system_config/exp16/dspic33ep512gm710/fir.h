/*******************************************************************************
  
  Company:
    Microchip Technology Inc.

  File Name:
    fir.h

  Summary:
    FIR Coeffecient Buffer definition.

  Description:
    This file consists of the definitions of FIR Coeffecient Buffers.
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

#ifdef __cplusplus  // Provide C++ Compatability

    extern "C" {

#endif

// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************
#define NY 20
#define CHUNK 512

////////////////////////////////////////
// FIR Coeffecient Buffer
fractional coeffecients[NY] __attribute__ ((space(xmemory),far)) = {
0x0655,	0x065B,	0x065F,	0x0664,	0x0667,	0x066A,	0x066D,	0x066F,	0x0670,
0x0670,	0x0670,	0x0670,	0x066F,	0x066D,	0x066A,	0x0667,	0x0664,	0x065F,
0x065B,	0x0655                                                         
};

// FIR Delay Buffer
//fractional z[NY] __attribute__ ((space(ymemory)));
fractional z[NY] __attribute__((space(ymemory), address(0x9000)));


#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif



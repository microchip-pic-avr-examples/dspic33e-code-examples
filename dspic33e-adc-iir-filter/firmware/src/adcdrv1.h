/*******************************************************************************
  ADC Driver header file

  Company:
    Microchip Technology Inc.

  File Name:
    adcdrv1.h

  Summary:
    Declarations for ADC functions.

  Description:
    This header file consists of the declaration of the 
    ADC and DMA initialization functions and also the DMA ISR. 
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
#ifndef __ADCDRV1_H__
#define __ADCDRV1_H__ 

#ifdef __cplusplus  // Provide C++ Compatability

    extern "C" {

#endif

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************

#define Fosc        120000000
#define Fcy            (Fosc/2)
#define Fs           8000
#define SAMPPRD    (Fcy/Fs)-1
#define NUMSAMP     256

// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************

#ifdef TEST_MODE
extern unsigned char test_flag;
#endif

/* These functions are used to initialize the ADC1, Timer 3 and DMA0 modules
    and are called by the main function. */
extern void InitAdc1(void);
extern void InitTmr3(void);
extern void InitDma0(void);
extern void __attribute__((__interrupt__)) _DMA0Interrupt(void);


#ifdef __cplusplus  // Provide C++ Compatibility

    }

#endif

#endif

/*******************************************************************************
 End of File
*/

/*******************************************************************************
  ce478 test data header file

  Company:
    Microchip Technology Inc.

  File Name:
    testdata.h

  Summary:
    Test input header file.

  Description:
    This header file consists of the data that stored in flash.
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
#ifndef _TESTDATA_H
    #define _TESTDATA_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
    #include <stdint.h>

    #ifdef __cplusplus      // Provide C++ Compatability
extern "C"
{
        #endif

    // *****************************************************************************
    // *****************************************************************************
    // Section: Constants
    // *****************************************************************************
    // *****************************************************************************
    uint16_t    myRowData1InFlash[] __attribute__( (space(prog), address(0x1000)) ) =
    {
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111,
        0x1111
    };

    uint16_t    myRowData2InFlash[] __attribute__( (space(prog), address(0x1100)) ) =
    {
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222,
        0x2222
    };

    uint16_t    myRowData3InFlash[] __attribute__( (space(prog), address(0x1200)) ) =
    {
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333,
        0x3333
    };

    uint16_t    myRowData4InFlash[] __attribute( (space(prog), address(0x1300)) ) =
    {
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444,
        0x4444
    };

    uint16_t    myRowData5InFlash[] __attribute__( (space(prog), address(0x1400)) ) =
    {
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555,
        0x5555
    };

    uint16_t    myRowData6InFlash[] __attribute__( (space(prog), address(0x1500)) ) =
    {
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666,
        0x6666
    };

    uint16_t    myRowData7InFlash[] __attribute__( (space(prog), address(0x1600)) ) =
    {
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777,
        0x7777
    };

    uint16_t    myRowData8InFlash[] __attribute__( (space(prog), address(0x1700)) ) =
    {
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888,
        0x8888
    };

        #ifdef __cplusplus  // Provide C++ Compatibility
}

    #endif
#endif // _TESTDATA_H

/*******************************************************************************
 End of File
*/
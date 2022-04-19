/*******************************************************************************
 
  Company:
    Microchip Technology Inc.

  File Name:
    rtdmuser.h

  Summary:
    User defined configuration for RTDM functions.

  Description:
    This file consists of the definitions for RTDM configuration, which user can set as per his setting.
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
#ifndef RTDMUSER_H
    #define RTDMUSER_H

    #ifdef __cplusplus                      // Provide C++ Compatability
extern "C"
{
        #endif

    // *****************************************************************************
    // *****************************************************************************
    // Section: File Scope or Global Constants
    // *****************************************************************************
    // *****************************************************************************
        #define YES 1
        #define NO  0

    /************************************** RTDM DEFINITIONS  ***************************************/
        #define RTDM_FCY    55275000        //This define has to be the system operating freq, this

    //value is used to calculate the value of the BRG register
        #define RTDM_BAUDRATE   38400       //This is the desired baudrate for the UART module to be

    //used by RTDM
        #define RTDM_UART   2               // This is the UART module to be used by RTDM. It has only

    // two possible values: 1 or 2
    // For dsPIC33E and PIC24E, values 3 and 4 are also supported
        #define RTDM_UART_PRIORITY  3       //This the UART RX interrupt priority assigned to receive

    // the RTDM messages
        #define RTDM_RXBUFFERSIZE   32      // This is the buffer size used by RTDM to handle messaages
        #define RTDM_MAX_XMIT_LEN   0x1000  //This the size in bytes of the max num of bytes allowed in

    //the RTDM protocol Frame
        #define RTDM_POLLING    YES         // This defines the mode that RTDM will be operating in

    //user's application. If it is YES then the user should place the
    //RTDM_ProcessMsgs() function in the main loop.
    //In order to make sure that the messages are being preoccessed
    // it is recommended that the main loop always polls this
    //function as fast as possible
        #define RTDM_MIN_CODE_SIZE  YES     //When defined causes the RTDM library to build  without

    //including a pre-calculated polynomial  table for the CRC algorythim.
    //This saves 768  bytes of code space.
    /*************************************************************************************************/
        #ifdef __cplusplus                  // Provide C++ Compatibility
}

    #endif
#endif

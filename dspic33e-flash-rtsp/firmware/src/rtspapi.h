/*******************************************************************************
  ce409 RTSP API header file

  Company:
    Microchip Technology Inc.

  File Name:
    rtspapi.h

  Summary:
    Flash API function definitions.

  Description:
    This file consists of the definitions for the Flash erase, 
    Flash read, Flash page modify and the Flash program fucntions
    that are called in the main.c.
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
#ifndef __RTSPAPI_H__
    #define __RTSPAPI_H__

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
    #include <stdint.h>
    #include <xc.h>

    #ifdef __cplusplus      // Provide C++ Compatability
extern "C"
{
        #endif

    // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Routines
    // *****************************************************************************
    // *****************************************************************************
    /*  Flash Memory is organised into ROWs of 128 instructions or 256 bytes.
    RTSP allows the user to erage a PAGE of memory which consists of EIGHT ROWs
    (512 instructions or 1024byts) at a time.
    RTSP allows the user to program a ROW (128 instructions or 192 bytes) at a
    time
*/
    // *****************************************************************************
    /******************************************************************************
* Function:     int16_t FlashPageErase(uint16_t nvmAdru, uint16_t nvmAdr)
*
* PreCondition:  None
*
* Input:  nvmAdru - Selects the upper 8bits of the location to program or erase
*                   in program flash memory
*          nvmAdr - Selects the location to program or erase in program flash
*                   memory. It must be aligned to 512 instruction boundary,
*                   LSB 10bits of address must be zero
*
* Output:  returns ERROREE (or -1), if it is not successful,
*          returns ZERO, if successful
*
* Overview:      This function provides the interface to erase the flash.
*
*******************************************************************************/
    int16_t FlashPageErase( uint16_t nvmAdru, uint16_t nvmAdr );

    /******************************************************************************
* Function:int16_t FlashPageRead(uint16_t nvmAdru, uint16_t nvmAdr, int16_t *pageBufPtr);
*
* PreCondition:  None
*
* Input:  nvmAdru - Selects the upper 8bits of the location to program or erase
*                   in program flash memory
*          nvmAdr - Selects the location to program or erase in program flash
*                   memory. It must be aligned to 512 instruction boundary,
*                   LSB 10bits of address must be zero
*       pageBufPtr - Pointer to the data array in which read data will be stored
*
* Output:  returns ERROREE (or -1), if it is not successful,
*          returns ZERO, if successful
*
* Overview:      This function provides the interface to read the flash.
*
*******************************************************************************/
    int16_t FlashPageRead( uint16_t nvmAdru, uint16_t nvmAdr, int16_t *pageBufPtr );

    /******************************************************************************
* Function:  int16_t FlashPageModify(uint16_t row, uint16_t size,
*                          int16_t *rowBuf, int16_t *pageBufPtr)
*
* PreCondition:  None
*
* Input:  nvmAdru - Selects the upper 8bits of the location to program or erase
*                   in program flash memory
*          nvmAdr - Selects the location to program or erase in program flash
*                   memory. It must be aligned to 512 instruction boundary,
*                   LSB 10bits of address must be zero
*         rowBuf  - Selects the location to read in program flash memory
*       pageBufPtr - Pointer to the data array in which read data will be stored
*
* Output:  returns ERROREE (or -1), if it is not successful,
*          returns ZERO, if successful
*
* Overview:      This function provides the interface to read the flash.
*
*******************************************************************************/
    int16_t FlashPageModify( uint16_t row, uint16_t size, int16_t *rowBuf, int16_t *pageBufPtr );

    /******************************************************************************
* Function:int16_t FlashPageWrite(uint16_t nvmAdru, uint16_t nvmAdr, int16_t *pageBufPtr)
*
* PreCondition:  None
*
* Input:  nvmAdru - Selects the upper 8bits of the location to program or erase
*                   in program flash memory
*          nvmAdr - Selects the location to program or erase in program flash
*                   memory. It must be aligned to 512 instruction boundary,
*                   LSB 10bits of address must be zero
*       pageBufPtr - Pointer to the data array from which data will be written
*
* Output:  returns ERROREE (or -1), if it is not successful,
*          returns ZERO, if successful
*
* Overview:      This function provides the interface to write the flash.
*
*******************************************************************************/
    int16_t FlashPageWrite( uint16_t nvmAdru, uint16_t nvmAdr, int16_t *pageBufPtr );

        #ifdef __cplusplus  // Provide C++ Compatibility
}

    #endif
#endif /* _RTSPAPI_H */

/*******************************************************************************
 End of File
*/

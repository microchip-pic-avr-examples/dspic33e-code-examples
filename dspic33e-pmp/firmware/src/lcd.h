/*******************************************************************************
  LCD driver header file

  Company:
    Microchip Technology Inc.

  File Name:
    lcd.h

  Summary:
    LCD function definitions.

  Description:
    This file consists of the definitions for the LCD APIs.
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
// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <stdint.h>

#ifdef __cplusplus  // Provide C++ Compatability
extern "C"
{
    #endif

    // *****************************************************************************
    // *****************************************************************************
    // Section: Constants
    // *****************************************************************************
    // *****************************************************************************
    // Display line length.
    #define LCD_DISPLAY_LEN 16

    // Interface variables used by control macros.
    extern uint16_t _uLCDloops;
    extern uint16_t _uLCDchar;
    extern uint16_t _uLCDstate;
#ifdef TEST_MODE
extern unsigned char test_flag;
#endif

    /*****************************************************************************
 * Function: LCDProcessEvents
 *
 * Preconditions: None.
 *
 * Overview: This is a state mashine to issue commands and data to LCD. Must be
 * called periodically to make LCD message processing.
 *
 * Input: None.
 *
 * Output: None.
 *
 *****************************************************************************/
    void            LCDProcessEvents( void );

    /*****************************************************************************
 * Macro: mLCDIsBusy
 *
 * Preconditions: None.
 *
 * Overview: Query if the LCD is busy processing.
 *
 * Input: None.
 *
 * Output: Macro returns zero if LCD is not busy.
 *
 *****************************************************************************/
    #define mLCDIsBusy()    _uLCDstate

    /*****************************************************************************
 * Macro: mLCDInit
 *
 * Preconditions: None.
 *
 * Overview: Init the LCD.
 *
 * Input: None.
 *
 * Output: None.
 *
 *****************************************************************************/
    #define mLCDInit()  _uLCDstate = 2;

    /*****************************************************************************
 * Macro: mLCDPutChar
 *
 * Preconditions: Call of mLCDInit must be done.
 *
 * Overview: Put a character on the display.
 *
 * Input: Character.
 *
 * Output: None.
 *
 *****************************************************************************/
    #define mLCDPutChar( __lcd_char ) \
        _uLCDchar = __lcd_char;       \
        _uLCDstate = 3;

    /*****************************************************************************
 * Macro: mLCDPutChar
 *
 * Preconditions: None.
 *
 * Overview: Send a generic command.
 *
 * Input: Command.
 *
 * Output: None.
 *
 *****************************************************************************/
    #define mLCDPutCmd( __lcd_cmd ) \
        _uLCDchar = __lcd_cmd;      \
        _uLCDstate = 6;

    /*****************************************************************************
 * Macro: mLCDClear
 *
 * Preconditions: None.
 *
 * Overview: Clear the display.
 *
 * Input: None.
 *
 * Output: None.
 *
 *****************************************************************************/
    #define mLCDClear() _uLCDstate = 4;

    /*****************************************************************************
 * Macro: mLCDHome
 *
 * Preconditions: None.
 *
 * Overview: Home the display.
 *
 * Input: None.
 *
 * Output: None.
 *
 *****************************************************************************/
    #define mLCDHome()  \
        _uLCDstate = 5; \
        _uLCDchar = 0;

    /*****************************************************************************
 * Macro: mLCDEMode
 *
 * Preconditions: None.
 *
 * Overview: Set the mode, dir and shift.
 *
 * Input: Command.
 *
 * Output: None.
 *
 *****************************************************************************/
    #define mLCDEMode( __lcd_cmd )    \
        _uLCDchar = __lcd_cmd | 0x04; \
        _uLCDstate = 6;

    /*****************************************************************************
 * Macro: mLCDCtl
 *
 * Preconditions: None.
 *
 * Overview: Set the display control, on/off, cursor.
 *
 * Input: Command.
 *
 * Output: None.
 *
 *****************************************************************************/
    #define mLCDCtl( __lcd_cmd )      \
        _uLCDchar = __lcd_cmd | 0x08; \
        _uLCDstate = 6;

    //
    #define mLCDShift( __lcd_cmd )    \
        _uLCDchar = __lcd_cmd | 0x10; \
        _uLCDstate = 6;

    /*****************************************************************************
 * Macro: mLCDFSet
 *
 * Preconditions: None.
 *
 * Overview: Set the interface.
 *
 * Input: Address.
 *
 * Output: None.
 *
 *****************************************************************************/
    #define mLCDFSet( __lcd_cmd )     \
        _uLCDchar = __lcd_cmd | 0x20; \
        _uLCDstate = 6;

    /*****************************************************************************
 * Macro: mLCDCDAddr
 *
 * Preconditions: None.
 *
 * Overview: Set the CGRAM address.
 *
 * Input: Address.
 *
 * Output: None.
 *
 *****************************************************************************/
    #define mLCDCDAddr( __lcd_addr )   \
        _uLCDchar = __lcd_addr | 0x40; \
        _uLCDstate = 6;

    /*****************************************************************************
 * Macro: mLCDAddr
 *
 * Preconditions: None.
 *
 * Overview: Set the DDRAM address.
 *
 * Input: Address.
 *
 * Output: None.
 *
 *****************************************************************************/
    #define mLCDAddr( __lcd_addr )     \
        _uLCDchar = __lcd_addr | 0x80; \
        _uLCDstate = 6;

    #ifdef __cplusplus  // Provide C++ Compatibility
}

#endif

/*****************************************************************************
 * EOF
 *****************************************************************************/

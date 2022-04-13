/*******************************************************************************
  Explorer 16 LCD function prototype header file

  Company:
    Microchip Technology Inc.

  File Name:
    lcd_exp16brd.h

  Summary:
    Contains the prototypes for the LCD routines that support explorer 16 board.

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

#ifdef __cplusplus                      // Provide C++ Compatability
extern "C"
{
    #endif

    // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Routines
    // *****************************************************************************
    // *****************************************************************************
    /******    LCD FUNCTION PROTOYPES ******/
    void    Init_LCD( void );           // initialize display
    void    LCD_Cmd( int8_t cmd );      // write command to lcd
    void    LCD_Data( int8_t data );    // write data to lcd
    void    Puts_LCD( uint8_t *data, uint8_t count );

    // *****************************************************************************
    // *****************************************************************************
    // Section: Helper Macros
    // *****************************************************************************
    // *****************************************************************************

    /*****    LCD COMMAND FUCNTION PROTOTYPES  *****/
    #define cursor_right()  LCD_Cmd( 0x14 )
    #define cursor_left()   LCD_Cmd( 0x10 )
    #define display_shift() LCD_Cmd( 0x1C )
    #define home_clr()      LCD_Cmd( 0x01 )
    #define home_it()       LCD_Cmd( 0x02 )
    #define line_2()        LCD_Cmd( 0xC0 ) // (0xC0)
    #ifdef __cplusplus                      // Provide C++ Compatibility
}

#endif

/*******************************************************************************
 End of File
*/

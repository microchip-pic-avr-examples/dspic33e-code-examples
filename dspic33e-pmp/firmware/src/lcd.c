/*******************************************************************************
  LCD driver function
  
  Company:
    Microchip Technology Inc.

  File Name:
    lcd.c

  Summary:
     Function to issue commands and data to LCD

  Description:
    function has a state machine to issue commands and data to LCD. Must be
    called periodically to make LCD message processing.
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
#include <xc.h>
#include <system.h>
#include <stdint.h>

// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************
// Define a fast instruction execution time in terms of loop time
// typically > 43us
#define LCD_F_INSTR 10

// Define a slow instruction execution time in terms of loop time
// typically > 1.35ms
#define LCD_S_INSTR 150

// Define the startup time for the LCD in terms of loop time
// typically > 30ms
#define LCD_STARTUP 2000

#define _LCD_IDLE( __cnt ) \
    _uLCDstate = 1;        \
    _uLCDloops = __cnt;
#define _LCD_INIT( __cnt ) \
    _uLCDstate++;          \
    _uLCDloops = __cnt;

uint16_t    _uLCDloops;
uint16_t    _uLCDstate;
uint16_t    _uLCDchar;

/******************************************************************************
 * Function:        void LCDProcessEvents(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:       This is a state machine to issue commands and data to LCD.
 *                 Must be called periodically to make LCD message processing.
 *****************************************************************************/
void LCDProcessEvents( void )
{
#ifdef TEST_MODE
    uint16_t test_char=0;
#endif
    switch( _uLCDstate )
    {
        case 1:                 // *** wait ***
            if( _uLCDloops )
            {
                _uLCDloops--;
            }
            else
            {
                _uLCDstate = 0;
            }

            break;

        case 2:                 // *** init ***
            PMCON = 0x83BF;     // Setup the PMP
            PMMODE = 0x03FF;
            PMAEN = 0x0001;
            PMADDR = 0x0000;
            _uLCDstate = 64;    // Set the next state
            _uLCDloops = LCD_STARTUP;   // Set the entry delay
            break;

        case 3:     // *** put ***
            _LCD_IDLE( LCD_F_INSTR );
            PMADDR = 0x0001;
            PMDIN1 = _uLCDchar;

            //   UART2PutChar(_uLCDchar);        // Copy character to UART
            break;

        case 4:     // *** clear ***
            _LCD_IDLE( LCD_S_INSTR );
            PMADDR = 0x0000;
            PMDIN1 = 0b00000001;

            //     UART2PutChar('\r');             // Send return to UART
            break;

        case 5:     // *** home ***
            _LCD_IDLE( LCD_S_INSTR );
            PMADDR = 0x0000;
            PMDIN1 = 0b00000010;

            //       UART2PutChar('\r');             // Send return to UART
            break;

        case 6:     // *** command ***
            _LCD_IDLE( LCD_F_INSTR );
            PMADDR = 0x0000;
            PMDIN1 = _uLCDchar;
#ifdef TEST_MODE
            test_char = _uLCDchar;
            if((PMADDR == 0x0000) && (PMDIN1 == test_char) )
                test_flag=1;
#endif
            break;

        // This is the LCD init state machine
        case 64:    // Standard delay states for the init
        case 66:
        case 68:
        case 70:    // Programmable delay loop
            if( _uLCDloops )
            {
                _uLCDloops--;
            }
            else
            {
                _uLCDstate++;
            }

            break;

        case 65:
            _LCD_INIT( LCD_F_INSTR );   //DELAY
            PMDIN1 = 0b00111100;       // Set the default function
            break;

        case 67:
            _LCD_INIT( LCD_F_INSTR );
            PMDIN1 = 0b00001100;       // Set the display control
            break;

        case 69:
            _LCD_INIT( LCD_S_INSTR );
            PMDIN1 = 0b00000001;       // Clear the display
            break;

        case 71:
            _LCD_INIT( LCD_S_INSTR );
            PMDIN1 = 0b00000110;       // Set the entry mode
            break;

        case 72:
            _uLCDstate = 0;
            break;

        default:
            _uLCDstate = 0;
            break;
    }
}

/*****************************************************************************
 * EOF
 *****************************************************************************/

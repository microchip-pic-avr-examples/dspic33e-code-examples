/*******************************************************************************
  Explorer 16 board LCD support source file

  Company:
    Microchip Technology Inc.

  File Name:
    lcd_exp16brd.c

  Summary:
    Contains LCD routines for explorer 16 board support.

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
#include <stdint.h>
#include "lcd_exp16brd.h"
#include "delay.h"

/* 
   For Explorer 16 board, here are the data and control signal definitions
   RS -> RB15
   E  -> RD4
   RW -> RD5
   DATA -> RE0 - RE7   
*/
// Control signal data pins
#define RW  LATDbits.LATD5  // LCD R/W signal
#define RS  LATBbits.LATB15 // LCD RS signal
#define E   LATDbits.LATD4  // LCD E signal

// Control signal pin direction
#define RW_TRIS TRISDbits.TRISD5
#define RS_TRIS TRISBbits.TRISB15
#define E_TRIS  TRISDbits.TRISD4

// Data signals and pin direction
#define DATA        LATE    // Port for LCD data
#define DATAPORT    PORTE
#define TRISDATA    TRISE   // I/O setup for data Port

/******************************************************************************
 * Function:        Init_LCD()
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is used to initialize the LCD.
 *****************************************************************************/
void Init_LCD( void )   // initialize LCD display
{
    // 15mS delay after Vdd reaches nnVdc before proceeding with LCD initialization
    // not always required and is based on system Vdd rise rate
    Delay( Delay_15mS_Cnt );        // 15ms delay

    /* set initial states for the data and control pins */
    LATE &= 0xFF00;
    RW = 0;                         // R/W state set low
    RS = 0;                         // RS state set low
    E = 0;                          // E state set low

    /* set data and control pins to outputs */
    TRISE &= 0xFF00;
    RW_TRIS = 0;                    // RW pin set as output
    RS_TRIS = 0;                    // RS pin set as output
    E_TRIS = 0;                     // E pin set as output

    /* 1st LCD initialization sequence */
    DATA &= 0xFF00;
    DATA |= 0x0038;
    E = 1;
    Nop();
    Nop();
    Nop();
    E = 0;                          // toggle E signal
    Delay( Delay_5mS_Cnt );         // 5ms delay

    /* 2nd LCD initialization sequence */
    DATA &= 0xFF00;
    DATA |= 0x0038;
    E = 1;
    Nop();
    Nop();
    Nop();
    E = 0;                          // toggle E signal
    Delay_Us( Delay200uS_count );   // 200uS delay

    /* 3rd LCD initialization sequence */
    DATA &= 0xFF00;
    DATA |= 0x0038;
    E = 1;
    Nop();
    Nop();
    Nop();
    E = 0;  // toggle E signal
    Delay_Us( Delay200uS_count );   // 200uS delay
    LCD_Cmd( 0x38 );                // function set
    LCD_Cmd( 0x0C );                // Display on/off control, cursor blink off (0x0C)
    LCD_Cmd( 0x06 );                // entry mode set (0x06)
}

/******************************************************************************
 * Function:        LCD_Cmd()
 *
 * PreCondition:    None
 *
 * Input:           cmd -> command to be sent
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:         This function serves as subroutiune for lcd commands
 *****************************************************************************/
void LCD_Cmd( int8_t cmd )  // subroutiune for lcd commands
{
    DATA &= 0xFF00;         // prepare RD0 - RD7
    DATA |= cmd;            // command byte to lcd
    RW = 0;                 // ensure RW is 0
    RS = 0;
    E = 1;                  // toggle E line
    Nop();
    Nop();
    Nop();
    E = 0;
    Delay( Delay_5mS_Cnt ); // 5ms delay
}

/******************************************************************************
 * Function:        LCD_Data
 *
 * PreCondition:    None
 *
 * Input:           data -> data to be sent
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:         This function serves as subroutiune for lcd data
 *****************************************************************************/
void LCD_Data( int8_t data )    // subroutine for lcd data
{
    RW = 0;         // ensure RW is 0
    RS = 1;         // assert register select to 1
    DATA &= 0xFF00; // prepare RD0 - RD7
    DATA |= data;   // data byte to lcd
    E = 1;
    Nop();
    Nop();
    Nop();
    E = 0;          // toggle E signal
    RS = 0;         // negate register select to 0
    Delay_Us( Delay200uS_count );   // 200uS delay
    Delay_Us( Delay200uS_count );   // 200uS delay
}

/******************************************************************************
 * Function:        Puts_LCD
 *
 * PreCondition:    None
 *
 * Input:           uint8_t *data, uint8_t count
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:         This function displays the data sent to LCD.
 *****************************************************************************/
void Puts_LCD( uint8_t *data, uint8_t count )
{
    while( count )
    {
        LCD_Data( *data++ );
        count--;
    }
}

/*******************************************************************************
 End of File
*/

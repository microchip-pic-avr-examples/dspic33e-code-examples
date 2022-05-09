/*******************************************************************************
  banner function
  
  Company:
    Microchip Technology Inc.

  File Name:
    banner.c

  Summary:
    This file is used to maintain data to be transmitted to LCD.

  Description: In this function defines array of data and calls a swicth
               function to transfer these array of data to LCD in a
               perticular array.
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
/*****************************************************************************
*  Banners strings.
 *****************************************************************************/
const int8_t    _T1[] = "Microchip       ";
const int8_t    _T2[] = "Technology, Inc ";

const int8_t    _T3[] = "Presenting the d ";
const int8_t    _T4[] = "sPIC33EP512GM10 ";

const int8_t    _T5[] = "Copyright 2012  ";
const int8_t    _T6[] = "                ";

const int8_t    _T7[] = "16-bit          ";
const int8_t    _T8[] = "Microcontroller ";

const int8_t    _T9[] = "60MIPS / 60MHz  ";
const int8_t    _T10[] = "3.0V - 3.6V     ";

const int8_t    _T11[] = "Features:       ";
const int8_t    _T12[] = "3 SPI modules   ";

const int8_t    _T13[] = "2 I2C module    ";
const int8_t    _T14[] = "4 UARTs w/ IrDA ";

const int8_t    _T15[] = "New Parallel    ";
const int8_t    _T16[] = "Master Port     ";

const int8_t    _T17[] = "10/12-bit A/D   ";
const int8_t    _T18[] = "16bit Audio DAC ";

const int8_t    _T19[] = "2 Enhanced CAN  ";
const int8_t    _T20[] = "8 Output compare";

const int8_t    _T21[] = "8 Input Capture ";
const int8_t    _T22[] = "Real-time clock ";

const int8_t    _T23[] = "and calendar    ";
const int8_t    _T24[] = "Watchdog Timer  ";

const int8_t    _T25[] = "On-chip voltage ";
const int8_t    _T26[] = "regulator       ";

const int8_t    _T27[] = "9 16-bit timers ";
const int8_t    _T28[] = "4 32-bit timers ";

const int8_t    _T29[] = "Many oscillator ";
const int8_t    _T30[] = "modes           ";

const int8_t    _T31[] = "7.37MHz internal";
const int8_t    _T32[] = "oscillator      ";

const int8_t    _T33[] = "Pin Remappable  ";
const int8_t    _T34[] = "Configuration   ";

// Last banner is showed at start only
const int8_t    _T35[] = "  Explorer 16   ";
const int8_t    _T36[] = "Development Brd ";

// Specify delay between banners
#define BNR_CHANGE_DELAY    1000

// Quantity of Banners - 2 (last banner is showed at start only and never counted again)
#define BNR_COUNT   34

/*****************************************************************************
*  Array of pointers to banners strings
******************************************************************************/
const int8_t    *_pBannersArray[] = { _T1, _T2, _T3, _T4, _T5, _T6, _T7, _T8, _T9, _T10, _T11, _T12, _T13, _T14, _T15, _T16, _T17, _T18, _T19, _T20, _T21, _T22, _T23, _T24, _T25, _T26, _T27, _T28, _T29, _T30, _T31, _T32, _T33, _T34, _T35, _T36 };

uint16_t        _uBannerNum;
uint16_t        _uBannerState;
const int8_t    *_pBanner;
uint16_t        _uBannerLen;
uint16_t        _uBannerWait;
uint16_t        _uBannerCharWait;

/******************************************************************************
 * Function:        void BannerStart(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:       The function starts to show banners from the last one.
 *                 The banner is displayed once.
 *****************************************************************************/
void BannerStart( void )
{
    _uBannerState = 3;
    _uBannerNum = 34;
    _uBannerWait = 2000;
}

/******************************************************************************
 * Function:        void BannerInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:       The function starts to show banners  from the first one.
 *****************************************************************************/
void BannerInit( void )
{
    _uBannerState = 9;
    _uBannerNum = 0;
    _uBannerWait = 20;
}

/******************************************************************************
 * Function:        void BannerProcessEvents(void)
 *
 * PreCondition:    BannerInit or BannerStart must be called before.
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:       The function implements a state mashine to display banners
 *                 sequence. Must be called periodically to output the strings.
 *****************************************************************************/
void BannerProcessEvents( void )
{
    switch( _uBannerState )
    {
        case 4: // Wait to put a char
        case 7:
            if( _uBannerCharWait )
            {
                _uBannerCharWait--;
            }
            else
            {
                _uBannerState++;
                _uBannerCharWait = 1;
            }

            break;

        case 5: // Put a char on the LCD
        case 8:
            if( !mLCDIsBusy() )
            {
                mLCDPutChar( *_pBanner );
                _pBanner++;
                _uBannerLen--;
                if( !_uBannerLen )
                {
                    _uBannerState++;
                }
            }

            break;

        case 3: // Put the first line
            if( !mLCDIsBusy() )
            {
                mLCDHome();
                _pBanner = _pBannersArray[_uBannerNum];
                _uBannerNum++;
                _uBannerLen = LCD_DISPLAY_LEN;
                _uBannerState++;
            }

            break;

        case 6: // Put the second line
            if( !mLCDIsBusy() )
            {
                mLCDPutCmd( 0xC0 );
                _pBanner = _pBannersArray[_uBannerNum];
                _uBannerNum++;
                _uBannerLen = LCD_DISPLAY_LEN;
                _uBannerState++;
            }

            break;

        case 9: // Wait at the end of each banner
            if( _uBannerWait-- )
            {
                break;
            }

            if( _uBannerNum >= BNR_COUNT )
            {
                _uBannerNum = 0;
            }

            _uBannerWait = BNR_CHANGE_DELAY;
            _uBannerState = 3;
            break;

        default:
            _uBannerState = 3;
    }
}

/*****************************************************************************
* EOF
******************************************************************************/

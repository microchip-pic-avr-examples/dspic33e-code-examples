/*******************************************************************************
  I2C funtions
  
  Company:
    Microchip Technology Inc.

  File Name:
  i2c_slavedrv.c

  Summary:
    This file is used to configure I2C.

  Description:
    This code example shows how to use I2C module in slave mode.
 The master I2C device uses the slave I2C device as RAM.
 Thus master I2C device can read/write particular RAM area of I2C slave device.
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
#include "i2c_slavedrv.h"

/*****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************/
uint8_t         ramBuffer[256]; //RAM area which will work as EEPROM for Master I2C device
uint8_t         *ramPtr;        //Pointer to RAM memory locations
struct FlagType flag;

// *****************************************************************************
// Section: Function definition
// *****************************************************************************
// *****************************************************************************

/******************************************************************************
 * Function:       void I2C1_Init(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Initializes I2C1 peripheral as Slave.
 *****************************************************************************/
void I2C1_Init( void )
{
    #if !defined( USE_I2C_Clock_Stretch )
    I2C1CON = 0x8000;       //Enable I2C1 module
    #else
    I2C1CON = 0x9040;       //Enable I2C1 module, enable clock stretching
    #endif
    I2C1ADD = 0x50;         // 7-bit I2C slave address must be initialised here.
    IFS1 = 0;
    ramPtr = &ramBuffer[0]; //set the RAM pointer and points to beginning of ramBuffer
    flag.AddrFlag = 0;      //Initlize Addflag
    flag.DataFlag = 0;      //Initlize Dataflag
    _SI2C1IE = 1;
}

/******************************************************************************
 * Function:   void __attribute__((interrupt,no_auto_psv)) _SI2C1Interrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This is the ISR for I2C1 Slave interrupt.
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _SI2C1Interrupt( void )
{
    unsigned char   temp;   //used for dummy read
    if( (I2C1STATbits.R_W == 0) && (I2C1STATbits.D_A == 0) )    //Address matched
    {
        temp = I2C1RCV;     //dummy read
        flag.AddrFlag = 1;  //next byte will be address
    }
    else if( (I2C1STATbits.R_W == 0) && (I2C1STATbits.D_A == 1) )   //check for data
    {
        if( flag.AddrFlag )
        {
            flag.AddrFlag = 0;
            flag.DataFlag = 1;                      //next byte is data
            ramPtr = ramPtr + I2C1RCV;

            #if defined( USE_I2C_Clock_Stretch )
            I2C1CONbits.SCLREL = 1;                 //Release SCL1 line
            #endif
        }
        else if( flag.DataFlag )
        {
            *ramPtr = ( unsigned char ) I2C1RCV;    // store data into RAM
            flag.AddrFlag = 0;                      //end of tx
            flag.DataFlag = 0;
            ramPtr = &ramBuffer[0];                 //reset the RAM pointer
            #if defined( USE_I2C_Clock_Stretch )
            I2C1CONbits.SCLREL = 1;                 //Release SCL1 line
            #endif
        }
    }
    else if( (I2C1STATbits.R_W == 1) && (I2C1STATbits.D_A == 0) )
    {
        temp = I2C1RCV;
        I2C1TRN = *ramPtr;      //Read data from RAM & send data to I2C master device
        I2C1CONbits.SCLREL = 1; //Release SCL1 line
        while( I2C1STATbits.TBF );

        //Wait till all
        ramPtr = &ramBuffer[0]; //reset the RAM pointer
    }

    _SI2C1IF = 0;               //clear I2C1 Slave interrupt flag
}

/*******************************************************************************
 End of File
*/

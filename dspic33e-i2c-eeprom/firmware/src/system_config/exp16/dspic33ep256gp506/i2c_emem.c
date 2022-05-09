/*******************************************************************************
  I2C with EEPROM source file

  Company:
    Microchip Technology Inc.

  File Name:
    i2c_emem.c

  Summary:
    Handles the operation of I2C with EEPROM.

  Description:
    This source file contains a state machine that enables the I2C module to
    communicate with an external EEPROM module. The state machine also has the
    ACK polling feature that is necessary when using an external EEPROM.
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
#include "i2c_emem.h"

uint16_t    jDone;

/*=============================================================================

=============================================================================*/
/******************************************************************************
 * Function:  void __attribute__((interrupt, no_auto_psv)) _MI2C1Interrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This serves the I2C Master Interrupt Service Routine.
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _MI2C1Interrupt( void )
{
    jDone = 1;
    IFS1bits.MI2C1IF = 0;   //Clear the DMA0 Interrupt Flag;
}

/******************************************************************************
 * Function:  void __attribute__((interrupt, no_auto_psv)) _SI2C1Interrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This serves the I2C Slave interrupt.
 *****************************************************************************/
void __attribute__ ( (interrupt, no_auto_psv) ) _SI2C1Interrupt( void )
{
    IFS1bits.SI2C1IF = 0;   //Clear the DMA0 Interrupt Flag
}

/******************************************************************************
 * Function:        void I2CEMEMinit(I2CEMEM_DRV *i2cMem)
 *
 * PreCondition:    None
 *
 * Input:           *i2cMem
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        I2C initialization/configuration function.
 *                  This function is used to configure the I2C module as the
 *                  master and use 8-bit mode for communication
 *                  with the serial EEPROM.
 *****************************************************************************/
void I2CEMEMinit( I2CEMEM_DRV *i2cMem )
{
    i2cMem->cmd = 0;
    i2cMem->oData = 0;

    // Configre SCA/SDA pin as open-drain. This may change from device to device.
    //Refer the datasheet for more information.
    ODCCbits.ODCC4 = 0;
    ODCCbits.ODCC5 = 0;

    I2C1CONbits.A10M = 0;
    I2C1CONbits.SCLREL = 1;
    I2C1BRG = 300;

    I2C1ADD = 0;
    I2C1MSK = 0;

    I2C1CONbits.I2CEN = 1;
    IEC1bits.MI2C1IE = 1;
    IFS1bits.MI2C1IF = 0;
}

/******************************************************************************
 * Function:        void I2CEMEMdrv(I2CEMEM_DRV *i2cMem)
 *
 * PreCondition:    None
 *
 * Input:           *i2cMem
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        I2C communication state machine.
 *                  This function is a state machine based on which different
 *                  set of actions is performed by the I2C master while
 *                  reading/writing from/to the slave device which is in this
 *                  case the serial EEPROM
 *****************************************************************************/
void I2CEMEMdrv( I2CEMEM_DRV *i2cMem )
{
    static int16_t  state = 0;

    static int16_t  cntr = 0;

    static int16_t  rtrycntr = 0;

    switch( state )
    {
        case 0:
            if( (i2cMem->cmd == I2C_WRITE) || (i2cMem->cmd == I2C_READ) )
            {
                state = 1;
            }

            break;

        /*==================================*/
        /* Control/Address Phase            */
        /*==================================*/
        case 1:
            // Start Condition
            I2C1CONbits.SEN = 1;
            state = state + 1;
            break;

        case 2:
            // Start Byte with device select id
            if( jDone == 1 )
            {
                jDone = 0;
                state = state + 1;

                //    I2C1TRN=(0x00A0)|(((i2cMem->oData->csel)&0x7)<<1);
                I2C1TRN = 0x00A0;
            }

            break;

        case 3:
            // Send address byte 1, if ack is received. Else Retry
            if( jDone == 1 )
            {
                jDone = 0;

                if( I2C1STATbits.ACKSTAT == 1 )
                {                   // Ack Not received, Retry
                    if( rtrycntr < MAX_RETRY )
                    {
                        state = 18;
                    }
                    else
                    {
                        state = 16; // Flag error and exit
                    }
                }
                else
                {
                    rtrycntr = 0;

                    #if ADDRWIDTH == TWO_BYTE
                    I2C1TRN = ( (i2cMem->oData->addr) & 0xFF00 ) >> 8;
                    state = state + 1;
                    #endif
                    #if ADDRWIDTH == ONE_BYTE
                    I2C1TRN = ( (i2cMem->oData->addr) );
                    state = state + 2;
                    #endif
                }
            }

            break;

        case 4:
            // Send address byte 2, if ack is received. Else Flag error and exit
            if( jDone == 1 )
            {
                jDone = 0;

                if( I2C1STATbits.ACKSTAT == 1 )
                {                   // Ack Not received, Flag error and exit
                    state = 16;
                }
                else
                {
                    #if ADDRWIDTH == TWO_BYTE
                    I2C1TRN = ( (i2cMem->oData->addr) & 0x00FF );
                    #endif
                    state = state + 1;
                }
            }

            break;

        case 5:
            // Read or Write
            if( jDone == 1 )
            {
                jDone = 0;

                if( I2C1STATbits.ACKSTAT == 1 )
                {                   // Ack Not received, Flag error and exit
                    state = 16;
                }
                else
                {
                    if( i2cMem->cmd == I2C_WRITE )
                    {
                        state = state + 1;
                    }

                    if( i2cMem->cmd == I2C_READ )
                    {
                        state = 8;
                    }
                }
            }

            break;

        /*==================================*/
        /* Write Data Phase                    */
        /*==================================*/
        case 6:
            // Send data
            I2C1TRN = *( i2cMem->oData->buff + cntr );
            state = state + 1;
            cntr = cntr + 1;
            break;

        case 7:
            // Look for end of data or no Ack
            if( jDone == 1 )
            {
                jDone = 0;
                state = state - 1;

                if( I2C1STATbits.ACKSTAT == 1 )
                {                   // Ack Not received, Flag error and exit
                    state = 16;
                }
                else
                {
                    if( cntr == i2cMem->oData->n )
                    {               //                    state=14;   // Close the Frame without ACK polling
                        state = 20; // Go to ACK polling to wait for write to complete
                    }
                }
            }

            break;

        /*==================================*/
        /* Read Data Phase                    */
        /*==================================*/
        case 8:
            // Repeat Start
            I2C1CONbits.RSEN = 1;
            state = state + 1;
            break;

        case 9:
            // Re-send control byte with W/R=R
            if( jDone == 1 )
            {
                jDone = 0;
                state = state + 1;
                I2C1TRN = ( 0x00A1 ) | ( ((i2cMem->oData->csel) & 0x7) << 1 );
            }

            break;

        case 10:
            // Check, if control byte went ok
            if( jDone == 1 )
            {
                jDone = 0;
                state = state + 1;

                if( I2C1STATbits.ACKSTAT == 1 )
                {                   // Ack Not received, Flag error and exit
                    state = 16;
                }
            }

            break;

        case 11:
            // Receive Enable
            I2C1CONbits.RCEN = 1;
            state++;
            break;

        case 12:
            // Receive data
            if( jDone == 1 )
            {
                jDone = 0;
                state = state + 1;

                *( i2cMem->oData->buff + cntr ) = I2C1RCV;
                cntr++;

                if( cntr == i2cMem->oData->n )
                {
                    I2C1CONbits.ACKDT = 1;  // No ACK
                }
                else
                {
                    I2C1CONbits.ACKDT = 0;  // ACK
                }

                I2C1CONbits.ACKEN = 1;
            }

            break;

        case 13:
            if( jDone == 1 )
            {
                jDone = 0;
                if( cntr == i2cMem->oData->n )
                {
                    state = state + 1;
                }
                else
                {
                    state = state - 2;
                }
            }

            break;

        /*==================================*/
        /* Stop Sequence                    */
        /*==================================*/
        case 14:
            I2C1CONbits.PEN = 1;
            state++;
            break;

        case 15:
            if( jDone == 1 )
            {
                jDone = 0;
                state = 0;
                cntr = 0;
                i2cMem->cmd = 0;
            }

            break;

        /*==================================*/
        /* Set Error                         */
        /*==================================*/
        case 16:
            I2C1CONbits.PEN = 1;
            state++;
            break;

        case 17:
            if( jDone == 1 )
            {
                jDone = 0;
                state = 0;
                rtrycntr = 0;
                cntr = 0;
                i2cMem->cmd = 0xFFFF;
            }

            break;

        /*==================================*/
        /* Retry                             */
        /*==================================*/
        case 18:
            I2C1CONbits.PEN = 1;
            state++;
            rtrycntr++;
            break;

        case 19:
            if( jDone == 1 )
            {
                jDone = 0;
                state = 0;
                cntr = 0;
            }

            break;

            /*==================================*/
            /* ACK Polling                        */
            /*==================================*/
            int acktest;

        case 20:
            I2C1CONbits.PEN = 1;
            while( !_MI2C1IF );
            while( _MI2C1IF );
            acktest = 1;
            while( acktest )
            {
                I2C1CONbits.RSEN = 1;
                while( !_MI2C1IF );
                while( _MI2C1IF );
                I2C1TRN = 0x00A0;
                while( _TRSTAT );
                acktest = I2C1STATbits.ACKSTAT;
            }

            I2C1CONbits.PEN = 1;
            while( !_MI2C1IF );
            while( _MI2C1IF );
            jDone = 0;
            state = 14;
            break;
    }
}

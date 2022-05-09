/*******************************************************************************
  I2C with EEPROM header file

  Company:
    Microchip Technology Inc.

  File Name:
    i2c_emem.h

  Summary:
    COntains prototypes for the I2C with EEPROM driver.

  Description:
    This header file has the prototype for the I2C driver. It also defines stuctures for the data
    object and the driver object that is made use in the driver function.
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
#ifndef __I2CEMEM_H__
    #define __I2CEMEM_H__

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
        #define MAX_RETRY   3
        #define ONE_BYTE    1
        #define TWO_BYTE    2

    // EEPROM ADDRESS SIZE
        #define ADDRWIDTH   ONE_BYTE

    // EEPROM DRIVER COMMAND DEFINITION
        #define I2C_IDLE    0
        #define I2C_WRITE   1
        #define I2C_READ    2
        #define I2C_ERR     0xFFFF

    // EEPROM DATA OBJECT
    typedef struct
    {
        uint16_t    *buff;
        uint16_t    n;
        uint16_t    addr;
        uint16_t    csel;
    } I2CEMEM_DATA;

    // EEPROM DRIVER OBJECT
    typedef struct
    {
        uint16_t        cmd;
        I2CEMEM_DATA    *oData;
        void ( *init ) ( void * );
        void ( *tick ) ( void * );
    }
    I2CEMEM_DRV;

        #define I2CSEMEM_DRV_DEFAULTS                                                                      \
        {                                                                                                  \
            0, ( I2CEMEM_DATA * ) 0, ( void(*) ( void * ) ) I2CEMEMinit, ( void(*) ( void * ) ) I2CEMEMdrv \
        }

    void I2CEMEMinit( I2CEMEM_DRV * );
    void I2CEMEMdrv( I2CEMEM_DRV * );

        #ifdef __cplusplus  // Provide C++ Compatibility
}

    #endif
#endif

/*******************************************************************************
 End of File
*/

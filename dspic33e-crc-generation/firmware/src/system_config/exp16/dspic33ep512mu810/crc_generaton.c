/*******************************************************************************
  ce451 main faunction
  
  Company:
    Microchip Technology Inc.

  File Name:
    crc_generation.c

  Summary:
    This file is used to call CRC APIs

  Description:
    The main.c is used to call the CRC Initialization function and function to calculate checksum.
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
#include "crc.h"

#if __XC16_VERSION == 1011
#warning "XC16 v1.11 detected. It is recommended that a newer version of XC16 be used."
#endif

// DSPIC33EP512MU810 Configuration Bit Settings
// FICD
#pragma config ICS = PGD1           // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config JTAGEN = OFF         // JTAG Enable bit (JTAG is disabled)

// FPOR
#pragma config BOREN = ON           // Brown-out Reset (BOR) Detection Enable bit (BOR is enabled)
#pragma config ALTI2C1 = OFF        // Alternate I2C1 pins (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2 = OFF        // Alternate I2C2 pins (I2C2 mapped to SDA2/SCL2 pins)

// FWDT
#pragma config WDTPOST = PS32768    // Watchdog Timer Postscaler bits (1:32,768)
#pragma config WDTPRE = PR128       // Watchdog Timer Prescaler bit (1:128)
#pragma config PLLKEN = ON          // PLL Lock Enable bit (Clock switch to PLL source will wait until the PLL lock signal is valid.)
#pragma config WINDIS = OFF         // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF         // Watchdog Timer Enable bit (Watchdog timer enabled/disabled by user software)

// FOSC
#pragma config POSCMD = XT          // Primary Oscillator Mode Select bits (XT Crystal Oscillator Mode)
#pragma config OSCIOFNC = OFF       // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config IOL1WAY = OFF        // Peripheral pin select configuration (Allow multiple reconfigurations)
#pragma config FCKSM = CSECMD       // Clock Switching Mode bits (Clock switching is enabled,Fail-safe Clock Monitor is disabled)

// FOSCSEL
#pragma config FNOSC = FRC          // Oscillator Source Selection (Internal Fast RC (FRC))

// FGS
#pragma config GWRP = OFF           // General Segment Write-Protect bit (General Segment may be written)

// Expected result depends on the contents of messageString[8] Array. If the content is changed
// the below CRC value may not suit. The below CRC checksum is for data 0x01 to 0x08(given).

#define EXPECTED_RESULT 0x76AC

// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************
uint8_t messageString[8] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};
uint16_t    crcResult=0;
/******************************************************************************
 * Function:        int main(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        Main function
 *****************************************************************************/
#ifdef TEST_MODE
int ce_main(void)
#else
int main(void)
#endif
{
    uint16_t    prev_CRC = 0x0000;
  
    CRC_Init();

     Nop();
     Nop();

    crcResult = CRC_ChecksumByte( messageString, 8, prev_CRC );
#ifdef TEST_MODE
    if(crcResult == EXPECTED_RESULT )
        return 0;
    else
        return 1;
#else
    while( 1 );
#endif
}

/******************************************************************************
 * Function:        void CRC_Init(void)
 *
 * PreCondition:    None   
 *
 * Input:           None
 *                  
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function configures CRC module
 *****************************************************************************/
void CRC_Init( void )
{

    CRCCON1bits.CRCEN = 0;
    CRCXORL = 0x0000;
         Nop();
         Nop();

    CRCWDATL = 0x0000;

    CRCWDATH = 0x0000; 	//  NOTE:Non-direct Initial value fed to CRCWDAT
    CRCXORL = POLY;             // Generator Polynomial // X^12 + X^5
    CRCXORH = 0x0000;
    CRCCON1bits.CRCISEL = 0;
    CRCCON2bits.PLEN = POLYLEN; // Length of polynomial-1"
    CRCCON2bits.DWIDTH = 7;
    CRCCON1bits.LENDIAN = 0x0;
    CRCCON1bits.CRCEN = 1;
}

/******************************************************************************
 * Function:        uint16_t CRC_ChecksumByte(uint8_t* data, uint8_t Number_of_bytes, uint16_t prev_CRC)
 *
 * PreCondition:    None
 *
 * Input:           data - Pointer to the first data byte for which CRC needs to be
 *                  calculated.
 *                  Number_of_bytess - Total number of bytes for which CRC needs to be
 *                  calculated.
 *                  prev_CRC - previous CRC result.
 *
 * Output:          CRCWDATL Register content that is the two byte checksum
 *
 * Side Effects:    None
 *
 * Overview:        Calculates the checksum and returns the value
 *****************************************************************************/
uint16_t CRC_ChecksumByte( uint8_t *data, uint8_t Number_of_bytes, uint16_t prev_CRC )
{
    uint8_t volatile    *dest = ( uint8_t * ) &CRCDATL;

    CRCWDATL = prev_CRC;
    IFS4bits.CRCIF=0;				//CRC status Flag is Cleared
    do
    {
       while( (0 == CRCCON1bits.CRCFUL) && (0 < Number_of_bytes) )
        {
            *dest = *data;
            data++;
            Number_of_bytes--;
        }
    } while( 0 < Number_of_bytes );
 CRCCON1bits.CRCGO = 1;
 CRCDATL = 0x0000;   /* Do this to shift the last word out of the CRC shift register */
 while( CRCCON1bits.CRCFUL == 1 );
    

   
        Nop();
        Nop();
        Nop();
        Nop();
        Nop();
        Nop();
	while(CRCCON1bits.CRCMPT!=1);

        Nop();
        Nop();
        Nop();
        Nop();
        Nop();
        Nop();
	while(IFS4bits.CRCIF!=1);

        Nop();
        Nop();
        Nop();
        Nop();
        Nop();
        Nop();
	CRCCON1bits.CRCGO=0;
        Nop();
        Nop();
        Nop();
        Nop();
        Nop();
    return ( CRCWDATL );
}

/*******************************************************************************
 End of File
 */

/*******************************************************************************
  ce416 main source file

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    Calls the DMA & SPI1 configure functions.

  Description:
    This source file calls the DMA configuration and SPI1 configuration functions for the operation of
    SPI1 in loopback. The SDI1 and SDO1 pins are externally connected so that the loopback is completed.
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
#include "spi1drv.h"

#if __XC16_VERSION < 1011
#warning "Please upgrade to XC16 v1.11 or newer."
#endif

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
 * Overview:        main function
 ***** main************************************************************************/
#ifdef TEST_MODE
unsigned char test_flag1,test_flag2, i,j;
#endif
#ifdef TEST_MODE
int ce_main(void)
#else
int main(void)
#endif
{

#ifdef TEST_MODE
    test_flag1= test_flag2= 0;
#endif
    // Configure Oscillator to operate the device at 60Mhz
    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
    // Fosc= 7.37M*65(2*2)=120Mhz for 7.37M input clock
    PLLFBD = 63;                                // M=65
    CLKDIVbits.PLLPOST = 0;                     // N1=2
    CLKDIVbits.PLLPRE = 0;                      // N2=2
    OSCTUN = 0;                                 // Tune FRC oscillator, if FRC is used

    // Disable Watch Dog Timer
    RCONbits.SWDTEN = 0;

    // Clock switch to incorporate PLL
    __builtin_write_OSCCONH( 0x01 );            // Initiate Clock Switch to

    // FRC with PLL (NOSC=0b001)
    __builtin_write_OSCCONL( OSCCON || 0x01 );  // Start clock switching
    while( OSCCONbits.COSC != 0b001 );

    // Wait for Clock switch to occur
    // Wait for PLL to lock
    while( OSCCONbits.LOCK != 1 )
    { };

    // SPI init with DMA
    InitSPIBuff();
    CfgDma0SpiTx();
    CfgDma1SpiRx();
    CfgSpi1Master();

#ifdef TEST_MODE
    while(1)
    {
    for(i=0;i<16;i++)
    {
        if(SPI1RxBuffA[i] == i)
            test_flag1=1;
        else
            test_flag1=0;
    }
     for(i=0;i<16;i++)
     {
         j=16+i;
         if(SPI1RxBuffB[i] == j )
             test_flag2=1;
         else
             test_flag2=0;
     }
    if( (test_flag1 ==1) && (test_flag2 ==1))
            return 0;

}
#else

    // Background loop
    while( 1 );
#endif

}

/*******************************************************************************
 End of File
*/

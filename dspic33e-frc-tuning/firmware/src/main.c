/*******************************************************************************
  ce410 main file

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    Toggles a pin continuously to monitor tuning of FRC oscillator.

  Description:
    Inside the main function, a port pin is toggled continuously at 1/4 the 
    system frequency. The device is run on the Internal FRC oscillator that
    can be tuned. The main function also calls the TuneFrcOsc function that
    performs the tuning of the oscillator. A change in the frequency of the
    pin toggling suggests that the tuning is functioning.
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
#include "tune_frc_osc.h"
#include "ctglpin.h"

#if __XC16_VERSION < 1011
#warning "Please upgrade to XC16 v1.11 or newer."
#endif

// *****************************************************************************
// *****************************************************************************
// Section: Macros for Configuration Fuse Registers:
// *****************************************************************************
// *****************************************************************************
/* Invoke macros to set up  device configuration fuse registers.The fuses will
   select the oscillator source, power-up timers, watch-dog timers etc. The
   macros are defined within the device header files. The configuration fuse
   registers reside in Flash memory.
 */
// Exported function
extern void CtglPin ( void );
extern void CtglPinInit( void );

// 6-bit 2s complement input control for interal FRC tuning
// tuneValue=31 will give high freq
// tuneValue=0 will give nominal freq
// tuneValue=32 will give low freq
uint16_t    tuneValue = 32;

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
uint16_t    tValue = 0;
int ce_main(void)
#else
int main(void)
#endif
{

    // Configure Oscillator to operate the device at 60Mhz
    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
    // Fosc= 7.37*65/(2*2)=120Mhz for 7.37 input clock
    PLLFBD = 63;                                // M=65
    CLKDIVbits.PLLPOST = 0;                     // N1=2
    CLKDIVbits.PLLPRE = 0;                      // N2=2
    OSCTUN = 0;                                 // Tune FRC oscillator, if FRC is used
    
        //// Disable Watch Dog Timer
    RCONbits.SWDTEN = 0;

    //
        //// Clock switch to incorporate PLL
    __builtin_write_OSCCONH( 0x01 );            // Initiate Clock Switch to

    // FRC with PLL (NOSC=0b001)
    __builtin_write_OSCCONL( OSCCON || 0x01 );  // Start clock switching
    while( OSCCONbits.COSC != 0b001 );

    // Wait for Clock switch to occur
    //
        //// Wait for PLL to lock
    while( OSCCONbits.LOCK != 1 )
    { };

    // Continuous Toggle Init
    CtglPinInit();

    // Tune FRC Osc
    TuneFrcOsc( tuneValue );
#ifdef TEST_MODE
    // Check the Tune value and make sure that it's written into the Osicllator tuning register.
    tValue = (uint16_t)OSCTUN;
    if (tValue == tuneValue)
        return 0;
    else
        return 1;
#else

    while( 1 )
    {
        // Toggle Freq is 1/4 of system clock frequency
        CtglPin();
    }
#endif
}

/*******************************************************************************
 End of File
 */

/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This file is used to call adc,timer,dma functions and FIR API

  Description:
    The main.c includes the header files that have the ADC, DMA, Timer and FIR API declarations
    and is used to call these functions.
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

#include<xc.h>

#include <dsp.h>
#include "adc1drv.h"
#include "fir.h"

#if __XC16_VERSION < 1011
#warning "Please upgrade to XC16 v1.11 or newer."
#endif

// *****************************************************************************
// *****************************************************************************
// Section: Macros for Configuration Fuse Registers:
// *****************************************************************************
// *****************************************************************************

//Macros for Configuration Fuse Registers:
//Invoke macros to set up  device configuration fuse registers.
//The fuses will select the oscillator source, power-up timers, watch-dog
//timers etc. The macros are defined within the device
//header files. The configuration fuse registers reside in Flash memory.

// DSPIC33EP512MU810 Configuration Bit Settings

// FICD
#pragma config ICS = PGD1               // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config JTAGEN = OFF             // JTAG Enable bit (JTAG is disabled)

// FPOR

#pragma config ALTI2C1 = OFF            // Alternate I2C1 pins (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2 = OFF            // Alternate I2C2 pins (I2C2 mapped to SDA2/SCL2 pins)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler bits (1:32,768)
#pragma config WDTPRE = PR128           // Watchdog Timer Prescaler bit (1:128)
#pragma config PLLKEN = ON              // PLL Lock Enable bit (Clock switch to PLL source will wait until the PLL lock signal is valid.)
#pragma config WINDIS = OFF             // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable bit (Watchdog timer enabled/disabled by user software)

// FOSC
#pragma config POSCMD = XT              // Primary Oscillator Mode Select bits (XT Crystal Oscillator Mode)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config IOL1WAY = OFF            // Peripheral pin select configuration (Allow multiple reconfigurations)
#pragma config FCKSM = CSECMD           // Clock Switching Mode bits (Clock switching is enabled,Fail-safe Clock Monitor is disabled)

// FOSCSEL
#pragma config FNOSC = FRC              // Oscillator Source Selection (Internal Fast RC (FRC))
#pragma config IESO = OFF               // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FGS
#pragma config GWRP = OFF               // General Segment Write-Protect bit (General Segment may be written)


// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************
 extern fractional bufferA[NUMSAMP];   // Ping pong buffer A
 extern fractional bufferB[NUMSAMP];   // Ping pong buffer B
 extern fractional outputSignal[NUMSAMP]; // Output buffer for the FIR filter output
 extern unsigned int dmaBuffer;    // DMA flag
 extern FIRStruct firfilter;
/******************************************************************************
 * Function:       void InitAdc1(void)
 *
 * PreCondition:   None
 *
 * Input:          None
 *
 * Output:         None
 *
 * Side Effects:   None
 *
 * Overview:       Sets CPU clock and initializes FIR filter, ADC  and timer module.
 *****************************************************************************/

#ifdef TEST_MODE
unsigned char test_flag;
#endif
#ifdef TEST_MODE
int ce_main(void)
#else
int main(void)
#endif
{
#ifdef TEST_MODE
test_flag=0;
#endif
//#ifdef TEST_MODE
//// Testing Reference Value consideration: Keep the POT(arrow mark of POT to extreme right/maximum)
//// on expl16 board for testing
//// Here we are checking for a range of samples to be correct
//
//
//    if((outputSignal[0] >= (-19700)) &&
//       (outputSignal[0] <= (-19600))
//      )
//        test_flag=1;
//#endif

 // Configure FRC to operate the device at 60MIPS
 // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
 // Fosc= 7.37M*65/(2*2)=119.7625Mhz for 7.37MHz input clock
 PLLFBD=58;         // M=65
 CLKDIVbits.PLLPOST=0;      // N1=2
 CLKDIVbits.PLLPRE=0;      // N2=2
 OSCTUN=0;         // Tune FRC oscillator, if FRC is used

 // Disable Watch Dog Timer
 RCONbits.SWDTEN=0;

 // Clock switch to incorporate PLL
 __builtin_write_OSCCONH(0x01);    // Initiate Clock Switch to
        // FRC with PLL (NOSC=0b001)
 __builtin_write_OSCCONL(OSCCON || 0x01);    // Start clock switching

 while (OSCCONbits.COSC != 0b001);   // Wait for Clock switch to occur

 while(OSCCONbits.LOCK!=1) {};    // Wait for PLL to lock

 // FIR Filter structure initialization
  FIRStructInit(&firfilter,NY,coeffecients,0xFF00,(fractional*)z);
 FIRDelayInit(&firfilter);

 // Peripheral Initialisation
 InitAdc1();// Initialize the A/D converter to convert Channel 5
 InitDma0();// Initialise the DMA controller to buffer ADC data in conversion order
 InitTmr3();// Initialise the Timer to generate sampling event to ADC @ 250Khz rate

 TRISAbits.TRISA7 = 0;

#ifdef TEST_MODE
    while(1)
    {
        if(test_flag ==1)
        return 0;
    }
#else

    while (1) //Loop Endlessly - Execution is interrupt driven
    { //from this point on.

    }

    return 0;
#endif


}

/*******************************************************************************
 End of File
*/
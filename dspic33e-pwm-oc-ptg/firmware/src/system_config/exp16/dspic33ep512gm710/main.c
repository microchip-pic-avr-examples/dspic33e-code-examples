/*******************************************************************************
  Main source file of Code example to generate Phase shifted waveforms using       
  Output Compare and Peripheral trigger Generator modules

  Company:
    Microchip Technology Inc.

  File Name:
    main.c
    
 Summary:
    This file contains following routines
        int  main(void)
        void Init_Timer2(void);
        void Init_PPS(void);
        void Init_OC1(void);
        void Init_OC2(void);
        
  Description:
        This is main file the code example to generate phase shifted 
        waveforms using  output compare module in association with
        Peripheral Trigger Generator. User can modify the Period,
        Phase shift and Duty cycle as desired.Pulse period can be
        modified by writting to PR2 registe.Duty cycle can be modified
        by writing to respective OCxR registers Phase shift may be
        modified by initilising PTGT0LIM with suitable value.
        In the example code phase shift is set as half of the PWM period.
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
#include "ptg_definition.h"

#if __XC16_VERSION < 1011
#warning "Please upgrade to XC16 v1.11 or newer."
#endif

// *****************************************************************************
// *****************************************************************************
// Section: Macros for Configuration Fuse Registers:
// *****************************************************************************
// *****************************************************************************
// DSPIC33EP256GM710 Configuration Bit Settings
// FICD
#pragma config ICS = PGD1           // ICD Communication Channel Select bits (Communicate on PGEC2 and PGED2)
#pragma config JTAGEN = OFF         // JTAG Enable bit (JTAG is disabled)

// FPOR
#pragma config BOREN = ON           //  (BOR is enabled)
#pragma config ALTI2C1 = OFF        // Alternate I2C1 pins (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2 = OFF        // Alternate I2C2 pins (I2C2 mapped to SDA2/SCL2 pins)
#pragma config WDTWIN = WIN25       // Watchdog Window Select bits (WDT Window is 25% of WDT period)

// FWDT
#pragma config WDTPOST = PS32768    // Watchdog Timer Postscaler bits (1:32,768)
#pragma config WDTPRE = PR128       // Watchdog Timer Prescaler bit (1:128)
#pragma config PLLKEN = ON          // PLL Lock Enable bit (Clock switch to PLL source will wait until the PLL lock signal is valid.)
#pragma config WINDIS = OFF         // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF         // Watchdog Timer Enable bit (Watchdog timer enabled/disabled by user software)

// FOSC
#pragma config POSCMD = XT          // Primary Oscillator Mode Select bits (XT Crystal Oscillator Mode)
#pragma config OSCIOFNC = OFF       // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config IOL1WAY = ON         // Peripheral pin select configuration (Allow only one reconfiguration)
#pragma config FCKSM = CSECMD       // Clock Switching Mode bits (Clock switching is enabled,Fail-safe Clock Monitor is disabled)

// FOSCSEL
#pragma config FNOSC = FRC          // Oscillator Source Selection (Internal Fast RC (FRC))
#pragma config PWMLOCK = OFF        // PWM Lock Enable bit (PWM registers may be written without key sequence)
#pragma config IESO = OFF           // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FGS
#pragma config GWRP = OFF           // General Segment Write-Protect bit (General Segment may be written)
#pragma config GCP = OFF            // General Segment Code-Protect bit (General Segment Code protect is Disabled)

// Functions prototypes
void Init_Timer2 ( void );
void    Init_PPS( void );
void    Init_OC1( void );
void    Init_OC2( void );

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
 ****************************************************************************/
#ifdef TEST_MODE
int ce_main(void)
#else
int main(void)
#endif
{
    // Configure Oscillator to operate the device at 40Mhz
    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
    // Fosc= 8M*70/(2*2)=1400Mhz for 8M input clock
    PLLFBD = 68;                        // M=70
    CLKDIVbits.PLLPOST = 0;             // N1=2
    CLKDIVbits.PLLPRE = 0;              // N2=2
    OSCTUN = 0;                         // Tune FRC oscillator, if FRC is used

    // Disable Watch Dog Timer
    RCONbits.SWDTEN = 0;

    // clock switch to incorporate PLL
    __builtin_write_OSCCONH( 0x03 );    // Initiate Clock Switch to Primary

    // Oscillator with PLL (NOSC=0b011)
    __builtin_write_OSCCONL( 0x01 );    // Start clock switching
    while( OSCCONbits.COSC != 0b011 );

    // Wait for Clock switch to occur
    // Wait for PLL to lock
    while( OSCCONbits.LOCK != 1 )
    { };

    Init_Timer2();          // Initilise Timer 2 with Period
    Init_PPS();             // Map OC1,OC2 Ouputs to RP54,RP55 repectively
    Init_OC1();             // Initialise OC1 module
    Init_OC2();             // Initialise OC2 module
    Init_PTG();             // Initialise PTG
    Write_PTG_Sequence();   // Defines PTG Sequencer steps
    PTGCSTbits.PTGEN = 1;   // Enable the PTG Module
    PTGCSTbits.PTGSTRT = 1; // Start the Sequencer
    T2CONbits.TON = 1;      // Start the timer,the clock source for Output Compare Modules

#ifdef TEST_MODE
// In test mode the PTG0 interrupt is generated during step4. If this gets generated
// it means to say that PTG sequence is executed properly.
    if (_PTG0IF ==1 )
    {
   
        return 0;
    }
  
#else
    while( 1 )
    { }

#endif
}


/******************************************************************************
 * Function:        void Init_Timer2(void)
 *
 * PreCondition:    None.
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is used to initialize the Timer 2.
 *****************************************************************************/
void Init_Timer2( void )
{
    T2CON = 0X0000;
    TMR2 = 0X0000;
    PR2 = 2800; // Specify the Pulse period here,is configured for 40 us
}

/******************************************************************************
 * Function:        void Init_OC1(void)
 *
 * PreCondition:    None.
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is used to initialize Output Compare 1
 *****************************************************************************/
void Init_OC1( void )
{
    OC1R = 350;                 // Specify the pulse width of the OC1 Output ,initialised for 5us width
    OC1RS = 0x000;              // Initialize the secondary compare register// Initialize Output Compare Module
    OC1CON1 = 0x0;              // Clear all control bits
    OC1CON2 = 0x0;              // Clear all control bits
    OC1CON1bits.OCTSEL = 0x0;   // Select peripheral clock as clock source
    OC1CON2bits.SYNCSEL = 0xC;  // Select Timer2 as sync source
    OC1CON1bits.OCM = 0x6;      // Double compare continuous pulse mode
}

/******************************************************************************
 * Function:        void Init_OC2(void)
 *
 * PreCondition:    None.
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is used to initialize Output Compare 2
 *****************************************************************************/
void Init_OC2( void )
{
    OC2R = 700;                 // Specify the pulse width of OC2 Output ,initialised for 10us width
    OC2RS = 0x000;              // Initialize the secondary compare register// Initialize Output Compare Module
    OC2CON1 = 0x0;              // Clear all control bits
    OC2CON2 = 0x0;              // Clear all control bits
    OC2CON1bits.OCTSEL = 0x7;   // Select peripheral clock as clock source
    OC2CON2bits.SYNCSEL = 0xA;  // Select PTG Output as sync source
    OC2CON1bits.OCM = 0x6;      // Double compare continuous pulse mode
}

/******************************************************************************
 * Function:        void Init_PPS(void)
 *
 * PreCondition:    None.
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function is used to initialize the Peripheral Pin Select.
 *****************************************************************************/
void Init_PPS( void )
{
    __builtin_write_OSCCONL( OSCCON & (~(1 << 6)) );

    _RP54R = 0X10;  // RP54 pin is mapped as OC1 out
    _RP55R = 0X11;  // RP55 pin is mapped as OC2 out
    __builtin_write_OSCCONL( OSCCON | (1 << 6) );
}

/*******************************************************************************
 End of File
 */

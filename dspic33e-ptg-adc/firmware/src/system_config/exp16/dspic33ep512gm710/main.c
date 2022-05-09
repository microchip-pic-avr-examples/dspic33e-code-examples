/*******************************************************************************
   This code example generates multiple ADC triggers in synchronisation with
    PWM time base

  Company:
    Microchip Technology Inc.

  File Name:
    main.c
    
  Summary:
    This code example to generate multiple adc triggers using PTG
    
  Description:
    This code example to generate multiple adc triggers using Peripheral Trigger Generator module 
    in synchroisation with High Speed PWM Generator.From the start of PWM ON time waits for 15 us, 
    and then generate ADC triggers to sample analog signal 6 times.After generating every ADC trigger,
    sequencer is made to  wait for ADC conversion to complete ,before generating next trigger.
    At the end generates a PTG Interrupt to process the acquired signal samples.

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
// DSPIC33EP512GM710 Configuration Bit Settings
// FICD
#pragma config ICS = PGD1           // ICD Communication Channel Select bits (Communicate on PGEC2 and PGED2)
#pragma config JTAGEN = OFF         // JTAG Enable bit (JTAG is disabled)

// FPOR
#pragma config BOREN = ON           // Brown-out Reset (BOR) Detection Enable bit (BOR is enabled)
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
extern void Init_ADC ( void );
extern void Init_PWMSync( void );
void        Init_IO_Ports( void );

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
unsigned int test_flag=0;
extern int API_Delay10xTCY(unsigned int);
int ce_main(void)
#else
int main(void)
#endif
{
#ifdef TEST_MODE
    test_flag=0;
#endif
    // Configure Oscillator to operate the device at 40Mhz
    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
    // Fosc= 8M*40/(2*2)=80Mhz for 8M input clock
    PLLFBD = 38;                        // M=40
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

    Init_IO_Ports();
    Init_PWMSync();
    Init_ADC();
    Init_PTG();

    Write_PTG_Sequence();
    PTCONbits.SYNCOEN = 1;
    PTCONbits.PTEN = 1;
    PTGCSTbits.PTGEN = 1;               //Enable PTG
    PTGCSTbits.PTGSTRT = 1;             //Start the sequencer

#ifdef TEST_MODE

   // To match the latency after PTG Enabling and actual triggering of ADC interrupt the following delay is given.
    API_Delay10xTCY(156);
   if(test_flag ==1 )
        return 0;   // Trap interrupt is generated
    
#else
    while(1);
      { }

#endif
}

/******************************************************************************
 * Function:        void Init_IO_Ports(void)
 *
 * PreCondition:    None.
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function initilaises ports
 *****************************************************************************/
void Init_IO_Ports( void )
{
    TRISCbits.TRISC6 = 0;
    TRISCbits.TRISC7 = 0;
    PORTCbits.RC6 = 0;
    PORTCbits.RC7 = 0;
}

/*******************************************************************************
 End of File
 */

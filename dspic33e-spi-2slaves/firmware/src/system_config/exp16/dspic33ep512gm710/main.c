/*******************************************************************************
  ce438 main source file

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This code example shows using the SPI module in conjunction with 2 GPIO pins to communicate with 2 different
    slavedevices. The concept is that the GPIO are the slave selects for the individual devices, but the SCL, SDI
    and SDO are shared between the two processors.
 
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

#if __XC16_VERSION < 1011
#warning "Please upgrade to XC16 v1.11 or newer."
#endif

//-----------------------------------------------------------------------------
//Macros for Configuration Fuse Registers:
//-----------------------------------------------------------------------------
//Invoke macros to set up  device configuration fuse registers.
//The fuses will select the oscillator source, power-up timers, watch-dog
//timers etc. The macros are defined within the device
//header files. The configuration fuse registers reside in Flash memory.
// DSPIC33EP512GM710 Configuration Bit Settings
// FICD
#pragma config ICS = PGD1                       // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
#pragma config JTAGEN = OFF                     // JTAG Enable bit (JTAG is disabled)

// FPOR
#pragma config ALTI2C1 = OFF                    // Alternate I2C1 pins (I2C1 mapped to SDA1/SCL1 pins)
#pragma config ALTI2C2 = OFF                    // Alternate I2C2 pins (I2C2 mapped to SDA2/SCL2 pins)
#pragma config WDTWIN = WIN25                   // Watchdog Window Select bits (WDT Window is 25% of WDT period)

// FWDT
#pragma config WDTPOST = PS32768                // Watchdog Timer Postscaler bits (1:32,768)
#pragma config WDTPRE = PR128                   // Watchdog Timer Prescaler bit (1:128)
#pragma config PLLKEN = ON                      // PLL Lock Enable bit (Clock switch to PLL source will wait until the PLL lock signal is valid.)
#pragma config WINDIS = OFF                     // Watchdog Timer Window Enable bit (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF                     // Watchdog Timer Enable bit (Watchdog timer enabled/disabled by user software)

// FOSC
#pragma config POSCMD = XT                      // Primary Oscillator Mode Select bits (XT Crystal Oscillator Mode)
#pragma config OSCIOFNC = OFF                   // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config IOL1WAY = OFF                    // Peripheral pin select configuration (Allow multiple reconfigurations)
#pragma config FCKSM = CSECMD                   // Clock Switching Mode bits (Clock switching is enabled,Fail-safe Clock Monitor is disabled)

// FOSCSEL
#pragma config FNOSC = FRC                      // Oscillator Source Selection (Internal Fast RC (FRC))
#pragma config IESO = OFF                       // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FGS
#pragma config GWRP = OFF                       // General Segment Write-Protect bit (General Segment may be written)
#pragma config GCP = OFF                        // General Segment Code-Protect bit (General Segment Code protect is Disabled)

/******************************************************************************
 * Function:        void Write_SPI1(int16_t command)
 *
 * PreCondition:    SPI module should be initialized   
 *
 * Input:           command - data to be sent over SPI
 *                  
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function transfers data over SPI
 *****************************************************************************/
void Write_SPI1 ( int16_t command )
{
    int16_t temp;

    PORTGbits.RG15 = 0;                         // lower the slave select line
    temp = SPI1BUF;                             // dummy read of the SPI1BUF register to clear the SPIRBF flag
    SPI1BUF = command;                          // write the data out to the SPI peripheral
    while( !SPI1STATbits.SPITBF )               // wait for the data to be sent out
        ;
    PORTGbits.RG15 = 1;                         // raise the slave select line
}

/******************************************************************************
 * Function:        void Write_SPI2(int16_t command)
 *
 * PreCondition:    SPI module should be initialized   
 *
 * Input:           command - data to be sent over SPI
 *                  
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function transfers data over SPI
 *****************************************************************************/
void Write_SPI2 ( int16_t command )
{
    int16_t temp;

    PORTGbits.RG13 = 0;                         // lower the slave select line
    temp = SPI1BUF;                             // dummy read of the SPI1BUF register to clear the SPIRBF flag
    SPI1BUF = command;                          // write the data out to the SPI peripheral
    while( !SPI1STATbits.SPIRBF )               // wait for the data to be sent out
        ;
    PORTGbits.RG13 = 1;                         // raise the slave select line
}

/******************************************************************************
 * Function:        void Delay(void))
 *
 * PreCondition:    None   
 *
 * Input:           None
 *                  
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function provides sofware delay
 *****************************************************************************/
void Delay ( void )
{
    int16_t temp;
    for( temp = 0; temp < 255; temp++ )
        ;
}

/******************************************************************************
 * Function:        void Init_SPI(void)
 *
 * PreCondition:    None   
 *
 * Input:           None
 *                  
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:        This function configers SPI module
 *****************************************************************************/
void Init_SPI ( void )
{
    // setup the SPI peripheral
    SPI1STAT = 0x0;                             // disable the SPI module (just in case)
    SPI1CON1 = 0x0161;                          // FRAMEN = 0, SPIFSD = 0, DISSDO = 0, MODE16 = 0; SMP = 0; CKP = 1; CKE = 1; SSEN = 0; MSTEN = 1; SPRE = 0b000, PPRE = 0b01
    SPI1CON1bits.CKE = 0x01;
    SPI1CON1bits.CKP = 0x00;
    SPI1STAT = 0x8000;                          // enable the SPI module
}

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
 *****************************************************************************/
int main ( void )
{
    int16_t i;

    // Configure Oscillator to operate the device at 60Mhz
    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
    // Fosc= 8M*60/(2*2)=120Mhz for 8M input clock
    PLLFBD = 58;                                // M=60
    CLKDIVbits.PLLPOST = 0;                     // N1=2
    CLKDIVbits.PLLPRE = 0;                      // N2=2
    OSCTUN = 0;                                 // Tune FRC oscillator, if FRC is used

    // Disable Watch Dog Timer
    RCONbits.SWDTEN = 0;

    // Clock switching to incorporate PLL
    __builtin_write_OSCCONH( 0x03 );            // Initiate Clock Switch to Primary

    // Oscillator(XT) with PLL (NOSC=0b011)
    __builtin_write_OSCCONL( OSCCON || 0x01 );  // Start clock switching
    while( OSCCONbits.COSC != 0b011 );

    // Wait for Clock switch to occur
    // Wait for PLL to lock
    while( OSCCONbits.LOCK != 1 )
    { };

    ANSELAbits.ANSA4 = 0;                       //make port pins as digital
    ANSELAbits.ANSA9 = 0;
    ANSELCbits.ANSC3 = 0;

    TRISAbits.TRISA4 = 0;                       //port pis as input/output
    TRISAbits.TRISA9 = 1;
    TRISCbits.TRISC3 = 0;

    while( 1 )
    {
        //demonstrate the SPI peripheral in Master Mode CKE = 0, CKP = 0
        Init_SPI();

        for( i = 0; i < 255; i++ )
        {
            Write_SPI1( i );
            Delay();
        }

        for( i = 0; i < 255; i++ )
        {
            Write_SPI2( i );
            Delay();
        }

        for( i = 0; i < 255; i++ )
        {
            Write_SPI1( i );
            Delay();
            Write_SPI2( i );
            Delay();
        }
    }
}

/******************************************************************************/
/*******************************************************************************
 End of File
*/

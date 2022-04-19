/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This file is used to call the RTDM API

  Description:
    The main.c includes the header files that have the RTDM API declarations
    and is used to call the RTDM_Start, RTDM_ProcessMsgs and RTDM_Close functions.
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
#include "rtdm.h"

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
// 'C' source line config statements
// FICD
#pragma config ICS = PGD1           // ICD Communication Channel Select bits (Communicate on PGEC1 and PGED1)
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
#pragma config POSCMD = NONE        // Primary Oscillator Mode Select bits (Primary Oscillator disabled)
#pragma config OSCIOFNC = OFF       // OSC2 Pin Function bit (OSC2 is clock output)
#pragma config IOL1WAY = OFF        // Peripheral pin select configuration (Allow multiple reconfigurations)
#pragma config FCKSM = CSECMD       // Clock Switching Mode bits (Clock switching is enabled,Fail-safe Clock Monitor is disabled)

// FOSCSEL
#pragma config FNOSC = FRC          // Oscillator Source Selection (Internal Fast RC (FRC))
#pragma config PWMLOCK = ON         // PWM Lock Enable bit (Certain PWM registers may only be written after key sequence)
#pragma config IESO = OFF           // Two-speed Oscillator Start-up Enable bit (Start up with user-selected oscillator source)

// FGS
#pragma config GWRP = OFF           // General Segment Write-Protect bit (General Segment may be written)
#pragma config GCP = OFF            // General Segment Code-Protect bit (General Segment Code protect is Disabled)

// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************
#define FCY                             55275000    //Instruction Cycle frequency
#define AMOUNT_OF_DATA_TO_BE_PLOTTED    256         //number of sanpshot samples expressed in 16-bit words since the Buffer is 16bit wide
unsigned int myVariable, frequency, amplitude;      //Varaible to be recored and its paramters to be modified using the DMCI sliders
unsigned int    snapShotBufferPlottedInDMCI[AMOUNT_OF_DATA_TO_BE_PLOTTED];  //buffer where the data is recorded
unsigned int    *pointerToSnapShotBuffer = &snapShotBufferPlottedInDMCI[0]; //Tail pointer required to plot circular buffers in DMCI
unsigned int    *pointerToSnapShotBufferUpperLimit = &snapShotBufferPlottedInDMCI[0] + AMOUNT_OF_DATA_TO_BE_PLOTTED - 1;

//Buffer Upper limit
struct
{
    unsigned    TrigggerSnapshot : 1;   //Tirgger variable to start recording the values of myVariable
    unsigned    LED_D3 : 1;             //Definition of Exp 16 LED D3-D6
    unsigned    LED_D4 : 1;
    unsigned    LED_D5 : 1;
    unsigned    LED_D6 : 1;
    unsigned    unused : 11;
} myFlags;

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
 * Overview:        Set CPU operating frequency and UART module.
 *****************************************************************************/
int main( void )
{
    // Configure Oscillator to operate the device at 60Mhz
    // Fosc= Fin*M/(N1*N2), Fcy=Fosc/2
    // Fosc= 7.37*60/(2*2)=110.55MHz for 7.37 MHz clock
    PLLFBD = 58;                                // M=60
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

    //Assigning the TX and RX pins to ports RP100 & RP101 to the dsPIC33EP512MU810
    RPINR19 = 0;
    RPOR1 = 0;
    RPOR1bits.RP36R = 3;                        //RB4 as U2TX
    RPINR19bits.U2RXR = 24;                     //RA8 as U2RX
    ANSELG = 0x0000;
    LATG = 0x0000;                  //LED initial state is OFF
    TRISG = 0x00;                   // Config LED's
    myFlags.TrigggerSnapshot = 0;   //Snapshot trigger intial state is OFF
    frequency = 1000;               //Initial frequency of myVariable saw-tooth wave
    amplitude = 65535;              //Initial amplitude of myVariable saw-tooth wave

    // NOTE: DMCI is configured to auto-adjust the plot min and max values,
    //To notice amplitude changes on the DMCI plot, right click on the DMCI plot
    // goto to Extended Functions Menu > Customization > Axis, set the desired
    //min an max values for the Y AXIS
    RTDM_Start();                   //RTDM start function

    // Overview:
    // Here is where the RTDM code initilizes the UART to be used to
    // exchange data wiht the host PC
    // Note:
    // Some processors may have 2 UART modules, that is why it is required to
    // specify wich UART module is going to be used by RTDM
    for( ;; )
    {
        LATGbits.LATG10 = ~myFlags.LED_D3;  //Passing data from the boolean variables controlled
        LATGbits.LATG11 = myFlags.LED_D4;   // in DMCI to the PORT A data registers
        LATGbits.LATG2 = ~myFlags.LED_D5;
        LATGbits.LATG3 = myFlags.LED_D6;

        RTDM_ProcessMsgs();                 // Overview:

        // Here is where the RTDM code process the message received and then
        // executes the required task. These tasks are reading an specified memory
        // location, writing an specified memory location, receive a communication
        // link sanity check command, or being asked for the size of the bufffers.
        myVariable += frequency;            //Here is were the variable is updated by the Application SW
        if( myVariable > amplitude )
        {                   //IF condition to check the max value of my variable
            myVariable = 0; // This section controls the max amplitude
        }

        //If the TriggerSnapShot is ON then the values of the variable are recorded on the SnapShot Buffer,
        // if the TriggerSnapShot is OFF the values are not recorded  this is a lock condition in order to prevent that the
        // data recorded on the snapshot buffer are being updated while the target device is sending/receiving information
        //to DMCI. If the data is  corrupted while a TX is in progress, DMCI will flush the received data and it
        //will display an error message. The values won't be displayed on the graph
        if( myFlags.TrigggerSnapshot )
        {
            *pointerToSnapShotBuffer++ = myVariable;    //Recordinf values
            if( pointerToSnapShotBuffer > pointerToSnapShotBufferUpperLimit )
            {       //SnapShot Buffer is a circular buffer
                pointerToSnapShotBuffer = snapShotBufferPlottedInDMCI;

                // myFlags.TrigggerSnapshot = 0; //Turning OFF the snapshot mode indicating that new data is available to be sent
                //DMCI user has to push the RECEIVE button in order to retrive data from the target
                //device
            }
        }
    }               /*END OF THE INFINITE LOOP FOR*/

    RTDM_Close();   // Overview:

    // Here is where the RTDM code closes the UART used to
    // exchange data wiht the host PC
    return ( 0 );
}

/*******************************************************************************
 End of File
*/

/*******************************************************************************
  Header file for Code example to generate Phase shifted waveforms using       
  Output Compare and Peripheral trigger Generator modules.

  Company:
    Microchip Technology Inc.

  File Name:
    ptg_definition.h

  Summary:
    Header file describing Peripheral Trigger Module Commands and Options.

  Description:
    This header defines the various Step Commands and its options.
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
#ifndef _PTG_DEFINITIONS_H                  // Guards against multiple inclusion
    #define _PTG_DEFINITIONE_H

    #ifdef __cplusplus                      // Provide C++ Compatability
extern "C"
{
        #endif

    // *****************************************************************************
    // *****************************************************************************
    // Section: Interface Routines
    // *****************************************************************************
    // *****************************************************************************
    //Functions
    extern void Write_PTG_Sequence( void );
    extern void Init_PTG( void );

    // *****************************************************************************
    // *****************************************************************************
    // Section: Helper Macros
    // *****************************************************************************
    // *****************************************************************************
    /* Command Definitions*/
        #define PTGCTRL     ( 0x0 << 4 )    //PTGCTRL Command
        #define PTGADD      ( 0x1 << 4 )    //PTGADD Command
        #define PTGCOPY     ( 0x1 << 4 )    //PTGCOPY Command
        #define PTGSTRB     ( 0x2 << 4 )    //PTGSTRB Command
        #define PTGWHI      ( 0x4 << 4 )    //PTGWHI Command
        #define PTGWLO      ( 0x5 << 4 )    //PTGWLO Command
        #define PTGIRQ      ( 0x7 << 4 )    //PTGIRQ Command
        #define PTGTRIG     ( 0x8 << 4 )    //PTGTRIG Command
        #define PTGJMP      ( 0xA << 4 )    //PTGJMP Command
        #define PTGJMPC0    ( 0xC << 4 )    //PTGJMPC0 Command
        #define PTGJMPC1    ( 0xF << 4 )    //PTGJMPC1 Command

    /*PTGCTRL Options Definitions */
        #define CTRLOPT2    ( 0x2 )         //Disable Step Delay Timer (PTGSD)
        #define CTRLOPT6    ( 0x6 )         //Enable Step Delay Timer (PTGSD)
        #define CTRLOPT8    ( 0x8 )         //Start and wait for the PTG Timer 0 to match Timer 0 Limit Register
        #define CTRLOPT9    ( 0x9 )         //Start and wait for the PTG Timer 1 to match Timer 1 Limit Register
        #define CTRLOPT11   ( 0xB )         //Wait for software trigger bit transition from low to high before continuing (PTGSWT = 0 to 1)
        #define CTRLOPT12   ( 0xC )         //Copy contents of the Counter 0 register to the AD1CHS0 register
        #define CTRLOPT13   ( 0xD )         //Copy contents of the Counter 1 register to the AD1CHS0 register
        #define CTRLOPT14   ( 0xE )         //Copy contents of the Literal 0 register to the AD1CHS0 register
        #define CTRLOPT15   ( 0xF )         //Generate triggers indicated in the Broadcast Trigger Enable Register

    /*PTGADD Options Definitions */
        #define ADDOPT0 ( 0x0 )             //Add contents of PTGADJ register to the Counter 0 Limit register (PTGC0LIM)
        #define ADDOPT1 ( 0x1 )             //Add contents of PTGADJ register to the Counter 1 Limit register (PTGC1LIM)
        #define ADDOPT2 ( 0x2 )             //Add contents of PTGADJ register to the Timer 0 Limit register (PTGT0LIM)
        #define ADDOPT3 ( 0x3 )             //Add contents of PTGADJ register to the Timer 1 Limit register (PTGT1LIM)
        #define ADDOPT4 ( 0x4 )             //Add contents of PTGADJ register to the Step Delay Limit register (PTGSDLIM)
        #define ADDOPT5 ( 0x5 )             //Add contents of PTGADJ register to the Literal 0 register (PTGL0)

    /*PTGCOPY Options Definitions */
        #define COPYOPT8    ( 0x8 )         //Copy contents of PTGHOLD register to the Counter 0 Limit register (PTGC0LIM)
        #define COPYOPT9    ( 0x9 )         //Copy contents of PTGHOLD register to the Counter 1 Limit register (PTGC1LIM)
        #define COPYOPT10   ( 0xA )         //Copy contents of PTGHOLD register to the Timer 0 Limit register (PTGT0LIM)
        #define COPYOPT11   ( 0xB )         //Copy contents of PTGHOLD register to the Timer 1 Limit register (PTGT1LIM)
        #define COPYOPT12   ( 0xC )         //Copy contents of PTGHOLD register to the Step Delay Limit register
        #define COPYOPT13   ( 0xD )         //Copy contents of PTGHOLD register to the Literal 0 register (PTGL0)

    /*PTGWHI/PTGWLO Options Definitions */
        #define PTGWOPT0    ( 0x0 )         //PWM Special Event Trigger(3)
        #define PTGWOPT1    ( 0x1 )         //PWM Master Timebase Synchronization Output(3)
        #define PTGWOPT2    ( 0x2 )         //PWM1 Interrupt(3)
        #define PTGWOPT3    ( 0x3 )         //PWM2 Interrupt(3)
        #define PTGWOPT4    ( 0x4 )         //PWM3 Interrupt(3)
        #define PTGWOPT5    ( 0x5 )         //Reserved
        #define PTGWOPT6    ( 0x6 )         //Reserved
        #define PTGWOPT7    ( 0x7 )         //OC1 Trigger Event
        #define PTGWOPT8    ( 0x8 )         //OC2 Trigger Event
        #define PTGWOPT9    ( 0x9 )         //IC1 Trigger Event
        #define PTGWOPT10   ( 0xA )         //CMP1 Trigger Event
        #define PTGWOPT11   ( 0xB )         //CMP2 Trigger Event
        #define PTGWOPT12   ( 0xC )         //CMP3 Trigger Event
        #define PTGWOPT13   ( 0xD )         //CMP4 Trigger Event
        #define PTGWOPT14   ( 0xE )         //ADC Conversion Done Interrupt
        #define PTGWOPT15   ( 0xF )         //INT2 External Interrupt

    /*PTGIRQ Options Definitions */
        #define IRQOPT0 ( 0x0 )             //Generate PTG interrupt 0
        #define IRQOPT1 ( 0x1 )             //Generate PTG interrupt 1
        #define IRQOPT2 ( 0x2 )             //Generate PTG interrupt 2
        #define IRQOPT3 ( 0x3 )             //Generate PTG interrupt 3

    /*PTGTRIG Options Definitions */
        #define TRIGOPT0    ( 0x0 )         //PTGO0
        #define TRIGOPT1    ( 0x1 )         //PTGO1
        #define TRIGOPT2    ( 0x2 )         //PTGO2
        #define TRIGOPT3    ( 0x3 )         //PTGO3
        #define TRIGOPT4    ( 0x4 )         //PTGO4
        #define TRIGOPT5    ( 0x5 )         //PTGO5
        #define TRIGOPT6    ( 0x6 )         //PTGO6
        #define TRIGOPT7    ( 0x7 )         //PTGO7
        #define TRIGOPT8    ( 0x8 )         //PTGO8
        #define TRIGOPT9    ( 0x9 )         //PTGO9
        #define TRIGOPT10   ( 0xA )         //PTGO10
        #define TRIGOPT11   ( 0xB )         //PTGO11
        #define TRIGOPT12   ( 0xC )         //PTGO12
        #define TRIGOPT13   ( 0xD )         //PTGO13
        #define TRIGOPT14   ( 0xE )         //PTGO14
        #define TRIGOPT15   ( 0xF )         //PTGO15
        #define TRIGOPT16   ( 0x10 )        //PTGO16
        #define TRIGOPT17   ( 0x11 )        //PTGO17
        #define TRIGOPT18   ( 0x12 )        //PTGO18
        #define TRIGOPT19   ( 0x13 )        //PTGO19
        #define TRIGOPT20   ( 0x14 )        //PTGO20
        #define TRIGOPT21   ( 0x15 )        //PTGO21
        #define TRIGOPT22   ( 0x16 )        //PTGO22
        #define TRIGOPT23   ( 0x17 )        //PTGO23
        #define TRIGOPT24   ( 0x18 )        //PTGO24
        #define TRIGOPT25   ( 0x19 )        //PTGO25
        #define TRIGOPT26   ( 0x1A )        //PTGO26
        #define TRIGOPT27   ( 0x1B )        //PTGO27
        #define TRIGOPT28   ( 0x1C )        //PTGO28
        #define TRIGOPT29   ( 0x1D )        //PTGO29
        #define TRIGOPT30   ( 0x1E )        //PTGO30
        #define TRIGOPT31   ( 0x1F )        //PTGO31
        #ifdef __cplusplus                  // Provide C++ Compatibility
}

    #endif
#endif // _PTG_DEFINITION_H

/*******************************************************************************
 End of File
*/

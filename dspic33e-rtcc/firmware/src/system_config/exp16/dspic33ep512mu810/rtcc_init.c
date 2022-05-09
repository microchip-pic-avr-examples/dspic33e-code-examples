/*******************************************************************************
 ce453 rtcc file.
 
 Company:
    Microchip Technology Inc.

  File Name:
    rtcc_init.c

  Summary:
    This file is used to initialize rtcc.

  Description:
    This code example aims to demonstrate the basic initialisation and operation
    of the Real Time Clock and Calender (RTCC) module.
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
#include "rtcc_init.h"


// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************
uint16_t    year, mondat, wkhr, minsec;

/******************************************************************************
 * Function:        void RTCCUnlock(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:       This function is a unlocksequence to write to write
 *                 protected register
 *****************************************************************************/

/* NOTE:  */
void RTCCUnlock( void ) //Unlocking sequence
{
    asm( "MOV #0x55,W0" );
    asm( "MOV W0,NVMKEY" ); // Write the 0x55 key
    asm( "MOV #0xAA,W1" );
    asm( "MOV W1,NVMKEY" ); // Write the 0xAA key
    RCFGCALbits.RTCWREN = 1;
}

/******************************************************************************
 * Function:        void RtccInit(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:       This function is used initialize RTCC module.
 *                 The alarm settings, time & date register and alarm values are
 *                 user selectable.The calibration bits can be programmed to
 *                 compensate the variation of the secondary crystal frequency
 *                 from the nominal value.Slower than nominal - add counts
 *                 Faster than nominal - subtract counts
 *****************************************************************************/
void RtccInit( void )
{
    RTCCUnlock();               //unlock Timer Registers
    RTCCOff();                  //disable the RTCC peripheral

    /* Configure the alarm settings*/
    ALCFGRPTbits.CHIME = 0;     //no rolloever of the repeat count
    ALCFGRPTbits.AMASK = 0;     //alarm mask configuration bits
    ALCFGRPTbits.ARPT = 0;      //alarm repeat counter value bits
    RCFGCALbits.RTCOE = 1;      //enable RTCC output
    ANSELBbits.ANSB3 = 0;       //RTCC pin pad conected to Alarm

    /* Load the initial values to the RTCC value registers*/
    RCFGCALbits.RTCPTR = 3;     //point to year register
    RTCVAL = 0x0010;            //year
    RTCVAL = 0x0311;            //Month & Day
    RTCVAL = 0x0414;            //WkDay & Hour
    RTCVAL = 0x3710;            //Min & Sec
    RCFGCALbits.CAL = 0x0000;   //No calibration
    ALCFGRPTbits.ALRMPTR = 2;   //Point to Month/Day register
    ALRMVAL = 0x0311;           //load month & day
    ALRMVAL = 0x0414;           //load weekday & hour
    ALRMVAL = 0x3715;           //load minute & seconds
    ALCFGRPTbits.ALRMEN = 1;    //enable the alarm
    ALCFGRPTbits.ARPT = 1;
    ALCFGRPTbits.AMASK = 6;

    RTCCOn();                   //enable RTCC peripheral
    RTCCLock();                 //lock the RTCC value registers

    /* Enable the RTCC interrupt*/
    _RTCIF = 0;                 //clear the RTCC interrupt flag
    _RTCIE = 1;                 //enable the RTCC interrupt
}

/******************************************************************************
 * Function:        void RtccRead(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:       Function to read the time and date registers
 *****************************************************************************/
void RtccRead( void )
{
    while( RCFGCALbits.RTCSYNC == 1 );

    //wait for RTCSYNC bit to become ‘0’
    RCFGCALbits.RTCPTR = 3; //points to the year register
    year = RTCVAL;
    mondat = RTCVAL;
    wkhr = RTCVAL;
    minsec = RTCVAL;        // read the device value
#ifdef TEST_MODE
    if((year == 0x0010) && (mondat == 0x0311))
        test_flag=1;
#endif

}

/******************************************************************************
 * Function:        void __attribute__((interrupt)) _RTCCInterrupt(void)
 *
 * PreCondition:    None
 *
 * Input:           None
 *
 * Output:          None
 *
 * Side Effects:    None
 *
 * Overview:       RTCCInterrupt() is the RTCC interrupt service routine (ISR).
 *                 The ISR name is chosen from the device linker script.
 *****************************************************************************/
void __attribute__ ( (interrupt) ) _RTCCInterrupt( void )
{
    RtccRead();             // read the date and time registers.
    _RTCIF = 0;             // clear the INT1 interrupt flag or else the CPU will keep
    //vectoring back to the ISR
}

/*******************************************************************************
 End of File
*/

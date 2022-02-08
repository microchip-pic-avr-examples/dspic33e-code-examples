/*******************************************************************************
  ADC driver functions source file

  Company:
    Microchip Technology Inc.

  File Name:
    adc1drv.c

  Summary:
    ADC, DMA, Timer function routines

  Description:
    This file has the initialization routines for the ADC and DMA modules. 
    The ADC is configured for auto sampling and produces results, 
    which are in the signed fractional format and puts the result in the DMA/x-memory
    buffers. These buffers are then used as inputs to perform filtering of the input
    analog signal. 
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
#include "dsp.h"
#include "adc1drv.h"
#include "tglpin.h"

// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************
unsigned int dmaBuffer = 0;
fractional bufferA[NUMSAMP]  __attribute__((space(xmemory)));
fractional bufferB[NUMSAMP]  __attribute__((space(xmemory)));
// FIR Output Buffer
//fractional outputSignal[NUMSAMP] __attribute__ ((space(xmemory),far,aligned(512)));
fractional outputSignal[NUMSAMP];
FIRStruct firfilter;

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
 * Overview:       initAdc1() is used to configure A/D to convert channel 10
 *                 on Timer event.It generates event to DMA on every
 *                 sample/convert sequence.
 *****************************************************************************/
void InitAdc1(void)
{
  AD1CON1bits.FORM   = 3;  // Data Output Format: Signed Fraction (Q15 format)
  AD1CON1bits.SSRC   = 2;  // Sample Clock Source: GP Timer starts conversion
  AD1CON1bits.ASAM   = 1;  // ADC Sample Control: Sampling begins immediately after conversion
  AD1CON1bits.AD12B  = 0;  // 10-bit ADC operation

  AD1CON2bits.CHPS  = 0;  // Converts CH0
     
  AD1CON3bits.ADRC = 0;  // ADC Clock is derived from Systems Clock
  AD1CON3bits.ADCS = 3;  // ADC Conversion Clock Tad=Tcy*(ADCS+1)= (1/40M)*4 = 100ns
         // ADC Conversion Time for 10-bit Tc=12*Tad = 1.2us 

     
  AD1CON1bits.ADDMABM = 1;  // DMA buffers are built in conversion order mode
  AD1CON2bits.SMPI    = 0; // SMPI must be 0
  AD1CON4bits.ADDMAEN = 1;  // all results written in ADC1BUF0

  //AD1CHS0: A/D Input Select Register
  AD1CHS0bits.CH0SA= 20;  // MUXA +ve input selection (AN20) for CH0
  AD1CHS0bits.CH0NA=0;  // MUXA -ve input selection (Vref-) for CH0
      
  IFS0bits.AD1IF = 0;   // Clear the A/D interrupt flag bit
  IEC0bits.AD1IE = 0;   // Do Not Enable A/D interrupt 
  AD1CON1bits.ADON = 1;  // Turn on the A/D converter 

  CtglPinInit();    // Toggle RA6 Init

}
/******************************************************************************
 * Function:       void InitTmr3(void)
 *
 * PreCondition:   None
 *
 * Input:          None
 *
 * Output:         None
 *
 * Side Effects:   None
 *
 * Overview:       Timer 3 is setup to time-out every 4 microseconds(250Khz Rate).
 *                 As a result, the module will stop sampling and trigger a
 *                 conversion on every Timer3 time-out, i.e., Ts=4us.
 *                 At that time, the conversion process starts and completes
 *                 Tc=12*Tad periods later.When the conversion completes,
 *                 the module starts sampling again. However, since Timer3
 *                 is already on and counting, about (Ts-Tc)us later, Timer3
 *                 will expire again and trigger next conversion.
 *****************************************************************************/
void InitTmr3(void)
{
   TMR3 = 0x0000;
   PR3 = SAMPPRD;
   IFS0bits.T3IF = 0;
   IEC0bits.T3IE = 0;

   //Start Timer 3
   T3CONbits.TON = 1;
}
/******************************************************************************
 * Function:       void InitDma0(void)
 *
 * PreCondition:   None
 *
 * Input:          None
 *
 * Output:         None
 *
 * Side Effects:   None
 *
 * Overview:       DMA0 configuration
 *                 Direction: Read from peripheral address 0-x300 (ADC1BUF0)
 *                 and write to DMA RAM
 *                 AMODE: Register indirect with post increment
 *                 MODE: Continuous, Ping-Pong Mode
 *                 IRQ: ADC Interrupt
 *                 ADC stores results stored alternatively between
 *                 bufferA[] and bufferB[]
 *****************************************************************************/
void InitDma0(void)
{
 DMA0CONbits.AMODE = 0;   // Configure DMA for Register indirect with post increment
 DMA0CONbits.MODE  = 2;   // Configure DMA for Continuous Ping-Pong mode

 DMA0PAD=(int)&ADC1BUF0;
 DMA0CNT=(NUMSAMP-1);    
 
 DMA0REQ=13; 
 
 DMA0STAL = (unsigned int)&bufferA;
 DMA0STAH = (unsigned int)&bufferA;

 DMA0STBL = (unsigned int)&bufferB;
 DMA0STBH =(unsigned int)&bufferB; 


 IFS0bits.DMA0IF = 0;   //Clear the DMA interrupt flag bit
 IEC0bits.DMA0IE = 1;   //Set the DMA interrupt enable bit
 
 DMA0CONbits.CHEN=1;

 TRISBbits.TRISB1 = 0;
}
/******************************************************************************
 * Function:       void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void)
 *
 * PreCondition:   None
 *
 * Input:          None
 *
 * Output:         None
 *
 * Side Effects:   None
 *
 * Overview:       Process the ADC output.
 *****************************************************************************/
void __attribute__((interrupt, no_auto_psv)) _DMA0Interrupt(void)
{
 if(dmaBuffer == 0)
 {
    FIR(NUMSAMP, &outputSignal[0],  &bufferA[0], &firfilter); // FIR filtering on bufferA
 }
 else
 {
     FIR(NUMSAMP, &outputSignal[0],  &bufferB[0], &firfilter); // FIR filtering on bufferB
 }
#ifdef TEST_MODE
// Testing Reference Value consideration: Keep the POT(arrow mark of POT to extreme right/maximum)
// on expl16 board for testing
// Here we are checking for one of the samples to be correct
     if((outputSignal[0] >= (32650)) &&
        (outputSignal[0] <= (32750))
      )
        test_flag=1;
#endif

 dmaBuffer ^= 1;

 CtglPin();     // Toggle RA6

 _DMA0IF = 0;  //Clear the DMA0 Interrupt Flag

}

/*******************************************************************************
 End of File
*/







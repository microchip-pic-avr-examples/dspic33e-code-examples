![image](../images/microchip.jpg)

##  ADC Sampling at 1.1MSPS 

This file contains the following sections:
- Code Example Description
- Suggested Development Resources
- Reconfiguring the project for a different dsPIC33E device
- Revision History


## Code Example Description:

In this example, ADC is set up to convert AIN0 using CH0 and CH1 sample/hold in 10-bit sequential mode 
at 1.1MHz throughput rate. ADC clock is configured at 13.3Mhz or Tad=75ns
ADC Conversion Time for 10-bit conversion is Tc=12 * Tab =  900ns (1.1MHz).

void initAdc1(void);
ADC CH0 and CH1 S/H is set-up to covert AIN0 in 10-bit mode. ADC is configured to next sample data immediately after the conversion.
So, ADC keeps conversion data through CH0/CH1 S/H alternatively. Effective conversion rate is 1.1Mhz

void initDma0(void);
DMA channel 0 is configured in ping-pong mode to move the converted data from ADC to DMA RAM on every sample/convert sequence. 
It generates interrupt after every 16 sample transfer. 

void __attribute__((__interrupt__)) _DMA0Interrupt(void);
DMA interrupt service routine, moves the data from DMA buffer to ADC signal buffer and collects 256 samples.

The Toggle frequency of one pulse should be around 240us(micro second), if the operating clock frequency at 40Mhz.
Short AN0/AN1 with +3.3v to get analog signal for sampling. These values should be approximately around 0x7FXXX when checked in the bufferA/bufferB in the code.


## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dspic33ep512gm710(https://www.microchip.com/ma330035) /dspic33ep512mu810 (https://www.microchip.com/MA330025-1)/ dspic33ep256gp506 (https://www.microchip.com/MA330030) PIM
	
	
## Software Used 

- MPLAB® X IDE v5.50 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v1.70 or newer (https://www.microchip.com/xc)

## Reconfiguring the project for a different dsPIC33E device:

The Project/Workspace can be easily reconfigured for dspic33ep512gm710/dspic33ep512mu810/dspic33ep256gp506 device.
Please use the following general guidelines:

a. Change device selection within MPLAB® IDE to dspic33ep512gm710/dspic33ep512mu810/dspic33ep256gp506 device of
   your choice by using the following menu option:
   MPLAB X>>Configuration drop-down option>><Listed Device Configuration>

b. Re-build the MPLAB® project using the menu option:
   MPLAB X>>Build Main Project

c. Download the hex file into the device and run.


## Revision History :
    04/01/2006 - Initial Release of the Code Example
    7/01/2010 - Code Example updated for dsPIC33E
	02/13/2014 - Code Example updated to new format for dspic33ep512gm710/dspic33ep512mu810/dspic33ep256gp506
	11/13/2014 - TEST_MODE code added for test automation
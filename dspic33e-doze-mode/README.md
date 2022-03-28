![image](../images/microchip.jpg)

## CPU IN DOZE MODE

## Description:

In this example, CPU is setup in DOZE mode to run at 1/128 of the System Clock. 
Device is initially configured to run at Fcy=60Mhz and then DOZE mode is enabled to run the CPU at 468.75Khz (60M/128).
Check the RA4 pin toggles i.e at a frequency of ~234khz.

extern void setDozeRatio(unsigned int r);
This function sets the required DOZE ratio and enables the DOZE mode

## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) or dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1) or dsPIC33EP256GP506 PIM (https://www.microchip.com/MA330030)	
	
	
## Software Used 

- MPLAB® X IDE v5.50 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v1.70 or newer (https://www.microchip.com/xc)


![image](../images/microchip.jpg)

## Clock Switch

## Description:

In this example, CPU is initially configured to run from external secondary osc and then clock switching 
is initiated to run from Internal FRC.
The RA4 pin toggles at frequency of 1/8th of system clock frequency.

extern void clockSwitch(unsigned int r);
This function selects the next clock input and initiates clock switch sequence.


## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) or dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1) or dsPIC33EP256GP506 PIM (https://www.microchip.com/MA330030)	
	
	
## Software Used 

- MPLAB® X IDE v5.50 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v1.70 or newer (https://www.microchip.com/xc)


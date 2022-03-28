![image](../images/microchip.jpg)

## INTR IN AUX FLASH ALT 

## Description:

This example demonstrates how to handle interrupts in auxiliary flash. Timer1 interrupt is used to demonstrate this.

In this example a modified version of the linker file(dspic33E_pic24e.gld) is used. 
The linker file is modified such that all the functions (including interrupt functions) are written in the auxiliary flash of program memory. 
A default interrupt is implemented which checks for the corresponding interrupt flag to find the actual interrupt.

Note : There is warning when build the project regarding the Aux_Interrupt. This is a general warning appears when a linker file is edited.
	   

## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1)
	
## Software Used 

- MPLAB® X IDE v5.50 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v1.70 or newer (https://www.microchip.com/xc)


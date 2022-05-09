![image](../images/microchip.jpg)

## External interrupt pins - Configuration and Use

## Description:

Microchip's 16-bit dsPIC� Digital Signal Controllers feature up to 5 external interrupt pins - INT0 through INT4.
These pins may be configured to interrupt on either a rising or a falling edge of a signal.

The attached code example written in C demonstrates how these pins may be configured to interrupt the CPU.
Both, initialization and interrupt service routines have been provided.
 
The code example in the file  traps.c  contains trap service routines (handlers) for hardware exceptions
generated by the dsPIC33E device

For Testing:  
dspic33ep512gm710 : connect the pins RB3 and RF8  
dspic33ep512mu810 : connect the pins RA0 and RA4  
dspic33ep256gp506 : connect the pins RB5 and RF8  

Note :- The PPS configuration in the external_interrupts.c source file changes with the device being used. The user is advised
to refer the datasheet and use the appropriate values for RPINR/RPOR registers for proper operation.

## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) or dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1) or dsPIC33EP256GP506 PIM (https://www.microchip.com/MA330030)	
	
	
## Software Used 

- MPLAB� X IDE v6.00 or newer (https://www.microchip.com/mplabx)
- MPLAB� XC16 v2.00 or newer (https://www.microchip.com/xc)

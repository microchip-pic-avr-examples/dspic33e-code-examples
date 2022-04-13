![image](../images/microchip.jpg)

## Intr In AUX Flash 

## Description:

This example demonstrates how to handle interrupts in auxiliary flash. Timer1 interrupt is used to demonstrate this.

aux_int.s 	- 	contains the default ISR for all interrupts in Aux flash   
init_timer1.c	-	initialise timer 1 interrupt  
main.c		-	contains main and ISR for timer 1  

To use this example in your project, include the aux_int.s file.


## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1) 
	
## Software Used 

- MPLAB® X IDE v6.00 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v2.00 or newer (https://www.microchip.com/xc)


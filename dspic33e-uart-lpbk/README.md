![image](../images/microchip.jpg)

## UART LOOP-BACK 

## Description:

In this code examples, UART receives and buffers characters from the hyperterminal at 9600 baudrate.
Once 8 characters are received, UART transmits (echoes) them back onto hyperterminal.

Note :- The PPS configuration in the uart_config.c source file changes with the device being used. The user is advised
to refer the datasheet and use the appropriate values for RPINR/RPOR registers for proper operation.

## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) or dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1) or dsPIC33EP256GP506 PIM (https://www.microchip.com/MA330030)	
	
	
## Software Used 

- MPLAB® X IDE v5.50 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v1.70 or newer (https://www.microchip.com/xc)


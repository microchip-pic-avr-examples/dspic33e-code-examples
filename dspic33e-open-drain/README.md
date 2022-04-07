![image](../images/microchip.jpg)

## OPEN DRAIN CONFIGURATION 

## Description:

This code shows an example of setting the Open Drain Configuration for a generic I/O Port.
The open-drain feature allows the generation of outputs higher than VDD
(e.g., 5V on a 5V tolerant pin)by using external pull-up resistors.
An external Pull up resistor should be connected for the port configured as
open drain output.  

Open drain output is default high because of the external pull up resistor.
In software the port configured as open drain is set to low during initialization
and set as high when Switch S3 is pressed.
Refer ce419_i2c_eeprom code example for a peripheral configured as open drain output.

## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) or dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1) or dsPIC33EP256GP506 PIM (https://www.microchip.com/MA330030)	
	
	
## Software Used 

- MPLAB® X IDE v6.00 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v2.00 or newer (https://www.microchip.com/xc)


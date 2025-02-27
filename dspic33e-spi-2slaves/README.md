![image](../images/microchip.jpg)

## USING 2 SPI SLAVE PERIPHERALS WITH ONE SPI MODULE

## Description:

This code example shows using the SPI module in conjunction with 2 GPIO pins to communicate with 2 different slave
devices.  The concept is that the GPIO are the slave selects for the individual devices, but the SCL, SDI and SDO are shared between the two processors.  
This is also scalable up to n devices, where your only limitation is the bus bandwidth on the SPI bus and the number of GPIO lines at your disposal.

![image](../images/dspic33e-spi-2slaves.JPG)

The code alternates between sending data from one device to another and mixed.

## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) or dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1) or dsPIC33EP256GP506 PIM (https://www.microchip.com/MA330030)	
	
	
## Software Used 

- MPLAB� X IDE v6.00 or newer (https://www.microchip.com/mplabx)
- MPLAB� XC16 v2.00 or newer (https://www.microchip.com/xc)


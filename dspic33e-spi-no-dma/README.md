![image](../images/microchip.jpg)

## SPI NO DMA 

## Description:

This code example shows using a single SPI module in conjunction with a GPIO to generate an SPI communication that will work with most SPI Slave devices.

![image](../images/dspic33e-spi-no-dma.jpg)

The code sends from 00h through FFh and then cycles through again.  A delay was added to permit catching the output on a logic analyzer easier.

Externally Connect for testing:
-------------------------------
RF7 & RF8 on Expl16 board for dspic33ep256GP506 PIM<br/>
RF7 & RF8 on Expl16 board for dspic33ep512gm710 PIM<br/>
RF2 & RF3 on Expl16 board for dspic33ep512mu810 PIM<br/>


## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) or dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1) or dsPIC33EP256GP506 PIM (https://www.microchip.com/MA330030)	
	
	
## Software Used 

- MPLAB® X IDE v6.00 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v2.00 or newer (https://www.microchip.com/xc)


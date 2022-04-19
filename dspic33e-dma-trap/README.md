![image](../images/microchip.jpg)

## DMA TRAP

## Description:

DMA generates trap error in the following conditions.

DMA Write collision:  
DMA write collision occurs when both DMA module and CPU tries to write 
to the same DMA RAM memory location.

Peripheral Write Collision:  
Peripheral write collision occurs when both DMA module and CPU tries to write
to the same peripheral SFR. 
	
In this code example, UART is configured to continuously transmit/receive data in loop-back mode. 
In the back-ground loop, CPU tries to write to DMA RAM or peripheral SFR to create DMA trap condition.

Select the required condition using the following macro in main.c function to generate DMA trap.

// Source Selection for Trap Creation  
#define PER_WRITE_COL 1  
#define DMA_WRITE_COL 0  


## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) or dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1) or dsPIC33EP256GP506 PIM (https://www.microchip.com/MA330030)	
	
	
## Software Used 

- MPLAB® X IDE v6.00 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v2.00 or newer (https://www.microchip.com/xc)


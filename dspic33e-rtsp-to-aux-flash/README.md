![image](../images/microchip.jpg)

## RTSP of Auxiliary Flash Memory from Primary Flash Memory

## Description:

This code example demonstrates how user code running from Primary Flash Program Memory
can perform Run-Time Self-Programming (RTSP) of the Auxiliary Flash Program Memory. Note 
that the user code does not get stalled when the Auxiliary Flash RTSP operation is in progress.
In this example, a page-sized array of data has been initialized and defined as an Auxiliary Flash
constant array using the space(auxflash) attribute of the compiler. Then, the user code erases
the entire page in Auxiliary Flash Program Memory and modifies the contents of one of the rows
in the page using RTSP. Subsequently, the user code reads back the contents of the page into
an array called 'pageMirrorBuff'. Verify that the specified row (row 7) now contains the modified
data and the remaining rows are in their original state.


## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1)
	
## Software Used 

- MPLAB® X IDE v6.00 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v2.00 or newer (https://www.microchip.com/xc)


![image](../images/microchip.jpg)

## RTSP WORD WRITE

## Description:

In this example, a page of Flash memory (512 instructions or 8 rows of 64 instruction) is read first.
Then the page is erased fully. One row of the page is modified and written back to the flash. Then, a few words
in the already programmed flash are modified and read back to verify the modification.

Following RTSP Application Program Interface (APIs) are used to perform the operation.

// Flash Memory is organised into ROWs of 64 instructions or 192 bytes<br/>
// RTSP allows the user to erase a PAGE of memory which consists of EIGHT ROWs (512 instructions or 1536byts) at a time.<br/>
// RTSP allows the user to program a ROW (64 instructions or 192 bytes) at a time<br/>

## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) or dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1) or dsPIC33EP256GP506 PIM (https://www.microchip.com/MA330030)	
	
	
## Software Used 

- MPLAB® X IDE v6.00 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v2.00 or newer (https://www.microchip.com/xc)


![image](../images/microchip.jpg)

## Flash RTSP code example

## Description:

In this code example for dsp33ep512mu810, a page of Flash memory (1024 instructions or 8 rows of 128 instruction) is read first.
Then the page is erased fully. One row of the page is modified and written back to the flash.

Following RTSP Application Program Interface (APIs) are used to perform the operation.

Flash Memory is organised into ROWs of 128 instructions or 384 bytes
RTSP allows the user to erase a PAGE of memory which consists of EIGHT ROWs (1024 instructions or 3072 bytes) at a time.
RTSP allows the user to program a ROW (128 instructions or 384 bytes) at a time


## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) or dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1) or dsPIC33EP256GP506 PIM (https://www.microchip.com/MA330030)	
	
	
## Software Used 

- MPLAB® X IDE v6.00 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v2.00 or newer (https://www.microchip.com/xc)


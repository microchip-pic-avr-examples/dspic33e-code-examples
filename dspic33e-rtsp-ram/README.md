![image](../images/microchip.jpg)

## RTSP RAM

## Description:

This code example shows the run time self programming (RTSP) of the flash directly from the RAM (data memory) without
the use of write latches in between. The RAM source address from where the data is to be taken is stored in the 
NVMSRCADRL/H registers and the destination flash address to where the data should be programmed is stored in the
NVMADR/NVMADRU registers. When the NVMCON is configured to program one row of flash at a time and the NVMCON.WR bit
is set, the destination row gets written without the intervention of write latches. The same procedure when repeated
can be used to program a page of flash program memory. 

## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) 
	
## Software Used 

- MPLAB® X IDE v6.00 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v2.00 or newer (https://www.microchip.com/xc)


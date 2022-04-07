![image](../images/microchip.jpg)

## CRC GENERATION 

## Description:

In this code examples, CRC module is used to generate CRC for input data.

CRC_Calc_ChecksumByte()  
This function calculates the CRC Checksum for the array of bytes provided by the user based on the polynomial 
set in the CRCXORH and CRCXORL registers

The initial CRC value in the CRCWDATL register depends on the CRC polynomial being used in the code. 

The CRC-CCITT polynomial requires 0x84CF to be loaded as the initial value in CRCWDATL register. 
The CRC16 and CRC32 polynomials require 0x0000 to be loaded as the initial value in CRCWDATL register. 


## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) or dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1) or dsPIC33EP256GP506 PIM (https://www.microchip.com/MA330030)	
	
	
## Software Used 

- MPLAB® X IDE v6.00 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v2.00 or newer (https://www.microchip.com/xc)


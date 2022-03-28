![image](../images/microchip.jpg)

## CROSSWIRE COMMUNICATION BETWEEN CAN 1 AND CAN 2 MODULES 

## Description:

The following boards are used for developing this code example
Explorer 16 Development Board
ECAN PicTail Card

Connections to be done on the ECANPicTail Card - The Tx line of ECAN1 is connected to the Rx line of ECAN2 
and the Tx line of ECAN2 is connected to the Rx line of ECAN1. 

Alternatively a simple 2 node (ECAN1 and ECAN2) CAN BUS can be implemented using a twisted wire pair. This twisted wire pair must be connected at the output of the transcievers. Jumpers J7 and J6 must be set.

MPLLAB ICD3 Debugger/Programmer is used for debugging and running this code example.

b. dsPIC33EP256GP506
The following boards are used for developing this code example

Explorer 16 Development Board
CAN BUS Analyzer

Connect ECAN1 of 506 device to CAN Bus Analyzer.
Press switch S3 on explorer 16 board to transmit ECAN message. Similarly transmit ecan message
from CAN bus analyzer and observe the received message in rx_ecan1message buffer.

MPLAB ICD3 Debugger/Programmer is used for debugging and running this code example.


## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) or dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1) or dsPIC33EP256GP506 PIM (https://www.microchip.com/MA330030)	
	
	
## Software Used 

- MPLAB® X IDE v5.50 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v1.70 or newer (https://www.microchip.com/xc)


![image](../images/microchip.jpg)

## USING RTDM DRIVER FOR COMMUNICATION WITH DMCI 

## Description:

This code example shows how to use Real-Time Data Monitor (RTDM) to create an 
alternative link between Host PC and target device for debugging applications 
in real-time using MPLAB DMCI ( MPLAB 8.10 or higher).  
It is required to include the RTDM.C file and RTDM.h into the application project 
in order to send/receive data through the UART to/from the host PC running under 
MPLAB DMCI environment.  <br/><br/>
DMCI included in MPLAB 8.10 or higher is ready and enabled to support data exchange 
between the host PC and target device. Previous versions of DMCI do not support this feature. 
RTDM is currently supported by PIC24H, dsPIC30F , dsPIC33F , PIC24E and dsPIC33E processors


Function: RTDM_Start()<br/>
Overview: Here is where the RTDM code initializes the UART to be used to
		  exchange data with the host PC<br/>
Note:	  Some processors may have 2 UART modules, that is why it is required to
		  specify which UART module is going to be used by RTDM	


Function: RTDM_ProcessMsgs()<br/>
Overview: Here is where the RTDM code process the message received and then 
		  executes the required task. These tasks are reading an specified memory
		  location, writing an specified memory location, receive a communication
		  link sanity check command, or being asked for the size of the buffers.
		  
Function: CloseRTDM()<br/>
Overview: Here is where the RTDM code closes the UART used to
		  exchange data with the host PC


## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) or dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1) or dsPIC33EP256GP506 PIM (https://www.microchip.com/MA330030)	
	
	
## Software Used 

- MPLAB® X IDE v6.00 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v2.00 or newer (https://www.microchip.com/xc)


![image](../images/microchip.jpg)

## PARALLEL MASTER PORT (PMP) MODULE 

## Description:

This code example aims to demonstrate the basic initialisation and operation of the Parallel Master Port (PMP) module.
It also demonstrates to communicate with the LCD module using PMP and display a set of characters on the LCD pannel
The PMP module is configured in the Master Mode.


Set the I/O ports as digital ports.


mLDCInit()<br />
This function will initialise the LCDinit pointer to sets LCDState mchine to _uLCDstate = 2


TimerInit()<br />
This function will enable to run the Timer


BannerStart()<br />
This function will enable to Setup the banner processing<br />
The following events are performed in this function
-  Check if the LCD is busy
-  Set the Cursor to the starting point
-  Initialise the banner number and banner length


In the While loop the following events follow.

LCDProcessEvents()<br />
This is a state machine to issue commands and data to LCD. Must be called periodically to make LCD message processing.
The communication with the LCD module is enabled through the PMP module.
This function Initialises the LCD function + Processes a series of LCD events.

-  Initialises the LCD function.<br />
   Open the PMP module for communication with LCD ( Case2) 
   complete a set of events to initialise the LCD from (Case 64 to case 71)
   Clear the LCD state machine i.e (_uLCDstate = 0)


TimerIsOverflowEvent()<br />
Only on timer Overflow the BannerProcessEvents is processed


BannerProcessEvents()<br />
-  Display the Banner array 
   Pick the character to be displayed from the banner pointer
   Prepare the character for display
   Select the first line or second line for display.
After the display of banner array the control moves back to the while loop.

The banner is displayed sequentially.


## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) or dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1) 
	
## Software Used 

- MPLAB® X IDE v6.00 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v2.00 or newer (https://www.microchip.com/xc)


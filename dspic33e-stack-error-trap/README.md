![image](../images/microchip.jpg)

## STACK ERROR TRAPS FOR EASY DEBUGGING 

## Description:

Microchip's 16-bit dsPIC® Digital Signal Controllers feature a software stack, i.e., the stack is part of general 
purpose RAM. PUSH and POP instructions use the W15 register as a stack pointer to place variables on to the 
top of stack location or to unload variables from the top of stack location in RAM. 

The CPU also features a mechanism to detect software errors and take corrective action.

Specifically, the ability to detect over-utilization or under-utilization of the Stack is provided by 
means of automatic Stack Error Trap detection. Stack Errors may be caused by one of the following:

a. Stack Overflow - W15 is greater than the SPLIM register
b. Stack Underflow - W15 is lesser than the base address of RAM

If the application defines a Stack Error Trap service routine (trap handler), the processor will vector 
to the trap handler when it detects a Stack Error trap.



Note:
The user should note that the MPLAB® xc16 C compiler will not intentionally generate any instructions 
that cause a stack error trap to occur.

## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) or dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1) or dsPIC33EP256GP506 PIM (https://www.microchip.com/MA330030)	
	
	
## Software Used 

- MPLAB® X IDE v5.50 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v1.70 or newer (https://www.microchip.com/xc)


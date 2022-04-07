![image](../images/microchip.jpg)

## OSCILLATOR FAILURE TRAPS AND FAIL-SAFE CLOCK MONITORING 

## Description:

Microchip's 16-bit dsPIC® Digital Signal Controllers feature an on-chip mechanism to detect oscillator/clock failures.
In addition, the processor features the ability to continue code execution using an internal Fast RC oscillator.

The attached code example demonstrates:
a. How the Fail-Safe Clock Monitor module may be enabled via config macros.
b. How to write an effective Oscillator Failure trap service routine.

In this example, the device is configured to operate off an external canned oscillator running via the PLL. 
The code in the main.c file simply toggles a general purpose I/O port pin,RA7.
The user may place an oscilloscope probe on RA7 to verify the frequency at which the it is being toggled.
The user may then simply disconnect the canned oscillator from the board or ground it's output to simulate a clock failure.
The device continues to operate using the Internal FRC oscillator. 
The CPU vectors to the Oscillator Failure trap and clears the trap condition via a special write sequence 
to the OSCCONL register. 

Further, a software flag is set if the PLL was found to be out-of-lock.

Note: In this project clock switching and clock monitoring has been enabled. MPLAB IDE will warn the user 
to perform a POR on the board in this case. This is being done because when clock switching has been enabled, 
the device uses the clock active prior to the last MCLR reset event (specified in the COSC bits in OSCCON register) 
rather than the clock settings specified by the FOSC fuse. In order for the device to use the clock source specified 
by the FOSC fuse, a POR reset event must take place.

Note:
The user should note that the MPLAB® xc16 C compiler will not intentionally generate any instructions 
that cause a stack error trap to occur.

## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) or dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1) or dsPIC33EP256GP506 PIM (https://www.microchip.com/MA330030)	
	
	
## Software Used 

- MPLAB® X IDE v6.00 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v2.00 or newer (https://www.microchip.com/xc)


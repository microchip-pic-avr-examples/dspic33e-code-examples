![image](../images/microchip.jpg)

## Math Error Traps for Robust Operation

## Description:

Microchip's 16-bit dsPIC® Digital Signal Controllers feature an on-chip mechanism to detect software errors and 
take corrective action. Specifically, the ability to detect arithmetic (math) errors is provided by means of 
automatic Math Error Trap detection.

Math errors may be caused by one of the following:

1.Divide by Zero

2.Accumulator A overflow (bit 31 destroyed)

3.Accumulator B Overflow (bit 31 destroyed)

4.Catastrophic overflow of Accumulator A (bit 39 destroyed)

5.Catastrophic overflow of Accumulator B (bit 39 destroyed)

6.Accumulator Shift count error

If the application defines an Math Error Trap service routine (trap handler), the processor will vector to the 
trap handler when it detects a math error.

NOTE: This routine also estimates the instruction that caused the math error trap to occur by examining the 
PC value that is stored in the stack prior to entering the Math Error Trap. Since the instruction that causes 
divide by zero error to occur is not executed, the stacked PC points to the offending instruction. 
However, the instructions that cause accumulator overflows to occur will be executed prior to the trap being caused. 
So the stacked PC will point to the instruction after the offending instruction. 
Thus, the estimation routines differ slightly for the Divide by Zero error and the Accumulator Overflow errors. 
It should also be noted that since this trap routine is written in C the estimation of the stacked PC will 
depend on the compiler optimization level set up for this file. In the trap routine presented here, a comiler 
optimization level of 0 is assumed for this file.


## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) or dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1) or dsPIC33EP256GP506 PIM (https://www.microchip.com/MA330030)	
	
## Software Used 

- MPLAB® X IDE v6.00 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v2.00 or newer (https://www.microchip.com/xc)


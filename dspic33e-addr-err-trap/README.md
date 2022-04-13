![image](../images/microchip.jpg)

## Address Error Traps for Easy Debugging 

## Description:

Microchip's 16-bit dsPIC® Digital Signal Controllers feature an on-chip mechanism to detect software errors and 
take corrective action. Specifically, the ability to detect memory addressing errors is provided by means of 
automatic Address Error Trap detection. Memory addressing errors may be caused by one of the following:

1.A misaligned data word fetch is attempted. This condition occurs when an instruction performs a word 
   access with the LSb of the effective address set to ‘1’. The dsPIC33E CPU requires all word accesses to 
   be aligned to an even address boundary.

2.A bit manipulation instruction using the Indirect Addressing mode with the LSb of the effective address set to ‘1’.

3.A data fetch from unimplemented data address space is attempted.

4.Execution of a “BRA #literal” instruction or a “GOTO #literal” instruction, where literal is an unimplemented 
   program memory address.

5.Executing instructions after modifying the PC to point to unimplemented program memory addresses. 
   The PC may be modified by loading a value into the stack and executing a RETURN instruction.

If the application defines an Address Error Trap service routine (trap handler), the processor will vector to the 
trap handler when it detects an Address Error trap.


Note:
The user should note that the MPLAB® xc16 C compiler will not intentionally generate any instructions that cause 
an address error trap to occur. The compiler/assembler will also detect address error instances in code whenever possible.


## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) or dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1) or dsPIC33EP256GP506 PIM (https://www.microchip.com/MA330030)	
	
	
## Software Used 

- MPLAB® X IDE v6.00 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v2.00 or newer (https://www.microchip.com/xc)


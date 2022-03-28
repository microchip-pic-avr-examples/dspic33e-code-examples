![image](../images/microchip.jpg)

## MANUAL EDS USAGE 

## Description:

This code example demonstrates how a mixture of automatic (compiler-managed) and manual
(user-managed) Extended Data Space (EDS) variables can be defined and used in an application.
In this example, 3 data arrays are defined to be stored in Extended Data Space (EDS):
'm1', 'm2' and 'm'. Of these, 'm' has been defined as a compiler-managed EDS variable, i.e. 
the EDS registers are automatically managed by the compiler without requiring the user code
to modify or save/restore the contents of the EDS registers. 'm1' and 'm2' have been defined
as EDS variables that the user application needs to manage manually (i.e. the compiler will not
modify or save/restore the value of the DSRPAG registers). In addition, a non-EDS array 'x' has
been defined. The code example performs an element-by-element Vector Multiplication of the 
'm1', 'm2' and 'm' arrays with the array 'x', and stores the products in arrays 'sum1', 'sum2' 
and 'sum', respectively. The arrays 'sum1' and 'sum2' have been defined as EDS variables 
that the user application needs to manage manually (i.e. the user needs to manually modify 
and save/restore the DSWPAG register in this case), whereas 'sum' has been defined as a
compiler-managed EDS variable.   

## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) or dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1) or dsPIC33EP256GP506 PIM (https://www.microchip.com/MA330030)	
	
	
## Software Used 

- MPLAB® X IDE v5.50 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v1.70 or newer (https://www.microchip.com/xc)


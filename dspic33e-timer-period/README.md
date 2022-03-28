![image](../images/microchip.jpg)

## USING TIMER1 FOR PERIOD INTERRUPTS 

## Description:

a. This Code example gives a demonstration of how to use the Timer1 for Period Interrupts .

b. When ever the Timer1 register is equal to the Period Register the Timer1 generates a Interrupt 
   and Toggles the PORTA.1 bit  of the PORTA i:e D4 LED on Explorer 16 .

c. In order to configure the Period Value Configure the PR1 register in the init_timer1.c file.

d. This code examples uses the value of M as 38 and N1 and N2 as 2 so and the External Crystal Frequency as 8 Mhz .

   In Case you are using the Externel Crystal value more then 8 Mhz you have to reconfigure the values of M ,N1,N2 (any one of
   them will also do) so that the Processore runs at or below 40MIPS otherwise the processor will run above 40MIPS and unexpected 
   results may occur.

   Example:

   If using External Crystal of 8Mhz M1=40,N1=N2=2 then Fosc=80 Mhz and Fcy= 40Mhz 
      External Crystal = 12Mhz M1=24,N1=N2=2 then Fosc=74Mhz and Fcy=37Mhz 
e. It is recommended to start thr processor on a clock without PLL and then enter the values to the PLL register and
   then switch to PLL mode . The same is being followed in this code.

f. Init_timer1.c--------For Initialization of Timer1 for Period Interrupt

g. Switch.s-------------For Switching from the non PLL mode to PLL mode with XT mode

h. main.c---------------Main Application running in the code.


## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) or dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1) or dsPIC33EP256GP506 PIM (https://www.microchip.com/MA330030)	
	
	
## Software Used 

- MPLAB® X IDE v5.50 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v1.70 or newer (https://www.microchip.com/xc)


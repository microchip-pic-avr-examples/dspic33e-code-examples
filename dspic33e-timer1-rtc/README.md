![image](../images/microchip.jpg)

## TIMER1 USED IN REAL-TIME CLOCK APPLICATIONS

## Description:

dsPIC Digital Signal Controllers feature several on-chip general-purpose
timers. Of these, the Timer1 module has the capability to be clocked by
an external asynchronous 32KHz crystal connected to the device via the
SOSCI and SOSCO pins. The attached code example demonstrates how Timer1
may be configured to use the 32KHz secondary oscillator for a real-time
clock (RTC) application.
Configuring Timer1 for the real-time clock application is a two-step
process. In the first step, the code demonstrates how the secondary
oscillator may be enabled via a special write sequence to the OSCCON
register. In the second step, the code demonstrates how the Timer1
is configured for using an external asynchronous clock. In addition
to these steps, the code also demonstrates Timer1 interrupt operation.



## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1)	
	
	
## Software Used 

- MPLAB® X IDE v5.50 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v1.70 or newer (https://www.microchip.com/xc)


![image](../images/microchip.jpg)

## FAST WAKE_UP FROM SLEEP MODE 

## Description:

Microchip's 16-bit dsPIC® Digital Signal Controllers are capable of resuming regular operation after waking up from a 
low-power mode, for example, SLEEP mode, within 10 microseconds. Additionally, within 30 microseconds of waking up from 
SLEEP mode, the device can operate at its highest speed of operation.To wake up from SLEEP in 10 microseconds or less, 
the device should be made to operate off the Fast RC (FRC) Oscillator prior to entering SLEEP mode. To wake up from SLEEP
and operate at maximum speed in 30 microseconds or lesser, the device should be made to operate off the Fast RC (FRC)
Oscillator with the PLL enabled, prior to entering SLEEP mode. This is possible because the PLL locks in 20 microseconds 
and the FRC oscillator, unlike a crystal oscillator has no start-up time. The POR circuit in the device inserts a small 
delay of 10 microseconds to ensure all bias circuits have stabilized on a power-up event.

The attached code example demonstrates this capability using external interrupt pin, INT1. Using this example, the user
can measure the time elapsed between a falling edge on the INT1 pin and a rising edge on any pin on Port D. 
The time measured will be the time to wake up from SLEEP using an FRC oscillator plus the time taken by the PLL to lock plus
the 5*Tcy consistent interrupt service routine entry time.The attached code example will
configure the PLLFBD register which provides a factor ‘M’, by which the input to the VCO is multiplied.
For devices that do not feature the FRCxPLL option, the FOSC configuration register will be set up to use the FRC oscillator on power-up.

The benefit offered by dsPIC33E devices to low-power applications is not only a low-power SLEEP mode option but also the 
option to wake-up from SLEEP in a short amount of time and execute code at the highest speed possible.


## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) or dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1) or dsPIC33EP256GP506 PIM (https://www.microchip.com/MA330030)	
	
	
## Software Used 

- MPLAB® X IDE v6.00 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v2.00 or newer (https://www.microchip.com/xc)


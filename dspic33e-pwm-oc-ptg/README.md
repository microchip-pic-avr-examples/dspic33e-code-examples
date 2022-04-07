![image](../images/microchip.jpg)

## PWM OC AND PTG 

## Description:

In this example, Timer 2 is setup to time-out every 40 microseconds (25Khz Rate). 
This is used as synchronisation source of Output Compare Module 1

Output Compare Module 1 is configured in Edge aligned PWM mode and Duty cycle is set for 5us(12.5% duty)
OC1 is clocked from Peripheral Clock .

PTG sequencer is set to wait for OC1 rising edge and then wait for half of the PWM period i.e 20us.
This is done by configuring PTG general purpose timer PTGTO.After the time out of 20 us ,the PTG will generate 
PTG Trigger Ouput1.And this is selects as synchronisation source of Output Compare 2 (OC2).Thus inserting 
phase shift of half the PWM period between OC1 and OC2 outputs.

PTG Timer is configured for delay of 20us and calculation is as follows:

Required delay: 20us<br/>
PTG Clock: 70MHz<br/>
COunt in PTGT0LIm: 20us/(1/70MHz) = 20us*70Mhz = 1400;<br/>

In this example OC2 is configured for 25% duty cycle or 10us .

And the OC1 and OC2 outputs are mapped to RP54,RP55 pin respectively.
                   
![image](../images/dspic33e-pwm-oc-ptg.jpg)

This can be reconfigured according to application requirement. 

## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) 

	
## Software Used 

- MPLAB� X IDE v6.00 or newer (https://www.microchip.com/mplabx)
- MPLAB� XC16 v2.00 or newer (https://www.microchip.com/xc)


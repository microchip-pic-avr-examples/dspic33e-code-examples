![image](../images/microchip.jpg)

## ADC CHANNEL SCANNING WITHOUT DMA 

## Description:

In this example, Timer 3 is setup to time-out every 125 microseconds (8Khz Rate). 
As a result, the module will stop sampling and trigger a A/D conversion on every Timer3 time-out, i.e., Ts=125us. 

ADC is configured in 10bit mode to sequentially scan AIN4, AIN5, AIN10 and AIN13 on Timer 3 interrupt. 
It will take FOUR Timer3 Timeout period to scan through all the FOUR Analog inputs.

ADC module clock time period is configured as Tad=Tcy*(ADCS+1)= (1/40M)*64 = 1.6us (625Khz). 
Hence the conversion time for 10-bit A/D Conversion Time Tc=12*Tad = 19.2us

void __attribute__((__interrupt__)) _ADC1Interrupt(void)
ADC ISR sorts out the data and stores the converted data in separated buffers.
ISR rate will be 8Khz, RA6 pin is toggled in ISR, hence it will be toggling at ~ 4Khz


## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) or dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1) or dsPIC33EP256GP506 PIM (https://www.microchip.com/MA330030)	
	
	
## Software Used 

- MPLAB® X IDE v5.50 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v1.70 or newer (https://www.microchip.com/xc)


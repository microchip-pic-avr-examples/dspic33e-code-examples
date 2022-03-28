![image](../images/microchip.jpg)

## ADC CONVERSION IN SLEEP MODE 

## Description:

In this example, ADC is set up to convert AIN5 using CH0 S/H and ADC is operating using its internal RC osc. 
Start of conversion is issued in the background loop and device enters sleep mode immediately after that. 

When the ADC conversion is completed in sleep mode, it wakes up the device and enters ADC ISR.


void initAdc1(void);
ADC CH0 is set-up to covert AIN5 in 10-bit mode. ADC is configured to next sample data immediately after the conversion.
But the start of conversion is issued manually in the background loop.

void _ADC1Interrupt()
Device enters the ADC ISR after waking up from sleep mode. ADC result is read in the ISR and PORTA (RA4) pin is toggled.


## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) or dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1) or dsPIC33EP256GP506 PIM (https://www.microchip.com/MA330030)	
	
	
## Software Used 

- MPLAB® X IDE v5.50 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v1.70 or newer (https://www.microchip.com/xc)


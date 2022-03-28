![image](../images/microchip.jpg)

## ADC Alternate Sampling 

## Description:

In this example, Timer 3 is setup to time-out every 125 microseconds (8Khz Rate). 
As a result, the module will stop sampling and trigger a A/D conversion on every Timer3 time-out, i.e., Ts=125us. 

ADC is configured in 10bit mode to alternatively sample AN4/AN5 analog input on Timer 3 interrupt. 
It will take TWO Timer3 Timeout period to sample AN4 first and then AN5.

ADC module clock time period is configured as Tad=Tcy*(ADCS+1)= (1/60M)*64 = 1.06us (625Khz). 
Hence the conversion time for 10-bit A/D Conversion Time Tc=12*Tad = 12.72us

DMA is used to sort and transfer the converted data to DMA RAM. DMA is configured in ping-pong mode 
and it transfers 16samples of each of the TWO analog inputs and generates interrupt. 


DMA channel 0 is configured in ping-pong mode to move the converted data from ADC to DMA RAM 
on every sample/convert sequence. 
First, DMA uses DMA0STA base address to store the ADC samples and it generates interrupt 
after transfering (TWO x 16 samples = 32 samples).
Next, DMA uses DMA0STB base address to store the ADC samples and it generates interrupt
after transfer (TWO x 16 samples = 32 samples).
Above process repeats continuously. 

void __attribute__((__interrupt__)) _DMA0Interrupt(void);
DMA interrupt service routine, moves the data from DMA buffer to ADC signal buffer 

Timer time outs at 60M/(4999+1) = 12000 Hz.
DMA interrupt @ 12K/32=  375 Hz.
I/O pin toggles at 375/2= 187 Hz.

RA4 pin is toggled in ISR, hence it will be toggling at ~ 187Hz



## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) or dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1) or dsPIC33EP256GP506 PIM (https://www.microchip.com/MA330030)	
	
	
## Software Used 

- MPLAB® X IDE v5.50 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v1.70 or newer (https://www.microchip.com/xc)


![image](../images/microchip.jpg) 

## ADC Sampling and IIR Filtering 

## Description:

In this example, ADC is configured to sample (AIN5) at 8Khz rate and converted data is assembled as 256 sample buffer before triggering filtering operation.

Timer 3 is setup to time-out every 125 microseconds (8Khz Rate).   
As a result, the module will stop sampling and trigger a 12-bit A/D conversion on every Timer3 time-out, i.e., Ts=125us.   
At that time, the conversion process starts and completes Tc=14*Tad periods later.  
When the conversion completes, the module starts sampling again. However, since Timer3 
is already on and counting, about (Ts-Tc)us later, Timer3 will expire again and trigger 
next conversion. 

ADC module clock time period is configured as Tad=Tcy*(ADCS+1)= (1/40M)*64 = 1.6us (625Khz). 
Hence the conversion time for 12-bit A/D Conversion Time Tc=14*Tad = 22.4us

void initTmr3();  
Timer 3 is configured to time-out at 8Khz rate. 

void initAdc1(void);  
ADC module is set-up to convert AIN5 input using CH0 S/H on Timer 3 event in 12-bit mode.

void initDma0(void);  
DMA channel 0 is configured in ping-pong mode to move the converted data from ADC to DMA RAM on every sample/convert sequence. 
It generates interrupt after every 256 sample transfer. 

void \_\_attribute\_\_((\_\_interrupt\_\_)) _DMA0Interrupt(void);  
DMA interrupt service routine performs IIR filtering on the data buffer.


## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) or dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1) or dsPIC33EP256GP506 PIM (https://www.microchip.com/MA330030)
	
	
## Software Used 

- MPLAB® X IDE v6.00 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v2.00 or newer (https://www.microchip.com/xc)

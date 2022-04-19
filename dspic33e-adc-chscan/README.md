![image](../images/microchip.jpg)

## ADC Channel Scanning 

## Description:

In this example, Timer 3 is setup to time-out every 125 microseconds (8Khz Rate). 
As a result, the module will stop sampling and trigger a A/D conversion on every Timer3 time-out, i.e., Ts=125us. 

ADC is configured in 10bit mode to sequentially scan AIN4, AIN5, AIN10, AIN13 inputs on Timer 3 interrupt. 
It will take FOUR Timer3 Timeout period to scan through all the FOUR Analog inputs.

ADC module clock time period is configured as Tad=Tcy*(ADCS+1)= (1/40M)*64 = 1.6us (625Khz). 
Hence the conversion time for 10-bit A/D Conversion Time Tc=12*Tad = 19.2us

DMA is used to sort and transfer the converted data to DMA RAM. DMA is configured in ping-pong mode 
and it transfers 8samples of each of the FOUR analog inputs and generates interrupt. 


DMA channel 0 is configured in ping-pong mode to move the converted data from ADC to DMA RAM on every sample/convert sequence. 
First, DMA uses DMA0STA base address to store the ADC samples and it generates interrupt 
after transferring (4 x 8 samples = 32 samples).
Next, DMA uses DMA0STB base address to store the ADC samples and it generates interrupt
after transfer (4 x 8 samples = 31 samples).
Above process repeats continuously. 

void \_\_attribute\_\_((\_\_interrupt\_\_)) _DMA0Interrupt(void);
DMA interrupt service routine, moves the data from DMA buffer to ADC signal buffer 


ISR rate will be (8k/32samples) = 250Hz for 8K ADC trigger rate from Timer. 
RA4/RA6 pin is toggled in ISR, hence it will be toggling at ~ 187Hz where device operating frequency is at 60Mhz.



## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) or dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1) or dsPIC33EP256GP506 PIM (https://www.microchip.com/MA330030)	
	
	
## Software Used 

- MPLAB® X IDE v6.00 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v2.00 or newer (https://www.microchip.com/xc)


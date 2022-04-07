![image](../images/microchip.jpg)

##  ADC Sampling at 1.1MSPS 

## Description:

In this example, ADC is set up to convert AIN0 using CH0 and CH1 sample/hold in 10-bit sequential mode 
at 1.1MHz throughput rate. ADC clock is configured at 13.3Mhz or Tad=75ns
ADC Conversion Time for 10-bit conversion is Tc=12 * Tab =  900ns (1.1MHz).

void initAdc1(void);<br />
ADC CH0 and CH1 S/H is set-up to covert AIN0 in 10-bit mode. ADC is configured to next sample data immediately after the conversion.
So, ADC keeps conversion data through CH0/CH1 S/H alternatively. Effective conversion rate is 1.1Mhz

void initDma0(void);<br />
DMA channel 0 is configured in ping-pong mode to move the converted data from ADC to DMA RAM on every sample/convert sequence. 
It generates interrupt after every 16 sample transfer. 

void \_\_attribute\_\_((\_\_interrupt\_\_)) _DMA0Interrupt(void);<br />
DMA interrupt service routine, moves the data from DMA buffer to ADC signal buffer and collects 256 samples.

The Toggle frequency of one pulse should be around 240us(micro second), if the operating clock frequency at 40Mhz.
Short AN0/AN1 with +3.3v to get analog signal for sampling. These values should be approximately around 0x7FXXX when checked in the bufferA/bufferB in the code.


## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) or dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1) or dsPIC33EP256GP506 PIM (https://www.microchip.com/MA330030)
	
	
## Software Used 

- MPLAB® X IDE v6.00 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v2.00 or newer (https://www.microchip.com/xc)


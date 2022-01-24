![image](../images/microchip.jpg)

## ADC Sampling and FIR Filtering 

## Description:

In this example, ADC is configured to sample (AN5) at 250 KHz rate and the converted data is assembled in a 480-sample buffer. This input data is then filtered using the block FIR filter function from the DSP library. A 20-tap filter is used.
Timer 3 is setup to time-out every 4 microseconds (250 KHz rate). On every Timer3 time-out (every Ts = 4 microsecs), the ADC module will stop sampling and trigger a 10-bit A/D conversion. At that time, the conversion process starts and completes Tc = 12 * Tad = 1.2 microsecs later. 
When the conversion is complete, the module starts sampling again. 
However, since Timer3 is already on and counting, about (Ts-Tc) secs later Timer3 will expire again and trigger the next conversion. 
The DMA is configured in continuous, ping pong mode, such that after the DMA channel has read 480 samples into a buffer (BufferA/BufferB) a DMA interrupt is generated. 
These samples are filtered by a function call in the main function while the DMA controller starts filling new ADC samples into buffer (BufferB/BufferA). Thus the two buffers are alternately filled and processed in an infinite loop.
The ADC module clock time period is configured as Tad = Tcy * (ADCS+1) = (1/40M) * 1 = 100ns nanosecs with ADCS = 3. Hence the conversion time for 10-bit A/D is 12 * Tad = 1.2 microsecs.
FIRStruct describes the data structure for FIR filter with the filter specifications given below. FIRStructInit() initializes the FIR filter structure parameters. 
FIRDelayInit() initializes the delay values in the FIR filter structure to zeros. The FIR() function applies an FIR filter to a sequence of source samples and places the result in a sequence of destination samples.

FIR filter specifications used:
Sampling freq = 250 KHz
FIR block size, N = 480
Number of FIR coefficients, M = 20
FCY = 40 MIPS
Passband Frequency = 1300 Hz
Stopband Frequency = 1350 Hz
Passband Ripple = 1 dB
Stopband Ripple = 3 dB
Kaiser Windowing

The FIR() function takes [53+N(4+M)] instruction cycles. In this example, since N = 480 and M = 20, the total number of instruction cycles taken by FIR() is C = 11,573. Since the instruction cycle frequency is FCY = 40 MIPS the total time taken by the FIR filter to filter 480 samples is TFIR = C/FCY = 0.3 millisecs.

NOTE: A NOP associated with Y memory errata was removed from the fir.s source in the DSP library before using it in this code example, as this errata item only applies to certain dsPIC30F devices and does not affect the dsPIC33E device family.


void initTmr3();
Timer 3 is configured to time-out at 250 KHz rate. 

void initAdc1(void);
ADC module is set-up to convert AIN5 input using CH0 S/H on Timer 3 event in 10-bit mode.

void initDma0(void);
DMA channel 0 is confiured in ping-pong mode to move the converted data from ADC to DMA RAM on every sample/convert sequence. 
It generates interrupt after every 480 sample transfer. 

void __attribute__((__interrupt__)) _DMA0Interrupt(void);
DMA interrupt service routine sets flag for FIR filtering on the data buffer.


## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dspic33ep512gm710(https://www.microchip.com/ma330035) /dspic33ep512mu810 (https://www.microchip.com/MA330025-1)/ dspic33ep256gp506 (https://www.microchip.com/MA330030) PIM
	
	
## Software Used 

- MPLAB® X IDE v5.50 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v1.70 or newer (https://www.microchip.com/xc)


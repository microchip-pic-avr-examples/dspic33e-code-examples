![image](../images/microchip.jpg)

## SPI LOOP-BACK 

## Description:

In this code examples, 2x16=32words is transmitted using SPI and received back in ping-pong mode. 
This operation happens continuously.

Note: RG7 pin should be connected to RG8 externally (Depending on the PIN mapping, accordingly the it varies.Pls refer corresponding PIM sheet respectively)<br/>
For dspic33ep512mu810 :RG7 pin should be connected to RG8 externally .<br/>
For dspic33ep256gp506 :RF7 pin should be connected to RF8 externally .<br/>
For dspic33ep512gm710 :RF7 pin should be connected to RF8 externally .<br/>

void CfgSpi1Master(void)/void CfgSpi2Master(void)<br/>
This function configures SPI in master mode to transmit/receive 16-bit word.

void initSPIBuff(void)<br/>
This function pre-initialise the transmit data buffer and DMA RAM buffer for transmission

void cfgDma0SpiTx(void)<br/>
This function configures DMA channel 0 for SPI transmission. DMA is configured in ping-pong mode 
with auto increment addressing for DMA memory read.

void cfgDma1SpiRx(void)<br/>
This function configures DMA channel 0 for SPI reception. DMA is configured in ping-pong mode 
with auto increment addressing for DMA memory write.

void __attribute__((__interrupt__)) _DMA0Interrupt(void)<br/>
This interrupt routine handles transmit DMA interrupt

void __attribute__((__interrupt__)) _DMA1Interrupt(void)<br/>
This interrupt routine handles the receive ping-pong buffer.


## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) or dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1) or dsPIC33EP256GP506 PIM (https://www.microchip.com/MA330030)	
	
	
## Software Used 

- MPLAB® X IDE v6.00 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v2.00 or newer (https://www.microchip.com/xc)


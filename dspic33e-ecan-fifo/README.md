![image](../images/microchip.jpg)

## Transmitting multiple messages from CAN1 and receiving it in CAN2 FIFO 

## Description:

Transmitting 6 messages from ECAN1 and receiving it in ECAN2 FIFO  
Transmitting 6 messages from ECAN2 and receiving it in ECAN1 FIFO

32 Buffers are defined for ECAN1 and ECAN2

Six message buffers are configured for ECAN1. All are configured as a Transmit Buffers to generate Data Frames.
The reception of messages from ECAN2 is in FIFO.

Six message buffers are configured for ECAN2. All are configured as a Transmit Buffers to generate Data Frames.
The reception of messages from ECAN1 is in FIFO.

Filters and Masks are enabled for both ECAN1 and ECAN2 for Message Reception in FIFO area.

In this code example, the ECAN1 and ECAN2 modules are configured for operation at 1 Mbps over the CAN bus.
This is done in the main.c file.

The configuration of the two ECAN modules is done in the ECAN1Config.c and ECAN2Config.c files.

The number of message buffers to be allocated in DMA RAM are configured by using the following definitions
in the ECAN1Config.h and ECAN2Config.h file.

#define  ECAN1_MSG_BUF_LENGTH  
#define  ECAN2_MSG_BUF_LENGTH

The alignment attribute is used to make sure that the DMA allocates the base address for the peripheral
correctly and assigns an offset based on this base address. This is essential when using peripheral indirect
addressing mode of DMA, where the effective address is computed based on the base address partially and an 
address generated by the peripheral. (in this case ECAN).

\_\_attribute\_\_((eds,space(dma),aligned(Number of bytes)));

The number of bytes may be decided based on the number of buffers being configured by the definition made earlier.
This is calculated as follows -

Number of bytes = (ECAN1_MSG_BUF_LENGTH * Number of words in one buffer)*2 and   
Number of bytes = (ECAN2_MSG_BUF_LENGTH * Number of words in one buffer)*2.

For eg : If 4 message buffers are defined as

#define  ECAN1_MSG_BUF_LENGTH 4, then,  
Number of bytes = 4 * 8 * 2 = 64 (because Number of words in one buffer = 8 for ECAN).

The information on transmit message identifiers and other bits are written as arguments into these functions


ecan1TransmitMsg(unsigned int buf, long txIdentifier, unsigned int ide, unsigned int remoteTransmit, unsigned int srr, unsigned int dataLength );  
ecan1TxData(unsigned int buf, unsigned int data1, unsigned int data2, unsigned int data3, unsigned int data4);  
ecan2TransmitMsg(unsigned int buf, long txIdentifier, unsigned int ide, unsigned int remoteTransmit, unsigned int srr, unsigned int dataLength );  
ecan2TxData(unsigned int buf, unsigned int data1, unsigned int data2, unsigned int data3, unsigned int data4);  

The information on message acceptance filters and masks are written as arguments into these function

ecan1Filter(int n, long identifier, unsigned int exide,unsigned int bufPnt,unsigned int maskSel);  
ecan1Mask(int m, long identifierMask, unsigned int mide);  
ecan2Filter(int n, long identifier, unsigned int exide,unsigned int bufPnt,unsigned int maskSel);  
ecan2Mask(int m, long identifierMask, unsigned int mide);  

Note :- The PPS configuration in the ecan1_config.c and ecan2_config.c source files change with the device being used. THe user is advised
to refer the datasheet and use the appropriate values for RPINR/RPOR registers for proper operation.

## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) or dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1)
	
## Software Used 

- MPLAB� X IDE v6.00 or newer (https://www.microchip.com/mplabx)
- MPLAB� XC16 v2.00 or newer (https://www.microchip.com/xc)


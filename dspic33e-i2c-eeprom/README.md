![image](../images/microchip.jpg)

## I2C SERIAL EEPROM INTERFACE 

## Description:

In this code examples, 10bytes of data is written to I2C serial EEPROM and 
then read back using I2C peripheral. 

I2C EEPROM driver module takes two inputs viz., Command and Data Object
Driver supports two commands viz., Read and Write 

// EEPROM DRIVER Module
typedef struct { 
        unsigned int	cmd; 		// Command Input
	I2CEMEM_DATA	*oData;	       	// I2C Serial EEPROM Data Object
        void (*init)(void *);                   
        void (*tick)(void *); 
        }I2CEMEM_DRV; 

I2C Serial EEPROM data object contains EEPROM address location, data buffer, size
of the data buffer and chip select bits for device addressing
// Data Object
typedef struct { 
        unsigned int *buff;     	// Data Buffer 
        unsigned int n;        		// Size of the Data Buffer
        unsigned int addr;       	// EEPROM Address
        unsigned int csel;            	// Chip Select bits (A2,A1,A0 bits)
}I2CEMEM_DATA; 


* I2C Serial EEPROM read/write operation begins with transmitting control byte first. 
This control byte contains 8bits as shown below
 ------------------------------------
| 1 | 0 | 1 | 0 | A2 | A1 | A0 | R/W |
 ------------------------------------

1010 is the code used for I2C Serial EEPROM peripheral and A2,A1,A0 is used for chip select (csel).


* After the control byte, address of serial EEPROM is sent for read/write operation
Small memory I2C EEPROM will use 1byte address and large memory I2C EEPROM will need 2byte addressing. 
User must select either 1byte memory address or 2byte memory address using 
i2cEmem.h file. 

// EEPROM ADDRESS SIZE
#define ADDRWIDTH   TWO_BYTE    


## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) or dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1) or dsPIC33EP256GP506 PIM (https://www.microchip.com/MA330030)	
	
	
## Software Used 

- MPLAB® X IDE v5.50 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v1.70 or newer (https://www.microchip.com/xc)


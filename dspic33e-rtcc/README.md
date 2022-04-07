![image](../images/microchip.jpg)

## RTCC 

## Description:

This code example aims to demonstrate the basic initialisation and operation of the Real Time Clock and Calender (RTCC) module.
The user has to ensure the presence of a 32.768kHz secondary oscillator for the module to function as desired.

void RTCCUnlock(void)<br/>
This function enables the time and date value registers to be written

void RtccInit(void)<br/>
This function initialises the time and date registers to the user defined values

void RtccRead(void)<br/>
This function enables the time and date registers to be read

void \_\_attribute\_\_((interrupt, no_auto_psv)) _RTCCInterrupt(void)<br/>
This is the Interrupt Service Routine of the RTCC module

User selected bit fields:
-------------------------

I]ALARM CONFIGURATIONS:

1. CHIME      - Enabled/Disabled
2. AMASK<3:0> - Alarm Mask Configuration
3. ARPT<7:0>  - Alarm Repeat Counter Value

II]RTCC VALUE REGISTERS:

1. RTCVAL

III]ALARM VALUE REGISTERS:

1. ALRMVAL

IV]RTCC CALIBRATION:

1.CAL<7:0>    - Calibration is provided to compensate the nominal crystal frequency and for variations in the
                external crystal frequency over temperature.

                Slow clock: Add counts
	        Fast clock: Subtract counts  


## Hardware Used

- Explorer 16/32 Development Board (https://www.microchip.com/DM240001-2)
- dsPIC33EP512GM710 PIM (https://www.microchip.com/ma330035) or dsPIC33EP512MU810 PIM (https://www.microchip.com/MA330025-1) 
	
## Software Used 

- MPLAB® X IDE v6.00 or newer (https://www.microchip.com/mplabx)
- MPLAB® XC16 v2.00 or newer (https://www.microchip.com/xc)


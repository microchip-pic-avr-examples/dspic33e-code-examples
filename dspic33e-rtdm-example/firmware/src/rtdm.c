/*******************************************************************************
  Company:
    Microchip Technology Inc.

  File Name:
    rtdm.c

  Summary:
    This file is used to configure RTDM and has RTDM routines

  Description:
    This program along with MPLAB DMCI create an alternative link 
    between Host PC and target device for debugging applications in real-time. 
    It is required to include the RTDM.C file and RTDM.h into the application project 
    in order to send/receive data through the UART to/from the host PC running under 
    MPLAB X DMCI environment. 
*******************************************************************************/
/*******************************************************************************
Copyright (c) 2012 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*******************************************************************************/
// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include "rtdm.h"

// *****************************************************************************
// *****************************************************************************
// Section: File Scope or Global Constants
// *****************************************************************************
// *****************************************************************************
/* Received data is stored in array rtdmRxBuffer  */
unsigned char       rtdmRxBuffer[RTDM_RXBUFFERSIZE];
unsigned char       *rtdmRxBufferLoLimit = rtdmRxBuffer;
unsigned char       *rtdmRxBufferHiLimit = rtdmRxBuffer + RTDM_RXBUFFERSIZE - 1;
unsigned char       *rtdmRxBufferIndex = rtdmRxBuffer;
unsigned char       *rtdmRxBufferStartMsgPointer;
unsigned char       *rtdmRxBufferEndMsgPointer;

/* Data to be transmitted using UART communication module */
const unsigned char rtdmTxdata[] = { 'R', 'T', 'D', 'M', '\0' };
const unsigned char rtdmSanityCheckOK[] = { '+', '$', 'R', 'T', 'D', 'M', '#', 0x1B, 0x86, '\0' };
const unsigned char rtdmWriteMemoryOK[] = { '+', '$', 'O', 'K', '#', 0x4C, 0x08, '\0' };
const unsigned char RTDMErrorIllegalFunction[] = { '-', '$', 'E', 0x01, '#', 0xD3, 0x6A, '\0' };
unsigned char       rtdmErrorFrame[] = { '-', '$', 'E', 0, '#', 0, 0, '\0' };

/* Temp variables used to calculate the CRC16*/
unsigned int        rtdmcrcTemp, rtdmcrcTempH, rtdmcrcTempL;

//unsigned char       rtdmPacketBuf[16];

/*Structure enclosing the rtdm flags*/
struct
{
    unsigned    MessageReceived : 1;
    unsigned    TransmitNow : 1;
    unsigned    unused : 14;
} rtdmFlags;

unsigned char   rtdmPacketBuf[16];

/* UART Configuration data */
/* Holds the value of uart config reg */
unsigned int    rtdm_Uart_Mode_Value;

/* Holds the information regarding uart TX & RX interrupt modes */
unsigned int    rtdm_Uart_Sta_Value;

#if ( RTDM_UART == 1 )

/******************************************************************************
* Function:     RTDM_Start()
*
* Output:  return 0 if no errors
*
* Overview: Here is where the RTDM code initilizes the UART to be used to
*   exchange data wiht the host PC
*
* Note:  Some processors may have more UART modules, that is why it is required to
*   specify wich UART module is going to be used by RTDM 
*******************************************************************************/
    #if defined( RTDM_UART_V2 )
int RTDM_Start( void )
{
    /********************** UART CONFIGURATIION ***************************/
    /* Turn off UART1 module */
    CloseUART1();

    /* Configure UART2 receive and transmit interrupt */
    ConfigIntUART1( UART_RX_INT_EN & (UART_RX_INT_PR0 + RTDM_UART_PRIORITY) & UART_TX_INT_DIS & UART_TX_INT_PR2 );

    /* Configure UART1 module to transmit 8 bit data with one stopbit.  */
    rtdm_Uart_Mode_Value = UART_EN & UART_IDLE_CON & UART_IrDA_DISABLE & UART_MODE_FLOW & UART_UEN_00 & UART_DIS_WAKE & UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_UXRX_IDLE_ONE & UART_BRGH_SIXTEEN & UART_NO_PAR_8BIT & UART_1STOPBIT;

    rtdm_Uart_Sta_Value = UART_INT_TX_BUF_EMPTY & UART_IrDA_POL_INV_ZERO & UART_SYNC_BREAK_DISABLED & UART_TX_ENABLE & UART_INT_RX_CHAR & UART_ADR_DETECT_DIS & UART_RX_OVERRUN_CLEAR;

    OpenUART1( rtdm_Uart_Mode_Value, rtdm_Uart_Sta_Value, RTDM_BRG );

    /************* RTDM Flags Configuration & Initial Values *****************/
    rtdmFlags.MessageReceived = 0;
    rtdmFlags.MessageReceived = 0;
    rtdmRxBufferIndex = rtdmRxBufferLoLimit;
    rtdmRxBufferStartMsgPointer = rtdmRxBufferLoLimit;
    rtdmRxBufferEndMsgPointer = rtdmRxBufferLoLimit;

    return ( 0 );
}

    #elif defined( RTDM_UART_V1 )
int RTDM_Start( void )
{
    /********************** UART CONFIGURATIION ***************************/
    /* Turn off UART1 module */
    CloseUART1();

    /* Configure UART2 receive and transmit interrupt */
    ConfigIntUART1( UART_RX_INT_EN & (UART_RX_INT_PR0 + RTDM_UART_PRIORITY) & UART_TX_INT_DIS & UART_TX_INT_PR2 );

    /* Configure UART1 module to transmit 8 bit data with one stopbit.  */
    rtdm_Uart_Mode_Value = UART_EN & UART_IDLE_CON & UART_DIS_WAKE & UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_NO_PAR_8BIT & UART_1STOPBIT;

    rtdm_Uart_Sta_Value = UART_INT_TX_BUF_EMPTY & UART_TX_PIN_NORMAL & UART_TX_ENABLE & UART_INT_RX_CHAR & UART_ADR_DETECT_DIS & UART_RX_OVERRUN_CLEAR;

    OpenUART1( rtdm_Uart_Mode_Value, rtdm_Uart_Sta_Value, RTDM_BRG );

    /************* RTDM Flags Configuration & Initial Values *****************/
    rtdmFlags.MessageReceived = 0;
    rtdmFlags.MessageReceived = 0;
    rtdmRxBufferIndex = rtdmRxBufferLoLimit;
    rtdmRxBufferStartMsgPointer = rtdmRxBufferLoLimit;
    rtdmRxBufferEndMsgPointer = rtdmRxBufferLoLimit;

    return ( 0 );
}

    #endif

/******************************************************************************
* Function:      CloseRTDM()
*
* PreCondition:  None
*
* Input:         None
*
* Output:        return 0 if no errors
*
* Overview:      Here is where the RTDM code closes the UART used to
*                exchange data wiht the host PC
*
* Note:         Some processors may have more UART modules, that is why it is
*               required to specify wich UART module is going to be used by RTDM
*******************************************************************************/
int CloseRTDM( void )
{
    int nRet = 0;
    CloseUART1();
    return ( nRet );
}

/******************************************************************************
* Function:      RTDM_ProcessMsgs()
*
* PreCondition:  None
*
* Input:         None
*
* Output:        return 0 if no errors
*
* Overview:      Here is where the RTDM code process the message received and
*                then executes the required task. These tasks are reading an
*                specified memory location, writing an specified memory location,
*                receive a communication link sanity check command, or being
*                asked for the size of the bufffers.
*
* Note:         Some processors may have more UART modules, that is why it is
*               required to specify wich UART module is going to be used by RTDM
*******************************************************************************/
int RTDM_ProcessMsgs( void )
{
    //Local pointer management variables
    unsigned long int   *rtdmpu32AddressTemp;
    unsigned char       *rtdmpucWrData;
    unsigned char       *rtdmpucRdData;
    unsigned char       *rtdmpucWrAddr;
    unsigned short      rtdmNumBytes;

    unsigned int        rtdmProcessMsgsTemp1;
    unsigned int        rtdmProcessMsgsTemp2;
    unsigned int        N;

    if( !rtdmFlags.MessageReceived )
    {
        return ( -1 );
    }

    rtdmcrcTemp = RTDM_CumulativeCrc16( rtdmRxBufferStartMsgPointer,
                                        ( unsigned int ) (rtdmRxBufferEndMsgPointer - rtdmRxBufferStartMsgPointer) + 1,
                                        0xFFFF );

    rtdmcrcTempH = ( rtdmcrcTemp & 0xFF00 ) >> 8;
    rtdmcrcTempL = rtdmcrcTemp & 0x00FF;
    rtdmProcessMsgsTemp1 = ( unsigned int ) *( (rtdmRxBufferEndMsgPointer) + 2 );
    rtdmProcessMsgsTemp2 = ( unsigned int ) *( (rtdmRxBufferEndMsgPointer) + 1 );

    rtdmRxBufferStartMsgPointer += 2;
    if( (rtdmProcessMsgsTemp1 == ( unsigned ) rtdmcrcTempH) && (rtdmProcessMsgsTemp2 == rtdmcrcTempL) )
    {
        switch( *((rtdmRxBufferLoLimit) + 1) )
        {
            case 'm':
                /*************** Extract Address **************/
                //Capture address as 32 bit quantity to match protocol definition.
                rtdmpu32AddressTemp = ( ( unsigned long * ) rtdmRxBufferStartMsgPointer );

                //Increment receive buffer pointer to length field.
                rtdmRxBufferStartMsgPointer += sizeof( unsigned long );

                //Init a byte oriented data pointer
                rtdmpucRdData = ( unsigned char * ) ( ( unsigned int ) *rtdmpu32AddressTemp );

                /********* Extract Number of Bytes ************/
                //Capture address as 16 bit quantity to match protocol definition.
                rtdmNumBytes = *( ( unsigned short * ) rtdmRxBufferStartMsgPointer );

                //Increment receive buffer pointer to start of data payload.
                rtdmRxBufferStartMsgPointer += sizeof( unsigned short );

                //Init the CRC seed for the cumulative checksum calculation.
                rtdmcrcTemp = 0xffff;

                //Add packet header prefix
                rtdmPacketBuf[0] = '+';
                rtdmPacketBuf[1] = '$';

                //Add null terminator for putsUARTx function...
                rtdmPacketBuf[2] = 0;

                //Calc header prefix checksum piece
                rtdmcrcTemp = RTDM_CumulativeCrc16( rtdmPacketBuf, 2, rtdmcrcTemp );

                //Calc data payload checksum
                rtdmcrcTemp = RTDM_CumulativeCrc16( rtdmpucRdData, rtdmNumBytes, rtdmcrcTemp );

                //Send packet header. Use string function to save code space...
                putsUART1( ( unsigned int * ) rtdmPacketBuf );
                while( BusyUART1() );

                //Send data portion of message...
                while( rtdmNumBytes-- )
                {
                    WriteUART1( *rtdmpucRdData++ );
                    while( BusyUART1() );
                }

                //Add packet trailer
                rtdmPacketBuf[0] = '#';
                rtdmcrcTemp = RTDM_CumulativeCrc16( rtdmPacketBuf, 1, rtdmcrcTemp );

                //Add checksum bytes to packet
                rtdmPacketBuf[1] = rtdmcrcTemp & 0x00FF;
                rtdmPacketBuf[2] = ( rtdmcrcTemp & 0xFF00 ) >> 8;

                //Send packet trailer and checksum.
                for( N = 0; N < 3; N++ )
                {
                    WriteUART1( rtdmPacketBuf[N] );
                    while( BusyUART1() );
                }

                break;

            case 'M':
                {
                    /*************** Extract Address **************/
                    //Capture address as 32 bit quantity to match protocol definition.
                    rtdmpu32AddressTemp = ( unsigned long * ) rtdmRxBufferStartMsgPointer;

                    //Increment receive buffer pointer to length field.
                    rtdmRxBufferStartMsgPointer += sizeof( unsigned long );

                    // Init a byte oriented address pointer for use in incrementing
                    //through the address range properly as we write each byte of data
                    //in the range (length) of this write request.
                    rtdmpucWrAddr = ( unsigned char * ) ( ( unsigned int ) *rtdmpu32AddressTemp );

                    /********* Extract Number of Bytes ************/
                    //MEllis Capture length as 16 bit quantity to match protocol definition.
                    rtdmNumBytes = *( ( unsigned short * ) rtdmRxBufferStartMsgPointer );

                    //MEllis Increment receive buffer pointer to start of data payload.
                    rtdmRxBufferStartMsgPointer += sizeof( unsigned short );

                    /********** Extract Data ************/
                    //Init a byte oriented data pointer so that we can increment a byte at at
                    //time for as many bytes as are in the range for this write.
                    rtdmpucWrData = rtdmRxBufferStartMsgPointer;

                    //*** Write Data in specified RAM location *****
                    //Important to increment through address range using byte oriented address and data
                    //pointers. Otherwise, single byte or odd byte ranges do not get written correctly.
                    while( rtdmNumBytes-- )
                    {
                        *rtdmpucWrAddr++ = *rtdmpucWrData++;
                    }

                    //Transmit OK message
                    putsUART1( ( unsigned int * ) rtdmWriteMemoryOK );

                    /* Wait for  transmission to complete */
                    while( BusyUART1() );
                    break;
                }

            case 's':
                {
                    /* Load transmit buffer and transmit the same till null character is encountered */
                    //Transmit OK message
                    putsUART1( ( unsigned int * ) rtdmSanityCheckOK );

                    /* Wait for  transmission to complete */
                    while( BusyUART1() );
                    break;
                }

            case 'L':
                rtdmcrcTemp = 0xffff;   //Init the CRC seed.
                rtdmPacketBuf[0] = '+';
                rtdmPacketBuf[1] = '$';

                //Size of the rtdm Receive buffer.
                rtdmPacketBuf[2] = ( sizeof(rtdmRxBuffer) & 0x00FF );
                rtdmPacketBuf[3] = ( sizeof(rtdmRxBuffer) & 0xFF00 ) >> 8;

                //Note: We dod not utilize a transmit buffer since any data memory source is
                //essentially already buffered. So the transmit limit is now just a way to
                //limit the total message length that a client make with any single read request.
                rtdmPacketBuf[4] = ( RTDM_MAX_XMIT_LEN & 0x00FF );
                rtdmPacketBuf[5] = ( RTDM_MAX_XMIT_LEN & 0xFF00 ) >> 8;
                rtdmPacketBuf[6] = '#';
                rtdmcrcTemp = RTDM_CumulativeCrc16( rtdmPacketBuf, 7, rtdmcrcTemp );
                rtdmPacketBuf[7] = ( rtdmcrcTemp & 0x00FF );
                rtdmPacketBuf[8] = ( rtdmcrcTemp & 0xFF00 ) >> 8;

                //Send completed message which is 9 bytes in length.
                for( N = 0; N < 9; N++ )
                {
                    WriteUART1( rtdmPacketBuf[N] );
                    while( BusyUART1() );
                }

                break;

            default:
                // ---> COMMAND SUPPORTED?? IF NOT ERROR HANDLER
                //Transmit ERROR message 1
                putsUART1( ( unsigned int * ) rtdmErrorIllegalFunction );

                /* Wait for  transmission to complete */
                while( BusyUART1() );
                break;
        }
    }

    memset( &rtdmRxBuffer, 0, sizeof(rtdmRxBuffer) );

    rtdmFlags.MessageReceived = 0;
    rtdmRxBufferIndex = rtdmRxBufferLoLimit;
    rtdmRxBufferStartMsgPointer = rtdmRxBufferLoLimit;
    rtdmRxBufferEndMsgPointer = rtdmRxBufferLoLimit;

    return ( 0 );
}

/******************************************************************************
* Function:      _U1RXInterrupt(void)
*
* Output:  void
*
* Overview: Here is where the rtdm receives the messages using the UART receiver
*   interrupt, If polling method is selected in the RTDMUSER.h file then
*   the user application should call the RTDM_ProcessMsgs routine in order 
*   to precess up comming messages. If polling method is disabled then the 
*   RTDM_ProcessMsgs routine is called in the UART received interrupt
*   routine.
*
* Note:  Some processors may have more UART modules, that is why it is required to
*   specify wich UART module is going to be used by RTDM 
*******************************************************************************/
    #if ( RTDM_POLLING == YES )

/* This is UART1 receive ISR Polling RTDM Messages*/
void __attribute__ ( (__interrupt__, no_auto_psv) ) _U1RXInterrupt( void )
{
    _U1RXIF = 0;

    /* Read the receive buffer until at least one or more character can be read */
    while( DataRdyUART1() )
    {
        *( rtdmRxBufferIndex++ ) = ReadUART1();
    }

    rtdmRxBufferEndMsgPointer = rtdmRxBufferIndex - 3;
    if( rtdmRxBufferIndex > (rtdmRxBufferHiLimit - 1) )
    {
        rtdmRxBufferIndex = rtdmRxBufferLoLimit;
        rtdmRxBufferEndMsgPointer = rtdmRxBufferHiLimit - 1;
    }

    if( *(rtdmRxBufferStartMsgPointer) == '$' )
    {
        if( *(rtdmRxBufferEndMsgPointer) == '#' )
        {
            rtdmFlags.MessageReceived = 1;
        }
    }
    else
    {
        rtdmRxBufferIndex = rtdmRxBufferLoLimit;
    }
}

/******************************************************************************
* Function:      _U1RXInterrupt(void)
*
* Output:  void
*
* Overview: Here is where the RTDM receives the messages using the UART receiver 
*   interrupt, If polling method is selected in the RTDMUSER.h file then
*   the user application should call the RTDM_ProcessMsgs routine in order 
*   to precess up comming messages. If polling method is disabled then the 
*   RTDM_ProcessMsgs routine is called in the UART received interrupt
*   routine.
*
* Note:  Some processors may have more UART modules, that is why it is required to
*   specify wich UART module is going to be used by RTDM 
*******************************************************************************/
    #else

/* This is UART1 receive ISR without polling RTDM Messages */
void __attribute__ ( (__interrupt__, no_auto_psv) ) _U1RXInterrupt( void )
{
    _U1RXIF = 0;

    /* Read the receive buffer until at least one or more character can be read */
    while( DataRdyUART1() )
    {
        *( rtdmRxBufferIndex++ ) = ReadUART1();
    }

    rtdmRxBufferEndMsgPointer = rtdmRxBufferIndex - 3;
    if( rtdmRxBufferIndex > (rtdmRxBufferHiLimit - 1) )
    {
        rtdmRxBufferIndex = rtdmRxBufferLoLimit;
        rtdmRxBufferEndMsgPointer = rtdmRxBufferHiLimit - 1;
    }

    if( *(rtdmRxBufferStartMsgPointer) == '$' )
    {
        if( *(rtdmRxBufferEndMsgPointer) == '#' )
        {
            rtdmFlags.MessageReceived = 1;
            RTDM_ProcessMsgs();
        }
    }
    else
    {
        rtdmRxBufferIndex = rtdmRxBufferLoLimit;
    }
}

    #endif
#elif ( RTDM_UART == 2 )

/******************************************************************************
* Function:     int RTDM_Start()
*
* PreCondition: None
*
* Input:        None
*
* Output:       return 0 if no errors
*
* Overview:     Here is where the RTDM code initilizes the UART to be used to
*               exchange data wiht the host PC
*
* Note:         Some processors may have more UART modules, that is why it is
*               required to specify wich UART module is going to be used by RTDM
*******************************************************************************/
    #if defined( RTDM_UART_V2 )
int RTDM_Start( void )
{
    /********************** UART CONFIGURATIION ******************************/
    /* Turn off UART2 module */
    CloseUART2();

    /* Configure UART2 receive and transmit interrupt */
    ConfigIntUART2( UART_RX_INT_EN & (UART_RX_INT_PR0 + RTDM_UART_PRIORITY) & UART_TX_INT_DIS & UART_TX_INT_PR2 );

    /* Configure UART2 module to transmit 8 bit data with one stopbit.  */
    rtdm_Uart_Mode_Value = UART_EN & UART_IDLE_CON & UART_IrDA_DISABLE & UART_MODE_FLOW & UART_UEN_00 & UART_DIS_WAKE & UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_UXRX_IDLE_ONE & UART_BRGH_SIXTEEN & UART_NO_PAR_8BIT & UART_1STOPBIT;

    rtdm_Uart_Sta_Value = UART_INT_TX_BUF_EMPTY & UART_IrDA_POL_INV_ZERO & UART_SYNC_BREAK_DISABLED & UART_TX_ENABLE & UART_INT_RX_CHAR & UART_ADR_DETECT_DIS & UART_RX_OVERRUN_CLEAR;

    OpenUART2( rtdm_Uart_Mode_Value, rtdm_Uart_Sta_Value, RTDM_BRG );

    /************* RTDM Flags Configuration & Initial Values *****************/
    rtdmFlags.MessageReceived = 0;
    rtdmFlags.MessageReceived = 0;
    rtdmRxBufferIndex = rtdmRxBufferLoLimit;
    rtdmRxBufferStartMsgPointer = rtdmRxBufferLoLimit;
    rtdmRxBufferEndMsgPointer = rtdmRxBufferLoLimit;

    return ( 0 );
}

    #elif defined( rtdm_UART_V1 )
int RTDM_Start( void )
{
    /********************** UART CONFIGURATIION ******************************/
    /* Turn off UART2 module */
    CloseUART2();

    /* Configure UART2 receive and transmit interrupt */
    ConfigIntUART2( UART_RX_INT_EN & (UART_RX_INT_PR0 + RTDM_UART_PRIORITY) & UART_TX_INT_DIS & UART_TX_INT_PR2 );

    /* Configure UART2 module to transmit 8 bit data with one stopbit.  */
    rtdm_Uart_Mode_Value = UART_EN & UART_IDLE_CON & UART_DIS_WAKE & UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_NO_PAR_8BIT & UART_1STOPBIT;

    rtdm_Uart_Sta_Value = UART_INT_TX_BUF_EMPTY & UART_TX_PIN_NORMAL & UART_TX_ENABLE & UART_INT_RX_CHAR & UART_ADR_DETECT_DIS & UART_RX_OVERRUN_CLEAR;

    OpenUART2( rtdm_Uart_Mode_Value, rtdm_Uart_Sta_Value, RTDM_BRG );

    /************* RTDM Flags Configuration & Initial Values *****************/
    rtdmFlags.MessageReceived = 0;
    rtdmFlags.MessageReceived = 0;
    rtdmRxBufferIndex = rtdmRxBufferLoLimit;
    rtdmRxBufferStartMsgPointer = rtdmRxBufferLoLimit;
    rtdmRxBufferEndMsgPointer = rtdmRxBufferLoLimit;

    return ( 0 );
}

    #endif

/******************************************************************************
* Function:      int CloseRTDM()
*
* PreCondition:  None
*
* Input:         None
*
* Output:        return 0 if no errors
*
* Overview:      Here is where the RTDM code closes the UART used to
*                exchange data wiht the host PC
*
* Note:         Some processors may have more UART modules, that is why it is
*               required to specify wich UART module is going to be used by RTDM
*******************************************************************************/
int CloseRTDM( void )
{
    int nRet = 0;
    CloseUART2();
    return ( nRet );
}

/******************************************************************************
* Function:      int RTDM_ProcessMsgs()
*
* PreCondition:  None
*
* Input:         None
*
* Output:        return 0 if no errors
*
* Overview:      Here is where the RTDM code process the message received and
*                then executes the required task. These tasks are reading an
*                specified memory location, writing an specified memory location,
*                receive a communication link sanity check command, or being
*                asked for the size of the bufffers.
*
* Note:         Some processors may have more UART modules, that is why it is
*               required to specify wich UART module is going to be used by RTDM
*******************************************************************************/
int RTDM_ProcessMsgs( void )
{
    //Local pointer management variables
    unsigned long int   *rtdmpu32AddressTemp;
    unsigned char       *rtdmpucWrData;
    unsigned char       *rtdmpucRdData;
    unsigned char       *rtdmpucWrAddr;
    unsigned short      rtdmNumBytes;

    unsigned int        rtdmProcessMsgsTemp1;
    unsigned int        rtdmProcessMsgsTemp2;
    unsigned int        N;

    if( !rtdmFlags.MessageReceived )
    {
        return ( -1 );
    }

    rtdmcrcTemp = RTDM_CumulativeCrc16( rtdmRxBufferStartMsgPointer,
                                        ( unsigned int ) (rtdmRxBufferEndMsgPointer - rtdmRxBufferStartMsgPointer) + 1,
                                        0xFFFF );

    rtdmcrcTempH = ( rtdmcrcTemp & 0xFF00 ) >> 8;
    rtdmcrcTempL = rtdmcrcTemp & 0x00FF;
    rtdmProcessMsgsTemp1 = ( unsigned int ) *( (rtdmRxBufferEndMsgPointer) + 2 );
    rtdmProcessMsgsTemp2 = ( unsigned int ) *( (rtdmRxBufferEndMsgPointer) + 1 );

    rtdmRxBufferStartMsgPointer += 2;
    if( (rtdmProcessMsgsTemp1 == ( unsigned ) rtdmcrcTempH) && (rtdmProcessMsgsTemp2 == rtdmcrcTempL) )
    {
        switch( *((rtdmRxBufferLoLimit) + 1) )
        {
            case 'm':
                /*************** Extract Address **************/
                //Capture address as 32 bit quantity to match protocol definition.
                rtdmpu32AddressTemp = ( ( unsigned long * ) rtdmRxBufferStartMsgPointer );

                //Increment receive buffer pointer to length field.
                rtdmRxBufferStartMsgPointer += sizeof( unsigned long );

                //Init a byte oriented data pointer
                rtdmpucRdData = ( unsigned char * ) ( ( unsigned int ) *rtdmpu32AddressTemp );

                /********* Extract Number of Bytes ***********/
                //Capture address as 16 bit quantity to match protocol definition.
                rtdmNumBytes = *( ( unsigned short * ) rtdmRxBufferStartMsgPointer );

                //Increment receive buffer pointer to start of data payload.
                rtdmRxBufferStartMsgPointer += sizeof( unsigned short );

                //Init the CRC seed for the cumulative checksum calculation.
                rtdmcrcTemp = 0xffff;

                //Add packet header prefix
                rtdmPacketBuf[0] = '+';
                rtdmPacketBuf[1] = '$';

                //Add null terminator for putsUARTx function...
                rtdmPacketBuf[2] = 0;

                //Calc header prefix checksum piece
                rtdmcrcTemp = RTDM_CumulativeCrc16( rtdmPacketBuf, 2, rtdmcrcTemp );

                //Calc data payload checksum
                rtdmcrcTemp = RTDM_CumulativeCrc16( rtdmpucRdData, rtdmNumBytes, rtdmcrcTemp );

                //Send packet header. Use string function to save code space...
                putsUART2( ( unsigned int * ) (rtdmPacketBuf) );
                while( BusyUART2() );

                //Send data portion of message...
                while( rtdmNumBytes-- )
                {
                    WriteUART2( *rtdmpucRdData++ );
                    while( BusyUART2() );
                }

                //Add packet trailer
                rtdmPacketBuf[0] = '#';
                rtdmcrcTemp = RTDM_CumulativeCrc16( rtdmPacketBuf, 1, rtdmcrcTemp );

                //Add checksum bytes to packet
                rtdmPacketBuf[1] = rtdmcrcTemp & 0x00FF;
                rtdmPacketBuf[2] = ( rtdmcrcTemp & 0xFF00 ) >> 8;

                //Send packet trailer and checksum.
                for( N = 0; N < 3; N++ )
                {
                    WriteUART2( rtdmPacketBuf[N] );
                    while( BusyUART2() );
                }

                break;

            case 'M':
                {
                    /*************** Extract Address **************/
                    //Capture address as 32 bit quantity to match protocol definition.
                    rtdmpu32AddressTemp = ( unsigned long * ) rtdmRxBufferStartMsgPointer;

                    //Increment receive buffer pointer to length field.
                    rtdmRxBufferStartMsgPointer += sizeof( unsigned long );

                    //Init a byte oriented address pointer for use in incrementing
                    //through the address range properly as we write each byte of data
                    //in the range (length) of this write request.
                    rtdmpucWrAddr = ( unsigned char * ) ( ( unsigned int ) *rtdmpu32AddressTemp );

                    /********* Extract Number of Bytes ************/
                    //Capture length as 16 bit quantity to match protocol definition.
                    rtdmNumBytes = *( ( unsigned short * ) rtdmRxBufferStartMsgPointer );

                    //Increment receive buffer pointer to start of data payload.
                    rtdmRxBufferStartMsgPointer += sizeof( unsigned short );

                    /********** Extract Data ************/
                    //Init a byte oriented data pointer so that we can increment a byte at at
                    //time for as many bytes as are in the range for this write.
                    rtdmpucWrData = rtdmRxBufferStartMsgPointer;

                    //*** Write Data in specified RAM location *****
                    //Important to increment through address range using byte oriented address and data
                    //pointers. Otherwise, single byte or odd byte ranges do not get written correctly.
                    while( rtdmNumBytes-- )
                    {
                        *rtdmpucWrAddr++ = *rtdmpucWrData++;
                    }

                    //Transmit OK message
                    putsUART2( ( unsigned int * ) rtdmWriteMemoryOK );

                    /* Wait for  transmission to complete */
                    while( BusyUART2() );
                    break;
                }

            case 's':
                {
                    /* Load transmit buffer and transmit the same till null character is encountered */
                    //Transmit OK message
                    putsUART2( ( unsigned int * ) rtdmSanityCheckOK );

                    /* Wait for  transmission to complete */
                    while( BusyUART2() );
                    break;
                }

            case 'L':
                rtdmcrcTemp = 0xffff;   //Init the CRC seed.
                rtdmPacketBuf[0] = '+';
                rtdmPacketBuf[1] = '$';

                //Size of the rtdm Receive buffer.
                rtdmPacketBuf[2] = ( sizeof(rtdmRxBuffer) & 0x00FF );
                rtdmPacketBuf[3] = ( sizeof(rtdmRxBuffer) & 0xFF00 ) >> 8;

                //Note: We dod not utilize a transmit buffer since any data memory source is
                //essentially already buffered. So the transmit limit is now just a way to
                //limit the total message length that a client make with any single read request.
                rtdmPacketBuf[4] = ( RTDM_MAX_XMIT_LEN & 0x00FF );
                rtdmPacketBuf[5] = ( RTDM_MAX_XMIT_LEN & 0xFF00 ) >> 8;
                rtdmPacketBuf[6] = '#';
                rtdmcrcTemp = RTDM_CumulativeCrc16( rtdmPacketBuf, 7, rtdmcrcTemp );
                rtdmPacketBuf[7] = ( rtdmcrcTemp & 0x00FF );
                rtdmPacketBuf[8] = ( rtdmcrcTemp & 0xFF00 ) >> 8;

                //Send completed message which is 9 bytes in length.
                for( N = 0; N < 9; N++ )
                {
                    WriteUART2( rtdmPacketBuf[N] );
                    while( BusyUART2() );
                }

                break;

            default:
                // ---> COMMAND SUPPORTED?? IF NOT ERROR HANDLER
                //Transmit ERROR message 1
                putsUART2( ( unsigned int * ) RTDMErrorIllegalFunction );

                /* Wait for  transmission to complete */
                while( BusyUART2() );
                break;
        }
    }

    memset( &rtdmRxBuffer, 0, sizeof(rtdmRxBuffer) );

    rtdmFlags.MessageReceived = 0;
    rtdmRxBufferIndex = rtdmRxBufferLoLimit;
    rtdmRxBufferStartMsgPointer = rtdmRxBufferLoLimit;
    rtdmRxBufferEndMsgPointer = rtdmRxBufferLoLimit;

    return ( 0 );
}

/******************************************************************************
* Function:void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt(void)
*
* PreCondition:  None
*
* Input:         None
*
* Output:        None
*
* Overview: Here is where the rtdm receives the messages using the UART receiver
*           interrupt, If polling method is selected in the RTDMUSER.h file then
*           the user application should call the RTDM_ProcessMsgs routine in
*           order to precess up comming messages. If polling method is disabled
*           then the RTDM_ProcessMsgs routine is called in the UART received
*           interrupt routine.
*
* Note:     Some processors may have more UART modules, that is why it is
*           required to specify wich UART module is going to be used by RTDM
*******************************************************************************/
    #if ( RTDM_POLLING == YES )

/* This is UART2 receive ISR Polling RTDM Messages*/
void __attribute__ ( (__interrupt__, no_auto_psv) ) _U2RXInterrupt( void )
{
    _U2RXIF = 0;

    /* Read the receive buffer until at least one or more character can be read */
    while( DataRdyUART2() )
    {
        *( rtdmRxBufferIndex++ ) = ReadUART2();
    }

    rtdmRxBufferEndMsgPointer = rtdmRxBufferIndex - 3;
    if( rtdmRxBufferIndex > (rtdmRxBufferHiLimit - 1) )
    {
        rtdmRxBufferIndex = rtdmRxBufferLoLimit;
        rtdmRxBufferEndMsgPointer = rtdmRxBufferHiLimit - 1;
    }

    if( *(rtdmRxBufferStartMsgPointer) == '$' )
    {
        if( *(rtdmRxBufferEndMsgPointer) == '#' )
        {
            rtdmFlags.MessageReceived = 1;
        }
    }
    else
    {
        rtdmRxBufferIndex = rtdmRxBufferLoLimit;
    }
}

/******************************************************************************
* Function:      _U2RXInterrupt(void)
*
* PreCondition:  None
*
* Input:         None
*
* Output:        None
*
* Overview: Here is where the rtdm receives the messages using the UART receiver
*           interrupt, If polling method is selected in the RTDMUSER.h file then
*           the user application should call the RTDM_ProcessMsgs routine in
*           order to precess up comming messages. If polling method is disabled
*           then the RTDM_ProcessMsgs routine is called in the UART received
*           interrupt routine.
*
* Note:     Some processors may have more UART modules, that is why it is
*           required to specify wich UART module is going to be used by RTDM
*******************************************************************************/
    #else

/* This is UART2 receive ISR without polling RTDM Messages */
void __attribute__ ( (__interrupt__, no_auto_psv) ) _U2RXInterrupt( void )
{
    _U2RXIF = 0;

    /* Read the receive buffer until at least one or more character can be read */
    while( DataRdyUART2() )
    {
        *( rtdmRxBufferIndex++ ) = ReadUART2();
    }

    rtdmRxBufferEndMsgPointer = rtdmRxBufferIndex - 3;
    if( rtdmRxBufferIndex > (rtdmRxBufferHiLimit - 1) )
    {
        rtdmRxBufferIndex = rtdmRxBufferLoLimit;
        rtdmRxBufferEndMsgPointer = rtdmRxBufferHiLimit - 1;
    }

    if( *(rtdmRxBufferStartMsgPointer) == '$' )
    {
        if( *(rtdmRxBufferEndMsgPointer) == '#' )
        {
            rtdmFlags.MessageReceived = 1;
            RTDM_ProcessMsgs();
        }
    }
    else
    {
        rtdmRxBufferIndex = rtdmRxBufferLoLimit;
    }
}

    #endif
#elif ( RTDM_UART == 3 )
    #if ( defined(RTDM_UART_V2) && (defined(__dsPIC33E__) || defined(__PIC24E__)) )

/******************************************************************************
* Function:     RTDM_Start()
*
* PreCondition: None
*
* Input:        None
*
* Output:       return 0 if no errors
*
* Overview:     Here is where the RTDM code initilizes the UART to be used to
*               exchange data wiht the host PC
*
* Note:         Some processors may have more UART modules, that is why it is
*               required to specify wich UART module is going to be used by RTDM
*******************************************************************************/
int RTDM_Start( void )
{
    /********************** UART CONFIGURATIION ******************************/
    /* Turn off UART3 module */
    CloseUART3();

    /* Configure UART3 receive and transmit interrupt */
    ConfigIntUART3( UART_RX_INT_EN & (UART_RX_INT_PR0 + RTDM_UART_PRIORITY) & UART_TX_INT_DIS & UART_TX_INT_PR2 );

    /* Configure UART3 module to transmit 8 bit data with one stopbit.  */
    rtdm_Uart_Mode_Value = UART_EN & UART_IDLE_CON & UART_IrDA_DISABLE & UART_MODE_FLOW & UART_UEN_00 & UART_DIS_WAKE & UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_UXRX_IDLE_ONE & UART_BRGH_SIXTEEN & UART_NO_PAR_8BIT & UART_1STOPBIT;

    rtdm_Uart_Sta_Value = UART_INT_TX_BUF_EMPTY & UART_IrDA_POL_INV_ZERO & UART_SYNC_BREAK_DISABLED & UART_TX_ENABLE & UART_INT_RX_CHAR & UART_ADR_DETECT_DIS & UART_RX_OVERRUN_CLEAR;

    OpenUART3( rtdm_Uart_Mode_Value, rtdm_Uart_Sta_Value, RTDM_BRG );

    /************* RTDM Flags Configuration & Initial Values *****************/
    rtdmFlags.MessageReceived = 0;
    rtdmFlags.MessageReceived = 0;
    rtdmRxBufferIndex = rtdmRxBufferLoLimit;
    rtdmRxBufferStartMsgPointer = rtdmRxBufferLoLimit;
    rtdmRxBufferEndMsgPointer = rtdmRxBufferLoLimit;

    return ( 0 );
}

/******************************************************************************
* Function:      CloseRTDM()
*
* PreCondition:  None
*
* Input:         None
*
* Output:        return 0 if no errors
*
* Overview:      Here is where the RTDM code closes the UART used to
*                exchange data wiht the host PC
*
* Note:         Some processors may have more UART modules, that is why it is
*               required to specify wich UART module is going to be used by RTDM
*******************************************************************************/
int CloseRTDM( void )
{
    int nRet = 0;
    CloseUART3();
    return ( nRet );
}

/******************************************************************************
* Function:      RTDM_ProcessMsgs()
*
* PreCondition:  None
*
* Input:         None
*
* Output:        return 0 if no errors
*
* Overview:      Here is where the RTDM code process the message received and
*                then executes the required task. These tasks are reading an
*                specified memory location, writing an specified memory location,
*                receive a communication link sanity check command, or being
*                asked for the size of the bufffers.
*
* Note:         Some processors may have more UART modules, that is why it is
*               required to specify wich UART module is going to be used by RTDM
*******************************************************************************/
int RTDM_ProcessMsgs( void )
{
    //Local pointer management variables
    unsigned long int   *RTDMpu32AddressTemp;
    unsigned char       *rtdmpucWrData;
    unsigned char       *rtdmpucRdData;
    unsigned char       *rtdmpucWrAddr;
    unsigned short      rtdmNumBytes;

    unsigned int        rtdmProcessMsgsTemp1;
    unsigned int        rtdmProcessMsgsTemp2;
    unsigned int        N;

    if( !rtdmFlags.MessageReceived )
    {
        return ( -1 );
    }

    rtdmcrcTemp = RTDM_CumulativeCrc16( rtdmRxBufferStartMsgPointer,
                                        ( unsigned int ) (rtdmRxBufferEndMsgPointer - rtdmRxBufferStartMsgPointer) + 1,
                                        0xFFFF );

    rtdmcrcTempH = ( rtdmcrcTemp & 0xFF00 ) >> 8;
    rtdmcrcTempL = rtdmcrcTemp & 0x00FF;
    rtdmProcessMsgsTemp1 = ( unsigned int ) *( (rtdmRxBufferEndMsgPointer) + 2 );
    rtdmProcessMsgsTemp2 = ( unsigned int ) *( (rtdmRxBufferEndMsgPointer) + 1 );

    rtdmRxBufferStartMsgPointer += 2;
    if( (rtdmProcessMsgsTemp1 == ( unsigned ) rtdmcrcTempH) && (rtdmProcessMsgsTemp2 == rtdmcrcTempL) )
    {
        switch( *((rtdmRxBufferLoLimit) + 1) )
        {
            case 'm':
                /*************** Extract Address **************/
                //Capture address as 32 bit quantity to match protocol definition.
                rtdmpu32AddressTemp = ( ( unsigned long * ) rtdmRxBufferStartMsgPointer );

                //Increment receive buffer pointer to length field.
                rtdmRxBufferStartMsgPointer += sizeof( unsigned long );

                //Init a byte oriented data pointer
                rtdmpucRdData = ( unsigned char * ) ( ( unsigned int ) *rtdmpu32AddressTemp );

                /********* Extract Number of Bytes ***********/
                //Capture address as 16 bit quantity to match protocol definition.
                rtdmNumBytes = *( ( unsigned short * ) rtdmRxBufferStartMsgPointer );

                //Increment receive buffer pointer to start of data payload.
                rtdmRxBufferStartMsgPointer += sizeof( unsigned short );

                //Init the CRC seed for the cumulative checksum calculation.
                rtdmcrcTemp = 0xffff;

                //Add packet header prefix
                rtdmPacketBuf[0] = '+';
                rtdmPacketBuf[1] = '$';

                //Add null terminator for putsUARTx function...
                rtdmPacketBuf[2] = 0;

                //Calc header prefix checksum piece
                rtdmcrcTemp = RTDM_CumulativeCrc16( rtdmPacketBuf, 2, rtdmcrcTemp );

                //Calc data payload checksum
                rtdmcrcTemp = RTDM_CumulativeCrc16( rtdmpucRdData, rtdmNumBytes, rtdmcrcTemp );

                //Send packet header. Use string function to save code space...
                putsUART3( ( unsigned int * ) rtdmPacketBuf );
                while( BusyUART3() );

                //Send data portion of message...
                while( rtdmNumBytes-- )
                {
                    WriteUART3( *rtdmpucRdData++ );
                    while( BusyUART3() );
                }

                //Add packet trailer
                rtdmPacketBuf[0] = '#';
                rtdmcrcTemp = RTDM_CumulativeCrc16( rtdmPacketBuf, 1, rtdmcrcTemp );

                //Add checksum bytes to packet
                rtdmPacketBuf[1] = rtdmcrcTemp & 0x00FF;
                rtdmPacketBuf[2] = ( rtdmcrcTemp & 0xFF00 ) >> 8;

                //Send packet trailer and checksum.
                for( N = 0; N < 3; N++ )
                {
                    WriteUART3( rtdmPacketBuf[N] );
                    while( BusyUART3() );
                }

                break;

            case 'M':
                {
                    /*************** Extract Address **************/
                    //Capture address as 32 bit quantity to match protocol definition.
                    rtdmpu32AddressTemp = ( unsigned long * ) rtdmRxBufferStartMsgPointer;

                    //Increment receive buffer pointer to length field.
                    rtdmRxBufferStartMsgPointer += sizeof( unsigned long );

                    //Init a byte oriented address pointer for use in incrementing
                    //through the address range properly as we write each byte of data
                    //in the range (length) of this write request.
                    rtdmpucWrAddr = ( unsigned char * ) ( ( unsigned int ) *rtdmpu32AddressTemp );

                    /********* Extract Number of Bytes ************/
                    //Capture length as 16 bit quantity to match protocol definition.
                    rtdmNumBytes = *( ( unsigned short * ) rtdmRxBufferStartMsgPointer );

                    //Increment receive buffer pointer to start of data payload.
                    rtdmRxBufferStartMsgPointer += sizeof( unsigned short );

                    /********** Extract Data ************/
                    //Init a byte oriented data pointer so that we can increment a byte at at
                    //time for as many bytes as are in the range for this write.
                    rtdmpucWrData = rtdmRxBufferStartMsgPointer;

                    //*** Write Data in specified RAM location *****
                    //Important to increment through address range using byte oriented address and data
                    //pointers. Otherwise, single byte or odd byte ranges do not get written correctly.
                    while( rtdmNumBytes-- )
                    {
                        *rtdmpucWrAddr++ = *rtdmpucWrData++;
                    }

                    //Transmit OK message
                    putsUART3( ( unsigned int * ) rtdmWriteMemoryOK );

                    /* Wait for  transmission to complete */
                    while( BusyUART3() );
                    break;
                }

            case 's':
                {
                    /* Load transmit buffer and transmit the same till null character is encountered */
                    //Transmit OK message
                    putsUART3( ( unsigned int * ) rtdmSanityCheckOK );

                    /* Wait for  transmission to complete */
                    while( BusyUART3() );
                    break;
                }

            case 'L':
                rtdmcrcTemp = 0xffff;   //Init the CRC seed.
                rtdmPacketBuf[0] = '+';
                rtdmPacketBuf[1] = '$';

                //Size of the rtdm Receive buffer.
                rtdmPacketBuf[2] = ( sizeof(rtdmRxBuffer) & 0x00FF );
                rtdmPacketBuf[3] = ( sizeof(rtdmRxBuffer) & 0xFF00 ) >> 8;

                //Note: We dod not utilize a transmit buffer since any data memory source is
                //essentially already buffered. So the transmit limit is now just a way to
                //limit the total message length that a client make with any single read request.
                rtdmPacketBuf[4] = ( RTDM_MAX_XMIT_LEN & 0x00FF );
                rtdmPacketBuf[5] = ( RTDM_MAX_XMIT_LEN & 0xFF00 ) >> 8;
                rtdmPacketBuf[6] = '#';
                rtdmcrcTemp = RTDM_CumulativeCrc16( rtdmPacketBuf, 7, rtdmcrcTemp );
                rtdmPacketBuf[7] = ( rtdmcrcTemp & 0x00FF );
                rtdmPacketBuf[8] = ( rtdmcrcTemp & 0xFF00 ) >> 8;

                //Send completed message which is 9 bytes in length.
                for( N = 0; N < 9; N++ )
                {
                    WriteUART3( rtdmPacketBuf[N] );
                    while( BusyUART3() );
                }

                break;

            default:
                // ---> COMMAND SUPPORTED?? IF NOT ERROR HANDLER
                //Transmit ERROR message 1
                putsUART3( ( unsigned int * ) RTDMErrorIllegalFunction );

                /* Wait for  transmission to complete */
                while( BusyUART3() );
                break;
        }
    }

    memset( &rtdmRxBuffer, 0, sizeof(rtdmRxBuffer) );

    rtdmFlags.MessageReceived = 0;
    rtdmRxBufferIndex = rtdmRxBufferLoLimit;
    rtdmRxBufferStartMsgPointer = rtdmRxBufferLoLimit;
    rtdmRxBufferEndMsgPointer = rtdmRxBufferLoLimit;

    return ( 0 );
}

/******************************************************************************
* Function:      _U3RXInterrupt(void)
*
* PreCondition:  None
*
* Input:         None
*
* Output:        None
*
* Overview: Here is where the rtdm receives the messages using the UART receiver
*           interrupt, If polling method is selected in the RTDMUSER.h file then
*           the user application should call the RTDM_ProcessMsgs routine in
*           order to precess up comming messages. If polling method is disabled
*           then the RTDM_ProcessMsgs routine is called in the UART received
*           interrupt routine.
*
* Note:     Some processors may have more UART modules, that is why it is
*           required to specify wich UART module is going to be used by RTDM
*******************************************************************************/
        #if ( RTDM_POLLING == YES )

/* This is UART3 receive ISR Polling RTDM Messages*/
void __attribute__ ( (__interrupt__, no_auto_psv) ) _U3RXInterrupt( void )
{
    _U3RXIF = 0;

    /* Read the receive buffer until at least one or more character can be read */
    while( DataRdyUART3() )
    {
        *( rtdmRxBufferIndex++ ) = ReadUART3();
    }

    rtdmRxBufferEndMsgPointer = rtdmRxBufferIndex - 3;
    if( rtdmRxBufferIndex > (rtdmRxBufferHiLimit - 1) )
    {
        rtdmRxBufferIndex = rtdmRxBufferLoLimit;
        rtdmRxBufferEndMsgPointer = rtdmRxBufferHiLimit - 1;
    }

    if( *(rtdmRxBufferStartMsgPointer) == '$' )
    {
        if( *(rtdmRxBufferEndMsgPointer) == '#' )
        {
            rtdmFlags.MessageReceived = 1;
        }
    }
    else
    {
        rtdmRxBufferIndex = rtdmRxBufferLoLimit;
    }
}

/******************************************************************************
* Function:      _U3RXInterrupt(void)
*
* PreCondition:  None
*
* Input:         None
*
* Output:        None
*
* Overview: Here is where the rtdm receives the messages using the UART receiver
*           interrupt, If polling method is selected in the RTDMUSER.h file then
*           the user application should call the RTDM_ProcessMsgs routine in
*           order to precess up comming messages. If polling method is disabled
*           then the RTDM_ProcessMsgs routine is called in the UART received
*           interrupt routine.
*
* Note:     Some processors may have more UART modules, that is why it is
*           required to specify wich UART module is going to be used by RTDM
*******************************************************************************/
        #else

/* This is UART3 receive ISR without polling RTDM Messages */
void __attribute__ ( (__interrupt__, no_auto_psv) ) _U3RXInterrupt( void )
{
    _U3RXIF = 0;

    /* Read the receive buffer until at least one or more character can be read */
    while( DataRdyUART3() )
    {
        *( rtdmRxBufferIndex++ ) = ReadUART3();
    }

    rtdmRxBufferEndMsgPointer = rtdmRxBufferIndex - 3;
    if( rtdmRxBufferIndex > (rtdmRxBufferHiLimit - 1) )
    {
        rtdmRxBufferIndex = rtdmRxBufferLoLimit;
        rtdmRxBufferEndMsgPointer = rtdmRxBufferHiLimit - 1;
    }

    if( *(rtdmRxBufferStartMsgPointer) == '$' )
    {
        if( *(rtdmRxBufferEndMsgPointer) == '#' )
        {
            rtdmFlags.MessageReceived = 1;
            RTDM_ProcessMsgs();
        }
    }
    else
    {
        rtdmRxBufferIndex = rtdmRxBufferLoLimit;
    }
}

        #endif
    #else
        #error UART3 not avaialable on the current device
    #endif
#elif ( RTDM_UART == 4 )
    #if ( defined(RTDM_UART_V2) && (defined(__dsPIC33E__) || defined(__PIC24E__)) )

/******************************************************************************
* Function:     RTDM_Start()
*
* PreCondition: None
*
* Input:        None
*
* Output:       return 0 if no errors
*
* Overview:     Here is where the RTDM code initilizes the UART to be used to
*               exchange data wiht the host PC
*
* Note:         Some processors may have more UART modules, that is why it is
*               required to specify wich UART module is going to be used by RTDM
*******************************************************************************/
int RTDM_Start( void )
{
    /********************** UART CONFIGURATIION ******************************/
    /* Turn off UART4 module */
    CloseUART4();

    /* Configure UART4 receive and transmit interrupt */
    ConfigIntUART4( UART_RX_INT_EN & (UART_RX_INT_PR0 + RTDM_UART_PRIORITY) & UART_TX_INT_DIS & UART_TX_INT_PR2 );

    /* Configure UART4 module to transmit 8 bit data with one stopbit.  */
    rtdm_Uart_Mode_Value = UART_EN & UART_IDLE_CON & UART_IrDA_DISABLE & UART_MODE_FLOW & UART_UEN_00 & UART_DIS_WAKE & UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_UXRX_IDLE_ONE & UART_BRGH_SIXTEEN & UART_NO_PAR_8BIT & UART_1STOPBIT;

    rtdm_Uart_Sta_Value = UART_INT_TX_BUF_EMPTY & UART_IrDA_POL_INV_ZERO & UART_SYNC_BREAK_DISABLED & UART_TX_ENABLE & UART_INT_RX_CHAR & UART_ADR_DETECT_DIS & UART_RX_OVERRUN_CLEAR;

    OpenUART4( rtdm_Uart_Mode_Value, rtdm_Uart_Sta_Value, RTDM_BRG );

    /************* RTDM Flags Configuration & Initial Values *****************/
    rtdmFlags.MessageReceived = 0;
    rtdmFlags.MessageReceived = 0;
    rtdmRxBufferIndex = rtdmRxBufferLoLimit;
    rtdmRxBufferStartMsgPointer = rtdmRxBufferLoLimit;
    rtdmRxBufferEndMsgPointer = rtdmRxBufferLoLimit;

    return ( 0 );
}

/******************************************************************************
* Function:      CloseRTDM()
*
* Output:  return 0 if no errors
*
* Overview: Here is where the RTDM code closes the UART used to
*   exchange data wiht the host PC
*
* Note:  Some processors may have more UART modules, that is why it is required to
*   specify wich UART module is going to be used by RTDM 
*******************************************************************************/
int CloseRTDM( void )
{
    int nRet = 0;
    CloseUART4();
    return ( nRet );
}

/******************************************************************************
* Function:      RTDM_ProcessMsgs()
*
* PreCondition:  None
*
* Input:         None
*
* Output:        return 0 if no errors
*
* Overview:      Here is where the RTDM code process the message received and
*                then executes the required task. These tasks are reading an
*                specified memory location, writing an specified memory location,
*                receive a communication link sanity check command, or being
*                asked for the size of the bufffers.
*
* Note:         Some processors may have more UART modules, that is why it is
*               required to specify wich UART module is going to be used by RTDM
*******************************************************************************/
int RTDM_ProcessMsgs( void )
{
    //Local pointer management variables
    unsigned long int   *RTDMpu32AddressTemp;
    unsigned char       *RTDMpucWrData;
    unsigned char       *rtdmpucRdData;
    unsigned char       *rtdmpucWrAddr;
    unsigned short      rtdmNumBytes;

    unsigned int        rtdmProcessMsgsTemp1;
    unsigned int        rtdmProcessMsgsTemp2;
    unsigned int        N;

    if( !rtdmFlags.MessageReceived )
    {
        return ( -1 );
    }

    rtdmcrcTemp = RTDM_CumulativeCrc16( rtdmRxBufferStartMsgPointer,
                                        ( unsigned int ) (rtdmRxBufferEndMsgPointer - rtdmRxBufferStartMsgPointer) + 1,
                                        0xFFFF );

    rtdmcrcTempH = ( rtdmcrcTemp & 0xFF00 ) >> 8;
    rtdmcrcTempL = rtdmcrcTemp & 0x00FF;
    rtdmProcessMsgsTemp1 = ( unsigned int ) *( (rtdmRxBufferEndMsgPointer) + 2 );
    rtdmProcessMsgsTemp2 = ( unsigned int ) *( (rtdmRxBufferEndMsgPointer) + 1 );

    rtdmRxBufferStartMsgPointer += 2;
    if( (rtdmProcessMsgsTemp1 == ( unsigned ) rtdmcrcTempH) && (rtdmProcessMsgsTemp2 == rtdmcrcTempL) )
    {
        switch( *((rtdmRxBufferLoLimit) + 1) )
        {
            case 'm':
                /*************** Extract Address **************/
                //Capture address as 32 bit quantity to match protocol definition.
                rtdmpu32AddressTemp = ( ( unsigned long * ) rtdmRxBufferStartMsgPointer );

                //Increment receive buffer pointer to length field.
                rtdmRxBufferStartMsgPointer += sizeof( unsigned long );

                //Init a byte oriented data pointer
                rtdmpucRdData = ( unsigned char * ) ( ( unsigned int ) *rtdmpu32AddressTemp );

                /********* Extract Number of Bytes ***********/
                //Capture address as 16 bit quantity to match protocol definition.
                rtdmNumBytes = *( ( unsigned short * ) rtdmRxBufferStartMsgPointer );

                //Increment receive buffer pointer to start of data payload.
                rtdmRxBufferStartMsgPointer += sizeof( unsigned short );

                //Init the CRC seed for the cumulative checksum calculation.
                rtdmcrcTemp = 0xffff;

                //Add packet header prefix
                rtdmPacketBuf[0] = '+';
                rtdmPacketBuf[1] = '$';

                //Add null terminator for putsUARTx function...
                rtdmPacketBuf[2] = 0;

                //Calc header prefix checksum piece
                rtdmcrcTemp = RTDM_CumulativeCrc16( rtdmPacketBuf, 2, rtdmcrcTemp );

                //Calc data payload checksum
                rtdmcrcTemp = RTDM_CumulativeCrc16( rtdmpucRdData, rtdmNumBytes, rtdmcrcTemp );

                //Send packet header. Use string function to save code space...
                putsUART4( ( unsigned int * ) rtdmPacketBuf );
                while( BusyUART4() );

                //Send data portion of message...
                while( rtdmNumBytes-- )
                {
                    WriteUART4( *rtdmpucRdData++ );
                    while( BusyUART4() );
                }

                //Add packet trailer
                rtdmPacketBuf[0] = '#';
                rtdmcrcTemp = RTDM_CumulativeCrc16( rtdmPacketBuf, 1, rtdmcrcTemp );

                //Add checksum bytes to packet
                rtdmPacketBuf[1] = rtdmcrcTemp & 0x00FF;
                rtdmPacketBuf[2] = ( rtdmcrcTemp & 0xFF00 ) >> 8;

                //Send packet trailer and checksum.
                for( N = 0; N < 3; N++ )
                {
                    WriteUART4( rtdmPacketBuf[N] );
                    while( BusyUART4() );
                }

                break;

            case 'M':
                {
                    /*************** Extract Address **************/
                    //Capture address as 32 bit quantity to match protocol definition.
                    rtdmpu32AddressTemp = ( unsigned long * ) rtdmRxBufferStartMsgPointer;

                    //Increment receive buffer pointer to length field.
                    rtdmRxBufferStartMsgPointer += sizeof( unsigned long );

                    //Init a byte oriented address pointer for use in incrementing
                    //through the address range properly as we write each byte of data
                    //in the range (length) of this write request.
                    rtdmpucWrAddr = ( unsigned char * ) ( ( unsigned int ) *rtdmpu32AddressTemp );

                    /********* Extract Number of Bytes ************/
                    //Capture length as 16 bit quantity to match protocol definition.
                    rtdmNumBytes = *( ( unsigned short * ) rtdmRxBufferStartMsgPointer );

                    //Increment receive buffer pointer to start of data payload.
                    rtdmRxBufferStartMsgPointer += sizeof( unsigned short );

                    /********** Extract Data ************/
                    //Init a byte oriented data pointer so that we can increment a byte at at
                    //time for as many bytes as are in the range for this write.
                    rtdmpucWrData = rtdmRxBufferStartMsgPointer;

                    //*** Write Data in specified RAM location *****
                    //Important to increment through address range using byte oriented address and data
                    //pointers. Otherwise, single byte or odd byte ranges do not get written correctly.
                    while( rtdmNumBytes-- )
                    {
                        *rtdmpucWrAddr++ = *rtdmpucWrData++;
                    }

                    //Transmit OK message
                    putsUART4( ( unsigned int * ) rtdmWriteMemoryOK );

                    /* Wait for  transmission to complete */
                    while( BusyUART4() );
                    break;
                }

            case 's':
                {
                    /* Load transmit buffer and transmit the same till null character is encountered */
                    //Transmit OK message
                    putsUART4( ( unsigned int * ) rtdmSanityCheckOK );

                    /* Wait for  transmission to complete */
                    while( BusyUART4() );
                    break;
                }

            case 'L':
                rtdmcrcTemp = 0xffff;   //Init the CRC seed.
                rtdmPacketBuf[0] = '+';
                rtdmPacketBuf[1] = '$';

                //Size of the rtdm Receive buffer.
                rtdmPacketBuf[2] = ( sizeof(rtdmRxBuffer) & 0x00FF );
                rtdmPacketBuf[3] = ( sizeof(rtdmRxBuffer) & 0xFF00 ) >> 8;

                //Note: We dod not utilize a transmit buffer since any data memory source is
                //essentially already buffered. So the transmit limit is now just a way to
                //limit the total message length that a client make with any single read request.
                rtdmPacketBuf[4] = ( RTDM_MAX_XMIT_LEN & 0x00FF );
                rtdmPacketBuf[5] = ( RTDM_MAX_XMIT_LEN & 0xFF00 ) >> 8;
                rtdmPacketBuf[6] = '#';
                rtdmcrcTemp = RTDM_CumulativeCrc16( rtdmPacketBuf, 7, rtdmcrcTemp );
                rtdmPacketBuf[7] = ( rtdmcrcTemp & 0x00FF );
                rtdmPacketBuf[8] = ( rtdmcrcTemp & 0xFF00 ) >> 8;

                //Send completed message which is 9 bytes in length.
                for( N = 0; N < 9; N++ )
                {
                    WriteUART4( rtdmPacketBuf[N] );
                    while( BusyUART4() );
                }

                break;

            default:
                // ---> COMMAND SUPPORTED?? IF NOT ERROR HANDLER
                //Transmit ERROR message 1
                putsUART4( ( unsigned int * ) rtdmErrorIllegalFunction );

                /* Wait for  transmission to complete */
                while( BusyUART4() );
                break;
        }
    }

    memset( &rtdmRxBuffer, 0, sizeof(rtdmRxBuffer) );

    rtdmFlags.MessageReceived = 0;
    rtdmRxBufferIndex = rtdmRxBufferLoLimit;
    rtdmRxBufferStartMsgPointer = rtdmRxBufferLoLimit;
    rtdmRxBufferEndMsgPointer = rtdmRxBufferLoLimit;

    return ( 0 );
}

/******************************************************************************
* Function:      _U4RXInterrupt(void)
*
* PreCondition:  None
*
* Input:         None
*
* Output:        None
*
* Overview: Here is where the rtdm receives the messages using the UART receiver
*           interrupt, If polling method is selected in the RTDMUSER.h file then
*           the user application should call the RTDM_ProcessMsgs routine in
*           order to precess up comming messages. If polling method is disabled
*           then the RTDM_ProcessMsgs routine is called in the UART received
*           interrupt routine.
*
* Note:     Some processors may have more UART modules, that is why it is
*           required to specify wich UART module is going to be used by RTDM
*******************************************************************************/
        #if ( RTDM_POLLING == YES )

/* This is UART4 receive ISR Polling RTDM Messages*/
void __attribute__ ( (__interrupt__, no_auto_psv) ) _U4RXInterrupt( void )
{
    _U4RXIF = 0;

    /* Read the receive buffer until at least one or more character can be read */
    while( DataRdyUART4() )
    {
        *( rtdmRxBufferIndex++ ) = ReadUART4();
    }

    rtdmRxBufferEndMsgPointer = rtdmRxBufferIndex - 3;
    if( rtdmRxBufferIndex > (rtdmRxBufferHiLimit - 1) )
    {
        rtdmRxBufferIndex = rtdmRxBufferLoLimit;
        rtdmRxBufferEndMsgPointer = rtdmRxBufferHiLimit - 1;
    }

    if( *(rtdmRxBufferStartMsgPointer) == '$' )
    {
        if( *(rtdmRxBufferEndMsgPointer) == '#' )
        {
            rtdmFlags.MessageReceived = 1;
        }
    }
    else
    {
        rtdmRxBufferIndex = rtdmRxBufferLoLimit;
    }
}

/******************************************************************************
* Function:      _U4RXInterrupt(void)
*
* PreCondition:  None
*
* Input:         None
*
* Output:        None
*
* Overview: Here is where the rtdm receives the messages using the UART receiver
*           interrupt, If polling method is selected in the RTDMUSER.h file then
*           the user application should call the RTDM_ProcessMsgs routine in
*           order to precess up comming messages. If polling method is disabled
*           then the RTDM_ProcessMsgs routine is called in the UART received
*           interrupt routine.
*
* Note:     Some processors may have more UART modules, that is why it is
*           required to specify wich UART module is going to be used by RTDM
*******************************************************************************/
        #else

/* This is UART4 receive ISR without polling RTDM Messages */
void __attribute__ ( (__interrupt__, no_auto_psv) ) _U4RXInterrupt( void )
{
    _U4RXIF = 0;

    /* Read the receive buffer until at least one or more character can be read */
    while( DataRdyUART4() )
    {
        *( rtdmRxBufferIndex++ ) = ReadUART4();
    }

    rtdmRxBufferEndMsgPointer = rtdmRxBufferIndex - 3;
    if( rtdmRxBufferIndex > (rtdmRxBufferHiLimit - 1) )
    {
        rtdmRxBufferIndex = rtdmRxBufferLoLimit;
        rtdmRxBufferEndMsgPointer = rtdmRxBufferHiLimit - 1;
    }

    if( *(rtdmRxBufferStartMsgPointer) == '$' )
    {
        if( *(rtdmRxBufferEndMsgPointer) == '#' )
        {
            rtdmFlags.MessageReceived = 1;
            RTDM_ProcessMsgs();
        }
    }
    else
    {
        rtdmRxBufferIndex = rtdmRxBufferLoLimit;
    }
}

        #endif
    #else
        #error UART4 not avaialable on the current device
    #endif
#else
    #error Please define the UART to be used by RTDM in RTDMuserdef.h file
#endif

//Conditionally compile for speed performance or minimum code size.
#if ( RTDM_MIN_CODE_SIZE == NO )

//When RTDM_MIN_CODE_SIZE is not defined we employ a table driven crc
//calculation with pre-calculated polyniomial values to speed-up
//checksum calculation time.
const unsigned int  crc_16_tab[] = { 0x0000, 0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241, 0xc601, 0x06c0, 0x0780, 0xc741, 0x0500, 0xc5c1, 0xc481, 0x0440, 0xcc01, 0x0cc0, 0x0d80, 0xcd41, 0x0f00, 0xcfc1, 0xce81, 0x0e40, 0x0a00, 0xcac1, 0xcb81, 0x0b40, 0xc901, 0x09c0, 0x0880, 0xc841, 0xd801, 0x18c0, 0x1980, 0xd941, 0x1b00, 0xdbc1, 0xda81, 0x1a40, 0x1e00, 0xdec1, 0xdf81, 0x1f40, 0xdd01, 0x1dc0, 0x1c80, 0xdc41, 0x1400, 0xd4c1, 0xd581, 0x1540, 0xd701, 0x17c0, 0x1680, 0xd641, 0xd201, 0x12c0, 0x1380, 0xd341, 0x1100, 0xd1c1, 0xd081, 0x1040, 0xf001, 0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1, 0xf281, 0x3240, 0x3600, 0xf6c1, 0xf781, 0x3740, 0xf501, 0x35c0, 0x3480, 0xf441, 0x3c00, 0xfcc1, 0xfd81, 0x3d40, 0xff01, 0x3fc0, 0x3e80, 0xfe41, 0xfa01, 0x3ac0, 0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840, 0x2800, 0xe8c1, 0xe981, 0x2940, 0xeb01, 0x2bc0, 0x2a80, 0xea41, 0xee01, 0x2ec0, 0x2f80, 0xef41, 0x2d00, 0xedc1, 0xec81, 0x2c40, 0xe401, 0x24c0, 0x2580, 0xe541, 0x2700, 0xe7c1, 0xe681, 0x2640, 0x2200, 0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0, 0x2080, 0xe041, 0xa001, 0x60c0, 0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281, 0x6240, 0x6600, 0xa6c1, 0xa781, 0x6740, 0xa501, 0x65c0, 0x6480, 0xa441, 0x6c00, 0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0, 0x6e80, 0xae41, 0xaa01, 0x6ac0, 0x6b80, 0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840, 0x7800, 0xb8c1, 0xb981, 0x7940, 0xbb01, 0x7bc0, 0x7a80, 0xba41, 0xbe01, 0x7ec0, 0x7f80, 0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40, 0xb401, 0x74c0, 0x7580, 0xb541, 0x7700, 0xb7c1, 0xb681, 0x7640, 0x7200, 0xb2c1, 0xb381, 0x7340, 0xb101, 0x71c0, 0x7080, 0xb041, 0x5000, 0x90c1, 0x9181, 0x5140, 0x9301, 0x53c0, 0x5280, 0x9241, 0x9601, 0x56c0, 0x5780, 0x9741, 0x5500, 0x95c1, 0x9481, 0x5440, 0x9c01, 0x5cc0, 0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40, 0x5a00, 0x9ac1, 0x9b81, 0x5b40, 0x9901, 0x59c0, 0x5880, 0x9841, 0x8801, 0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1, 0x8a81, 0x4a40, 0x4e00, 0x8ec1, 0x8f81, 0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41, 0x4400, 0x84c1, 0x8581, 0x4540, 0x8701, 0x47c0, 0x4680, 0x8641, 0x8201, 0x42c0, 0x4380, 0x8341, 0x4100, 0x81c1, 0x8081, 0x4040 };

/******************************************************************************
* Function:  unsigned int  RTDM_CumulativeCrc16 (unsigned char *buf,
*            unsigned int u16Length, unsigned int u16CRC)
*
* PreCondition:None
*
* Input:       None
*
* Output:      return CRC16
*
* Overview:    This routine calculates the polynomial for the checksum byte on
*              the fly every time. Saves code space because no const table is
*              required. This approach saves code space but yields slower
*              throughput performance.
*
* Note:        Some processors may have more UART modules, that is why it is
*              required to specify wich UART module is going to be used by RTDM
*******************************************************************************/
unsigned int RTDM_CumulativeCrc16( unsigned char *buf, unsigned int bsize, unsigned int crcSeed )
{
    unsigned char   *pData = buf;

    while( bsize-- )
    {
        crcSeed = ( unsigned int ) ( crcSeed >> 8 ) ^ crc_16_tab[( crcSeed ^ *pData++ ) & 0xff];
    }

    return ( crcSeed );
}

#else //This is when the _RTDM_CODE_FOOTPRINT = RTDM_MIN_SIZE

/******************************************************************************
* Function:  unsigned int  RTDM_CumulativeCrc16 (unsigned char *buf,
*            unsigned int u16Length, unsigned int u16CRC)
*
* PreCondition:None
*
* Input:       None
*
* Output:      return CRC16
*
* Overview:    This routine calculates the polynomial for the checksum byte on
*              the fly every time. Saves code space because no const table is
*              required. This approach saves code space but yields slower
*              throughput performance.
*
* Note:        Some processors may have more UART modules, that is why it is
*              required to specify wich UART module is going to be used by RTDM
*******************************************************************************/
int wcopy;
unsigned int RTDM_CumulativeCrc16( unsigned char *buf, unsigned int u16Length, unsigned int u16CRC )
{
    unsigned int    u16Poly16 = 0xA001; // Bits 15, 13 and 0
    unsigned int    DATA_BITS = 8;      // Number of data bits
    unsigned int    u16BitIdx;
    unsigned int    u16MsgIdx;
    unsigned int    u16MsgByte;

    for( u16MsgIdx = 0; u16MsgIdx < u16Length; u16MsgIdx++ )
    {
        asm( "mov.w w14,_wcopy" );
        u16MsgByte = 0x00FF &*buf++;
        for( u16BitIdx = 0; u16BitIdx < DATA_BITS; u16BitIdx++ )
        {
            if( (u16CRC ^ u16MsgByte) & 0x0001 )
            {
                u16CRC = ( u16CRC >> 1 ) ^ u16Poly16;
            }
            else
            {
                u16CRC = u16CRC >> 1;
            }

            u16MsgByte >>= 1;           // Right shift one to get to next bit
        }
    }

    return ( u16CRC );
}

#endif /* End of #ifndef compilation condition. */

/*******************************************************************************
 End of File
*/

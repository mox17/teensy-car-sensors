/*
 * Send data using byte stuffing protocol
 * 
 * Output:
 * Byte in frame has value 0x7E is changed into 2 bytes: 0x7D, 0x5E
 * Byte in frame has value 0x7D is changed into 2 bytes: 0x7D, 0x5D
 * Input:
 * When byte 0x7D is received, discard this byte, and the next byte is XORed with 0x20.
 * 
 */
#include "telemetry.h"
#include <HardwareSerial.h>
#include <QueueList.h>

#define serialPort Serial1

#define START_STOP 0x7e
#define DATA_ESCAPE 0x7d
#define DATA_FRAME 0x10


void serialPolling()
{
    if (serialPort.available() > 0)
    {
        
    }
}

uint16_t rxCrc=0;
uint16_t txCrc=0;

void SerialTransmitByte(uint8_t b) 
{
    if ( b == 0x7e ) {
        serialPort.write( 0x7d );
        serialPort.write( 0x5e );
    } else if ( b == 0x7d ) {
        serialPort.write( 0x7d );
        serialPort.write( 0x5d );
    } else {
        serialPort.write( b );
    } 
}

void SerialSendByte(uint8_t b) 
{
    SerialTransmitByte( b );
    // CRC update
    txCrc += b; //0-1FF
    txCrc += ( txCrc >> 8 ); //0-100
    txCrc &= 0x00ff;
}

void SerialSendCrc() 
{
    SerialTransmitByte( 0xFF-txCrc );
    txCrc = 0; // CRC reset
}

void SerialSendPacket(uint16_t id, uint32_t value) 
{
    SerialSendByte(DATA_FRAME);
    uint8_t *bytes = (uint8_t*)&id;
    SerialSendByte(bytes[0]);
    SerialSendByte(bytes[1]);
    bytes = (uint8_t*)&value;
    SerialSendByte(bytes[0]);
    SerialSendByte(bytes[1]);
    SerialSendByte(bytes[2]);
    SerialSendByte(bytes[3]);
    SerialSendCrc();
    serialPort.flush();
}

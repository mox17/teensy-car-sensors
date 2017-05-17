/*
 * Send data using byte stuffing protocol
 * 
 * Output:
 * Byte in frame has value 0x7E is changed into 2 bytes: 0x7D, 0x5E
 * Byte in frame has value 0x7D is changed into 2 bytes: 0x7D, 0x5D
 * Input:
 * When byte 0x7D is received, discard this byte, and the next byte is XORed with 0x20.
 * 
 * The telemetry code will do a best effort to send as much data as possible over the link.
 * There may be more data produced than what can be sent. It is therefore split into a number of queues.
 * 1) priorityQueue  These are messages that should NOT be dropped. For example ping/pong messages.
 * 2) sonarQueue     These are measurements of obstacle distances. These should be mixed fairly with rotationQueue.
 * 3) rotationQueue  These are measurements of the wheel rotations. These should be mixed fairly with sonarQueue.
 * 
 * When selecting what to send, first check priority queue. Then consider ultrasoundQueue and odometerQueue in alternating order.
 * If data cannot be sent, then items in ultrasoundQueue and odometerQueue can be replaced with newer measurements.
 *
 *
 */
#include "telemetry.h"
#include <HardwareSerial.h>
#include <QueueList.h>
#include "sonararray.h"

#define serialPort Serial1


// Outgoing packet queues
QueueList <packet *> priorityQueue;  // allow for 4
QueueList <packet *> rotationQueue;  // allow for 2
QueueList <packet *> sonarQueue[MAX_NO_OF_SONAR]; // allow for 6+2
QueueList <packet *> freeList;

// Static array of packet buffers to flow around the queues
const unsigned bufferCount = 4+2+6+2;
packet packetPool[bufferCount];

unsigned sequenceIdx=0;
const unsigned sequenceMax = MAX_NO_OF_SONAR+1;
QueueList <packet *> * queueSequence[sequenceMax];  // This is used to cycle between queues

/**
 * \brief Initialize packet queues
 * Initialize free list
 */
void initQueues()
{
    for (unsigned i=0;i<bufferCount;i++)
    {
        freeList.push(&packetPool[i]);
    }
    sequenceIdx = 0;
    queueSequence[sequenceIdx++] = &rotationQueue;
    for (unsigned i=0; i<MAX_NO_OF_SONAR; i++)
    {
        queueSequence[sequenceIdx++] = &sonarQueue[i];
    }
    sequenceIdx = 0;
}

/**
 * \brief Get a packet from most appropriate queue
 */
packet * getPacketFromQueues()
{
    // First check priority queue
    if (!priorityQueue.isEmpty())
    {
        return priorityQueue.pop();
    }
    for (unsigned i=0;i<sequenceMax;i++)
    {
        if (queueSequence[sequenceIdx]->count())
        {
            sequenceIdx = ((sequenceIdx+1)%sequenceMax);
            return queueSequence[sequenceIdx]->pop();
        }
    }
    return NULL;
}

size_t getPacketLength(packet *p)
{
    switch (p->hdr.cmd) 
    {
    case CMD_PING:  
    case CMD_PONG:
        return sizeof(pingpong);

    case CMD_US_SET_SEQ:
        return sizeof(sequence);

    case CMD_US_STOP:
    case CMD_US_START:
        return sizeof(header);

    case CMD_US_STATUS:
        return sizeof(distance);

    case CMD_ROT_STATUS:
        return sizeof(rotation);

    case CMD_ROT_RESET:
        return sizeof(header);
    }
    return 0;
}

void setPing(packet &packet, uint32_t val)
{
    packet.pp.hdr.dst = ADDR_RPI;
    packet.pp.hdr.src = ADDR_TEENSY;
    packet.pp.hdr.cmd = CMD_PING;
    packet.pp.hdr.reserved = 0;
    packet.pp.timestamp1 = val;
    packet.pp.timestamp2 = 0;
}

void serialPolling()
{
    if (serialPort.available() > 0)
    {
        
    }
}

inline void crcUpdate(byte b);

enum receiveStates {
    RS_BEGIN,  // Waiting for framing 0x7e
    RS_DATA,   // Incoming data
    RS_ESCAPE, // Received 0x7d - xor next byte with 0x20
};

packet *currentRxPacket = NULL;
receiveStates rxState = RS_BEGIN;
uint16_t currentRxOffset = 0;
uint16_t rxCrc=0;
uint32_t rxErrorCount=0;  // Number of packets with checksum errors
uint32_t rxErrorBuffer=0; // No receive buffer available

bool getRxBuffer()
{
    if (freeList.count())
    {
        currentRxPacket = freeList.pop();
        currentRxOffset = 0;
        rxState = RS_BEGIN;
        rxCrc=0;
        return true;
    }
    else
    {
        rxErrorBuffer++;
        return false;
    }
}

void handleRxByte(byte b)
{
    if (currentRxPacket == NULL)
    {
        if (!getRxBuffer())
        {
            return;
        }
    }
    switch (rxState)
    {
    case RS_BEGIN:
        // Waiting for 0x7e to arrive
        break;

    case RS_DATA:
        break;
    
    case RS_ESCAPE:
        break;
    }
}

enum transmitStates {
    TS_BEGIN,  // Nothing sent yet, deliver 0x7e
    TS_DATA,   // Sending normal data
    TS_ESCAPE, // Escape has been sent, escByte is next
    TS_CHKSUM, // Last data byte sent, checksum is next
    TS_END,    // Checksum sent, frame is next. Can be skipped if there is a next packet in queue
    TS_IDLE,   // No data to transmit.
};

packet *currentTxPacket = NULL;       // The currently transmitting packet
uint16_t totalTxSize;                 // Total number of bytes (payload - note escaped) in bufefr
uint16_t currentTxOffset;             // Current offset for transmission
transmitStates txState = TS_BEGIN;    // State for packet transmission
byte escByte;                         // stuffed byte, i.e. value xor'ed with 0x20 to transmit.
uint16_t txCrc=0;

/**
 * \brief Handle the last by of the packet.
 * Put transmitted packet buffer back on the free list.
 * If there is another packet ready return true, otherwise false.
 * If there is no new packet, then end frame marker and serial flush is wanted.
 */
bool endOfPacketHandling()
{
    packet *p;

    if (currentTxPacket != NULL)
    {
        freeList.push(currentTxPacket);
        currentTxPacket = NULL;
    }
    p = getPacketFromQueues();
    if (p != NULL)
    {
        currentTxPacket = p;
        currentTxOffset = 0;
        totalTxSize = getPacketLength(p);
        txCrc = 0;
        return true;
    }
    return false;
}

/*
 * Pull bytes one-by-one from buffer with framing, stuffing and checksum calculation
 */
bool getPacketByte(byte &b)
{
    bool ret = true;
    switch (txState)
    {
    case TS_BEGIN:
        b = FRAME_START_STOP;
        txState = TS_DATA;
        break;

    case TS_DATA:
        b = currentTxPacket->raw[currentTxOffset++];
        if ((b == FRAME_DATA_ESCAPE)||(b == FRAME_START_STOP))
        {
            escByte = b ^ FRAME_XOR;
            b = FRAME_DATA_ESCAPE;
        }
        else 
        {
            txState = (currentTxOffset < totalTxSize) ? TS_DATA : TS_CHKSUM;
        }
        break;

    case TS_ESCAPE:
        b = escByte;
        txState = (currentTxOffset < totalTxSize) ? TS_DATA : TS_CHKSUM;
        break;

    case TS_CHKSUM:
        b = txCrc;
        txState = TS_END;
        break;

    case TS_END:
        if (endOfPacketHandling())
        {
            // A new packet is ready to go
            b = FRAME_START_STOP;  // This will serve as separator between packets
            txState = TS_DATA;
        }
        else
        {
        b = FRAME_START_STOP;  // Terminator for this packet
        txState = TS_IDLE;
        }
        break;

    case TS_IDLE:
        // While waiting for data, check the queues
        if (endOfPacketHandling())
        {
            // A new packet is ready to go
            b = FRAME_START_STOP;  // This will serve as separator between packets
            txState = TS_DATA;
        }
        else
        {
            return false;
        }
    }
    crcUpdate(b);
    return ret;
}

void serviceSerial()
{
    int txRoom = Serial1.availableForWrite();
    int rxRoom = Serial1.available();
    
    while (txRoom > 0) 
    {
        
    }
}



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

inline void crcUpdate(byte b)
{
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
    //SerialSendByte(DATA_FRAME);
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

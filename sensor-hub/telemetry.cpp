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

Telemetry::Telemetry(HardwareSerial port, unsigned speed) :
    rxCurrentPacket(NULL),
    rxState(RS_BEGIN),
    rxCurrentOffset(0),
    rxChecksum(0),
    rxErrorChecksum(0),
    rxErrorTooShort(0),
    rxErrorTooLong(0),
    rxErrorBuffer(0),
    rxErrorDropped(0),
    txCurrentPacket(NULL),
    txTotalSize(0),
    txCurrentOffset(0),
    txState(TS_BEGIN),
    txChecksum(0),
    sequenceIdx(0)
{
    serialPort = port;
    initQueues();
    serialPort.begin(speed);
    serialPort.println("ctr...");
}

/**
 * \brief Initialize packet queues
 * Initialize free list
 */
void Telemetry::initQueues()
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
packet * Telemetry::getPacketFromQueues()
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

size_t Telemetry::getPacketLength(packet *p)
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

void Telemetry::setPing(packet &packet, uint32_t val)
{
    packet.pp.hdr.dst = ADDR_RPI;
    packet.pp.hdr.src = ADDR_TEENSY;
    packet.pp.hdr.cmd = CMD_PING;
    packet.pp.hdr.reserved = 0;
    packet.pp.timestamp1 = val;
    packet.pp.timestamp2 = 0;
}

void Telemetry::serialPolling()
{
    // first handle incoming data
    while (serialPort.available() > 0) 
    {
        rxHandleUartByte(serialPort.read());
    }
    // Then transmit data
    while (serialPort.availableForWrite() > 0)
    {
        byte b;
        if (txGetPacketByte(b))
        {
            serialPort.write(b);
        }
    }
}

void Telemetry::rxInitPacket()
{
    if (rxCurrentPacket)
    {
        rxCurrentOffset = 0;
        rxState = RS_BEGIN;
        rxChecksum = 0;
    }
}

bool Telemetry::rxGetBuffer()
{
    if (freeList.count())
    {
        rxCurrentPacket = freeList.pop();
        rxInitPacket();
        return true;
    }
    else
    {
        rxErrorBuffer++;
        return false;
    }
}

void Telemetry::rxSaveByte(byte b)
{
    if ((rxCurrentOffset <= MAX_MSG_SIZE) && (rxCurrentPacket != NULL))
    {
        rxCurrentPacket->raw[rxCurrentOffset++] = b;
    } else {
        rxErrorTooLong++;
        rxInitPacket();
    }
}

/**
 * @brief Calculate checksum for incoming data.
 *
 * The incoming checksum is calculated when the FRAME_START_STOP byte is received.
 * The final checksum, with the checksum included, shall be 0xff 
 */
void Telemetry::rxCalcChecksum(byte b)
{
    crcUpdate(rxChecksum, b);
}

/*
 * @brief Validation and processing of received packet
 * 
 * The indicated length is 1 longer than actual payload because of checksum byte.
 */
bool Telemetry::rxEndOfPacketHandling()
{
    if ((rxChecksum != 0xff))
    {
        rxErrorChecksum++;
        rxInitPacket();
        return false;
    }
    if (rxCurrentOffset <= sizeof(header))
    {
        rxErrorTooShort++;
        rxInitPacket();
        return false;
    }
    if (rxCurrentOffset > (MAX_MSG_SIZE+1))
    {
        rxErrorTooLong++;
        rxInitPacket();
        return false;
    }
    // Packet has passed validation tests and can now be acted on.

    return true;
}

/**
 * @brief Handle incoming bytes from UART. 
 * 
 * This function is called from polling loop.
 */
void Telemetry::rxHandleUartByte(byte b)
{
    if (rxCurrentPacket == NULL)
    {
        if (!rxGetBuffer())
        {
            return;
        }
    }
    switch (rxState)
    {
    case RS_BEGIN:
        // Waiting for 0x7e to arrive
        if (b == FRAME_START_STOP)
        {
            rxInitPacket();
            rxCalcChecksum(b);
            rxState = RS_DATA;
        } else {
            rxErrorDropped++;
        }
        break;

    case RS_DATA:
        if (b == FRAME_DATA_ESCAPE)
        {
            rxCalcChecksum(b);
            rxState = RS_ESCAPE;
        } else if (b == FRAME_START_STOP)
        {
            txEndOfPacketHandling();
        } else {
            rxCalcChecksum(b);
            rxSaveByte(b);        
        }
        break;
    
    case RS_ESCAPE:
        rxCalcChecksum(b);
        rxSaveByte(b ^ FRAME_XOR);
        rxState = RS_DATA;
        break;
    }
}

/**
 * @brief Handling triggered by the last byte of an outgoing packet.
 *
 * Put transmitted packet buffer back on the free list.
 * If there is another packet ready return true, otherwise false.
 * If there is no new packet, then end frame marker and serial flush is wanted.
 */
bool Telemetry::txEndOfPacketHandling()
{
    packet *p;

    if (txCurrentPacket != NULL)
    {
        freeList.push(txCurrentPacket);
        txCurrentPacket = NULL;
    }
    p = getPacketFromQueues();
    if (p != NULL)
    {
        txCurrentPacket = p;
        txCurrentOffset = 0;
        txTotalSize = getPacketLength(p);
        txChecksum = 0;
        return true;
    }
    return false;
}

/**
 * @brief Calculate bytes for TX side
 *
 * Pull bytes one-by-one from buffer with framing, stuffing and checksum calculation
 */
bool Telemetry::txGetPacketByte(byte &b)
{
    bool ret = true;
    switch (txState)
    {
    case TS_BEGIN:
        b = FRAME_START_STOP;
        txState = TS_DATA;
        break;

    case TS_DATA:
        b = txCurrentPacket->raw[txCurrentOffset++];
        if ((b == FRAME_DATA_ESCAPE)||(b == FRAME_START_STOP))
        {
            txEscByte = b ^ FRAME_XOR;
            b = FRAME_DATA_ESCAPE;
        } else {
            txState = (txCurrentOffset < txTotalSize) ? TS_DATA : TS_CHKSUM;
        }
        break;

    case TS_ESCAPE:
        b = txEscByte;
        txState = (txCurrentOffset < txTotalSize) ? TS_DATA : TS_CHKSUM;
        break;

    case TS_CHKSUM:
        b = 0xFF-txChecksum;  // Finalize checksum to total 0xFF
        txState = TS_END;
        break;

    case TS_END:
        b = FRAME_START_STOP;  // This will serve as separator between packets
        if (txEndOfPacketHandling())
        {
            // A new packet is ready to go, separate delivered here
            txState = TS_DATA;
        } else {
            txState = TS_IDLE;
        }
        break;

    case TS_IDLE:
        // While waiting for data, check the queues
        if (txEndOfPacketHandling())
        {
            // A new packet is ready to go
            b = FRAME_START_STOP;  // This will serve as separator between packets
            txState = TS_DATA;
        } else {
            return false;
        }
    }
    // Checksum is calculated over all bytes sent from FRAME_START_STOP up to
    // but not including the checksum byte preceeding the closing FRAME_START_STOP
    crcUpdate(txChecksum, b);
    return ret;
}

inline void Telemetry::crcUpdate(uint16_t &chksum, byte b)
{
    chksum += b; //0-1FF
    chksum += (txChecksum >> 8); //0-100
    chksum &= 0x00ff;
}

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
#include "counters.h"

Telemetry::Telemetry(HardwareSerial port, unsigned speed) :
    rxCurrentPacket(NULL),
    rxState(RS_BEGIN),
    rxCurrentOffset(0),
    rxChecksum(0),
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
}

/**
 * @brief Send wheel event to RPi.
 *
 * Replace any pending event already in TX queue.
 */
void Telemetry::wheelEvent(rot_one left, rot_one right)
{
    packet *p;

    if (rotationQueue.isEmpty())
    {
        p = getEmptyPacket();
        if (p == NULL)
        {
            return;
        }
    } else {
        // An earlier buffer was not sent yet, so it is updated.
        cnt.inc(txInfoWheelDrop);
        counterUpdate = true;
        p = (packet*)rotationQueue.pop();
    }
    // At this point we have a tx buffer and the rotationQueue should be empty (or shorter)
    p->rt.hdr.dst = ADDR_RPI;
    p->rt.hdr.src = ADDR_TEENSY;
    p->rt.hdr.cmd = CMD_WHEEL_STATUS;
    p->rt.hdr.reserved = 0;
    p->rt.rot[ROT_LEFT] = left;
    p->rt.rot[ROT_RIGHT] = right;
    rotationQueue.push(p);
}

/**
 * @brief Send a sonar event to RPi.
 *
 * Replace any pending sonar event in queue for this sensor.
 */
void Telemetry::sonarEvent(packet *sonarPacket)
{
    byte sensor = sonarPacket->ds.sensor;

    if (!sonarQueue[sensor].isEmpty())
    {
        packet *p;
        // An earlier buffer was not sent yet, so it is updated.
        cnt.inc(txInfoSonarDrop);
        p = (packet*)sonarQueue[sensor].pop();
        freePacket(p);
    }
    sonarPacket->ds.hdr.dst = ADDR_RPI;
    sonarPacket->ds.hdr.src = ADDR_TEENSY;
    sonarPacket->ds.hdr.reserved = 0;
    sonarQueue[sensor].push(sonarPacket);
}

/**
 * @brief Send a ping message to Raspberry Pi side and prepare for result.
 */
void Telemetry::sendPing(bool &ready, uint32_t &delay)
{
    packet *p;

    p = getEmptyPacket();
    if (p == NULL)
    {
        return;
    }
    p->pp.hdr.dst = ADDR_RPI;
    p->pp.hdr.src = ADDR_TEENSY;
    p->pp.hdr.cmd = CMD_PING_QUERY;
    p->pp.hdr.reserved = 0;
    p->pp.timestamp1 = millis();
    p->pp.timestamp2 = 0;
    pingReady = &ready;
    *pingReady = false;
    pingDelay = &delay;
    *pingDelay = 0;
    priorityQueue.push(p);
}

/**
 * @brief Initialize packet queues
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
 * @brief Get a packet from most appropriate queue
 *
 * This is the prioritized packet scheduler for optimizing the bandwidth to RPi.
 * It is a round-robin scheme for non-prioritized messages.
 */
packet * Telemetry::txGetPacketFromQueues()
{
    // First check priority queue
    if (!priorityQueue.isEmpty())
    {
        return (packet*)priorityQueue.pop();
    }
    for (unsigned i=0;i<sequenceMax;i++)
    {
        if (!queueSequence[sequenceIdx]->isEmpty())
        {
            packet *p = (packet*)queueSequence[sequenceIdx]->pop();
            return p;
        }
        sequenceIdx = ((sequenceIdx+1)%sequenceMax);
    }
    return NULL;
}

/**
 * @brief Calculate packet length from command (opcode)
 *
 * This function is called when the packet is ready for transmission.
 */
size_t Telemetry::getPacketLength(packet *p)
{
    unsigned len;
    switch ((command)p->hdr.cmd)
    {
    case CMD_PING_QUERY:
    case CMD_PONG_RESP:
        return sizeof(pingpong);

    case CMD_SET_SONAR_SEQ:
        return sizeof(sequence);

    case CMD_SONAR_STOP:
    case CMD_SONAR_START:
    case CMD_WHEEL_RESET:
    case CMD_SONAR_RETRY:
    case CMD_GET_COUNTERS:
        return sizeof(header);

    case CMD_SONAR_STATUS:
        return sizeof(distance);

    case CMD_WHEEL_STATUS:
        return sizeof(rotation);

    case CMD_ERROR_COUNT:
        len = strlen(p->ec.name);
        return (sizeof(header)+sizeof(p->ec.count)+len+1);
    }
    return 0;
}

void Telemetry::serialPolling()
{
    // first handle incoming data
    while (serialPort.available() > 0)
    {
        rxHandleUartByte(serialPort.read());
    }
    // Then transmit data
    byte b;
    while ((serialPort.availableForWrite() > 0) && txGetPacketByte(b))
    {
        serialPort.write(b);
        cnt.inc(txBytes);
    }
}

void Telemetry::rxReInitPacket()
{
    if (rxCurrentPacket)
    {
        rxCurrentOffset = 0;
        rxState = RS_BEGIN;
        rxChecksum = 0;
    }
}

/**
 * @brief Make a buffer ready for receiving serial data.
 *
 * In case of error a counter in increased.
 */
bool Telemetry::rxGetBuffer()
{
    if (freeList.count())
    {
        rxCurrentPacket = (packet*)freeList.pop();
        rxReInitPacket();
        return true;
    } else {
        cnt.inc(rxErrorBuffer);
        counterUpdate = true;
        return false;
    }
}

void Telemetry::rxSaveByte(byte b)
{
    if ((rxCurrentOffset <= MAX_MSG_SIZE) && (rxCurrentPacket != NULL))
    {
        rxCurrentPacket->raw[rxCurrentOffset++] = b;
    } else {
        cnt.inc(rxErrorTooLong);
        counterUpdate = true;
        rxReInitPacket();
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
    // Check basic packet validity
    if ((rxChecksum != 0xff))
    {
        cnt.inc(rxErrorChecksum);
        counterUpdate = true;
        rxReInitPacket(); // recycle current rx buffer in place
        return false;
    }
    if (rxCurrentOffset <= sizeof(header))
    {
        cnt.inc(rxErrorTooShort);
        counterUpdate = true;
        rxReInitPacket();
        return false;
    }
    if (rxCurrentOffset > (MAX_MSG_SIZE+1))
    {
        cnt.inc(rxErrorTooLong);
        counterUpdate = true;
        rxReInitPacket();
        return false;
    }
    // Packet has passed basic validation tests and can now be acted on.
    cnt.inc(rxPackets);
    packet *p = rxCurrentPacket;
    rxCurrentPacket = NULL;
    // Basic sanity check of incoming packet
    if (p->hdr.dst == ADDR_TEENSY)
    {
        switch ((command)p->hdr.cmd)
        {
        case CMD_PING_QUERY:
            // Reply with a CMD_PONG_RESP
            p->pp.hdr.dst = p->pp.hdr.src;
            p->pp.hdr.src = ADDR_TEENSY;
            p->pp.hdr.cmd = CMD_PONG_RESP;
            p->pp.timestamp2 = millis();
            priorityQueue.push(p);
            p = NULL;
            break;

        case CMD_PONG_RESP:
            // Response to an earlier ping (we hope)
            if ((pingReady != NULL) && (pingDelay != NULL))
            {
                *pingDelay = millis() - p->pp.timestamp1;
                *pingReady = true;
            }
            mainLoop.push(p);
            p = NULL;
            break;

        case CMD_SET_SONAR_SEQ:
            mainLoop.push(p);
            p = NULL;
            break;

        case CMD_SONAR_STOP:
            mainLoop.push(p);
            p = NULL;
            break;

        case CMD_SONAR_START:
            mainLoop.push(p);
            p = NULL;
            break;

        case CMD_WHEEL_RESET:
            mainLoop.push(p);
            p = NULL;
            break;

        case CMD_GET_COUNTERS:
            mainLoop.push(p);
            p = NULL;
            break;

        default:
            cnt.inc(rxErrorUnknown);
            counterUpdate = true;
            break;
        }
        if (p != NULL)
        {
            freePacket(p);
        }
    } else {
        // Packet was not for this processor
        rxPacketForwarding(p);
    }
    return true;
}

void Telemetry::rxPacketForwarding(packet *p)
{
    // There is no paket forwarding implemented, so free buffer.
    freePacket(p);
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
    cnt.inc(rxBytes);
    switch (rxState)
    {
    case RS_BEGIN:
        // Waiting for 0x7e to arrive
        if (b == FRAME_START_STOP)
        {
            rxReInitPacket();
            rxState = RS_DATA;
        } else {
            cnt.inc(rxErrorDropped);
            counterUpdate = true;
        }
        break;

    case RS_DATA:
        if (b == FRAME_DATA_ESCAPE)
        {
            rxState = RS_ESCAPE;
        } else if (b == FRAME_START_STOP)
        {
            rxEndOfPacketHandling();
        } else {
            rxCalcChecksum(b);
            rxSaveByte(b);
        }
        break;

    case RS_ESCAPE:
        rxSaveByte(b ^ FRAME_XOR);
        rxCalcChecksum(b);
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
        freePacket(txCurrentPacket);
        txCurrentPacket = NULL;
        cnt.inc(txPackets);
    }
    p = txGetPacketFromQueues();
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
 * Pull bytes one-by-one from buffer with framing, stuffing and checksum calculation.
 * This function drives the whole tx side, from pulling messages from queues to
 * sending packetized records to RPi.
 *
 * @return true if a byte is available for transmission.
 * @param b the variable where byte is returned.
 */
bool Telemetry::txGetPacketByte(byte &b)
{
    switch (txState)
    {
    case TS_BEGIN:
        b = FRAME_START_STOP;
        txState = TS_DATA;
        break;

    case TS_DATA:
        b = txCurrentPacket->raw[txCurrentOffset++];
        crcUpdate(txChecksum, b);
        if ((b == FRAME_DATA_ESCAPE)||(b == FRAME_START_STOP))
        {
            txEscByte = b ^ FRAME_XOR;
            b = FRAME_DATA_ESCAPE;
            txState = TS_ESCAPE;
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
        if ((b == FRAME_DATA_ESCAPE)||(b == FRAME_START_STOP))
        {
            txEscByte = b ^ FRAME_XOR;
            b = FRAME_DATA_ESCAPE;
            txState = TS_CHECKSUM_ESC;
        } else {
            txState = TS_END;
        }
        break;

    case TS_CHECKSUM_ESC:
        b = txEscByte;
        txState = TS_END;
        break;

    case TS_END:
        b = FRAME_START_STOP;  // This will serve as separator between packets
        if (txEndOfPacketHandling())
        {
            // A new packet is ready to go, the framing byte is already delivered here
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
            // No data available - stay in IDLE mode
            return false;
        }
    }
    // Checksum is calculated over all bytes sent from FRAME_START_STOP up to
    // but not including the checksum byte preceeding the closing FRAME_START_STOP
    return true;
}

inline void Telemetry::crcUpdate(uint16_t &chksum, byte b)
{
    chksum += b; //0-1FF
    chksum += (txChecksum >> 8); //0-100
    chksum &= 0x00ff;
}

packet* Telemetry::getMainLoopPacket()
{
    if (!mainLoop.isEmpty())
    {
        return (packet*)mainLoop.pop();
    }
    return NULL;
}

void Telemetry::freePacket(packet *p)
{
    if (p != NULL)
    {
        freeList.push(p);
    }
}

packet* Telemetry::getEmptyPacket()
{
    if (!freeList.isEmpty())
    {
        return (packet*)freeList.pop();
    } else {
        cnt.inc(txErrorNoBuf);
        counterUpdate = true;
    }
    return NULL;
}

void Telemetry::putMainLoopPacket(packet *p)
{
    if (p != NULL)
    {
        mainLoop.push(p);
    }
}

void Telemetry::errorCounter(uint32_t count, char const *name)
{
    packet *p = getEmptyPacket();
    if (p)
    {
        p->ec.hdr.dst = ADDR_RPI;
        p->ec.hdr.src = ADDR_TEENSY;
        p->ec.hdr.cmd = CMD_ERROR_COUNT;
        p->ec.hdr.reserved = 0;
        p->ec.count = count;
        strncpy(p->ec.name, name, sizeof(p->ec.name));
        priorityQueue.push(p);
    }
}

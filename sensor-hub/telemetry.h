
#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>
#include "common.h"
#include <QueueList.h>

class Telemetry
{
public:
    Telemetry(HardwareSerial port, unsigned speed);
    void serialPolling();
    void printErrorCounters(HardwareSerial out) const;
    void wheelEvent(rot_one left, rot_one right);
    void sonarEvent(byte sensor, uint16_t distance, uint32_t when);
    void sendPing(bool &ready, uint32_t &delay);

private:
    enum receiveStates {
        RS_BEGIN,  // Waiting for framing 0x7e
        RS_DATA,   // Incoming data
        RS_ESCAPE, // Received 0x7d - XOR next byte with 0x20
    };

    enum transmitStates {
        TS_BEGIN,  // Nothing sent yet, deliver 0x7e
        TS_DATA,   // Sending normal data
        TS_ESCAPE, // Escape has been sent, escByte is next
        TS_CHKSUM, // Last data byte sent, checksum is next
        TS_CHECKSUM_ESC, // checksum needs escaping
        TS_END,    // Checksum sent, frame is next. Can be skipped if there is a next packet in queue
        TS_IDLE,   // No data to transmit.
    };

    HardwareSerial serialPort;
    // Outgoing packet queues
    QueueList <packet *> priorityQueue;  // allow for 4
    QueueList <packet *> rotationQueue;  // allow for 2
    QueueList <packet *> sonarQueue[MAX_NO_OF_SONAR]; // allow for 6+2
    QueueList <packet *> freeList;

    // RX data housekeeping
    packet *rxCurrentPacket;  // Packet being received
    receiveStates rxState;    // state of packet reception
    uint16_t rxCurrentOffset; // Bytes received so far
    uint16_t rxChecksum;      // Checksum so far
    // Error counters
    uint32_t rxErrorChecksum; // Number of packets with checksum errors
    uint32_t rxErrorTooShort; // Number of too short packets (not even a header)
    uint32_t rxErrorTooLong;  // Number of packets exceeding max length.
    uint32_t rxErrorBuffer;   // No receive buffer available
    uint32_t rxErrorDropped;  // Number of bytes thrown away while looking for sync
    uint32_t rxErrorUnknown;  // Unknown command
    uint32_t txErrorNoBuf;    // No free tx buffers
    uint32_t txInfoSonarDrop;
    uint32_t txInfoWheelDrop;

    // TX data housekeeping
    packet *txCurrentPacket;  // The currently transmitting packet
    uint16_t txTotalSize;     // Total number of bytes (payload - note escaped) in bufefr
    uint16_t txCurrentOffset; // Current offset for transmission
    transmitStates txState;   // State for packet transmission
    byte txEscByte;           // stuffed byte, i.e. value xor'ed with 0x20 to transmit.
    uint16_t txChecksum;      // TX checksum

    // Protocol housekeeping
    bool *pingReady;
    uint32_t *pingDelay;

    // Static array of packet buffers to flow around the queues
    static const unsigned bufferCount = 4+2+6+2;
    packet packetPool[bufferCount];

    // Mechanism to cycle between available output queues
    uint16_t sequenceIdx;
    static const unsigned sequenceMax = MAX_NO_OF_SONAR+1; // sonars + wheels
    QueueList <packet *> * queueSequence[sequenceMax];  // This is used to cycle between queues

    // Functions
    void initQueues();
    packet *getPacketFromQueues();
    size_t getPacketLength(packet *p);
    inline void crcUpdate(uint16_t &chksum, byte b);
    bool rxGetBuffer();
    void rxHandleUartByte(byte b);
    void rxCalcChecksum(byte b);
    bool rxEndOfPacketHandling();
    void rxInitPacket();
    void rxSaveByte(byte b);
    void rxPacketForwarding(packet *p);
    bool txEndOfPacketHandling();
    bool txGetPacketByte(byte &b);
};

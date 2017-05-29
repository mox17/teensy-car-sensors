
#ifndef TELEMETRY_H
#define TELEMETRY_H
#include <Arduino.h>
#include <HardwareSerial.h>
#include "common.h"
#include "queuelist.h"

static const unsigned WHEEL_EVENT_MIN_INTERVAL = 5;

class Telemetry
{
public:
    Telemetry(HardwareSerial port, unsigned speed);
    void serialPolling();
    void wheelEvent(rot_one left, rot_one right);
    void sonarEvent(packet *sonarPacket);
    void sendPing(bool &ready, uint32_t &delay);
    packet *getMainLoopPacket();
    void freePacket(packet *p);
    packet *getEmptyPacket();
    void putMainLoopPacket(packet *p);
    void errorCounter(uint32_t count, char const *name);

private:
    enum receiveStates {
        RS_BEGIN,  //!< Waiting for framing 0x7e
        RS_DATA,   //!< Incoming data
        RS_ESCAPE, //!< Received 0x7d - XOR next byte with 0x20
    };

    enum transmitStates {
        TS_BEGIN,  //!< Nothing sent yet, deliver 0x7e
        TS_DATA,   //!< Sending normal data
        TS_ESCAPE, //!< Escape has been sent, escByte is next
        TS_CHKSUM, //!< Last data byte sent, checksum is next
        TS_CHECKSUM_ESC, //!< checksum needs escaping
        TS_END,    //!< Checksum sent, frame is next. Can be skipped if there is a next packet in queue
        TS_IDLE,   //!< No data to transmit.
    };

    HardwareSerial serialPort;
    // Outgoing packet queues
    queuelist priorityQueue;  // allow for 4
    queuelist rotationQueue;  // allow for 2
    queuelist sonarQueue[MAX_NO_OF_SONAR]; // allow for 6+2
    // Free buffers
    queuelist freeList;
    // Internal processing queue
    queuelist mainLoop;

    // RX data housekeeping
    packet *rxCurrentPacket;  //!< Packet being received
    receiveStates rxState;    //!< state of packet reception
    uint16_t rxCurrentOffset; //!< Bytes received so far
    uint16_t rxChecksum;      //!< Checksum so far
    // Error counters
    bool counterUpdate;

    // TX data housekeeping
    packet *txCurrentPacket;  //!< The currently transmitting packet
    uint16_t txTotalSize;     //!< Total number of bytes (payload - note escaped) in bufefr
    uint16_t txCurrentOffset; //!< Current offset for transmission
    transmitStates txState;   //!< State for packet transmission
    byte txEscByte;           //!< stuffed byte, i.e. value xor'ed with 0x20 to transmit.
    uint16_t txChecksum;      //!< TX checksum

    // Protocol housekeeping
    bool *pingReady;
    uint32_t *pingDelay;

    // Static array of packet buffers to flow around the queues
    static const unsigned bufferCount = 32;
    packet packetPool[bufferCount];

    // Mechanism to cycle between available output queues
    uint16_t sequenceIdx;
    static const unsigned sequenceMax = MAX_NO_OF_SONAR+1; // sonars + wheels
    queuelist* queueSequence[sequenceMax];  // This is used to cycle between queues

    // Functions
    void initQueues();
    packet *txGetPacketFromQueues();
    size_t getPacketLength(packet *p);
    inline void crcUpdate(uint16_t &chksum, byte b);
    bool rxGetBuffer();
    void rxHandleUartByte(byte b);
    void rxCalcChecksum(byte b);
    bool rxEndOfPacketHandling();
    void rxReInitPacket();
    void rxSaveByte(byte b);
    void rxPacketForwarding(packet *p);
    bool txEndOfPacketHandling();
    bool txGetPacketByte(byte &b);
};

#endif // TELEMETRY_H

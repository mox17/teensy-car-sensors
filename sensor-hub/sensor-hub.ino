/**
 * @brief Hall-effect and ultrasound sensors processing
 */
#include "sonararray.h"
#include "rotation.h"
#include "telemetry.h"
#include "counters.h"

const int ledPin   = 13;  // Flash LED on hall-effect state change

// Hall-effect pin configuration
const int hallPinL1 = 23;
const int hallPinL2 = 22;
const int hallPinR1 = 21;
const int hallPinR2 = 20;

// Prototypes
void mainSonarReport(int id, int value, unsigned long time_in_ms);
void mainWheelReport(rot_one left, rot_one right);

const int maxDistance = 200; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
const int ultraSound1TrigPin = 2;
const int ultraSound2TrigPin = 3;
const int ultraSound3TrigPin = 4;
const int ultraSound4TrigPin = 5;
const int ultraSound5TrigPin = 6;
const int ultraSound6TrigPin = 7;
const int sonarPins[] = {ultraSound1TrigPin,
                         ultraSound2TrigPin,
                         ultraSound3TrigPin,
                         ultraSound4TrigPin,
                         ultraSound5TrigPin,
                         ultraSound6TrigPin};
const int sonarCount = 6;

volatile unsigned int sonarResults[MAX_NO_OF_SONAR] = {0};// Raw timing data in microseconds
volatile unsigned int sonarCounts[MAX_NO_OF_SONAR] = {0}; // Number of reports
volatile unsigned long sonarTiming[MAX_NO_OF_SONAR] = {0}; // Timestamp;
volatile int sonarUpdate = 0;      // sonar event counter - updated from interrupt context
int sonarTrack = 0;                // Compare with sonarUpdate to determine updates
SonarArray sonarArray(sonarCount, sonarPins, maxDistance, mainSonarReport);
// Setup serial to RPi
Telemetry messageHandling(Serial1, 115200);
// Wheel sensor setup
Axle axle = Axle(mainWheelReport);

unsigned loopTimer;

void setup()
{
    Serial.begin(115200);  // USB serial traces
    pinMode(ledPin, OUTPUT);

    axle.config(hallPinL1, hallPinL2, hallPinR1, hallPinR2);
    loopTimer = millis(); // Start now.
    sonarArray.startSonar();
    // Control ultrasound sensor sequencing
    //int sequence[] = {2,1,1,0,0,0,1,1};
    //sonarArray.setSequence(8, sequence);
}

uint32_t errorCounterTime = 0;
uint32_t wheelCounterTime = 0;
uint32_t sonarPingInterval = SONAR_PING_INTERVAL;
uint32_t sonarDelayTime = 0;
bool sonarDelayFlag = false;
bool newWheelData = false;

void handlePingError()
{
    switch (sonarArray.getId())
    {
    case 0 : cnt.inc(errPing0); break;
    case 1 : cnt.inc(errPing1); break;
    case 2 : cnt.inc(errPing2); break;
    case 3 : cnt.inc(errPing3); break;
    case 4 : cnt.inc(errPing4); break;
    case 5 : cnt.inc(errPing5); break;
    }
}

void loop()
{
    uint32_t now = millis();
    bool newSonarData = (sonarTrack != sonarUpdate);

    if (now > errorCounterTime)
    {
        errorCounterTime = millis() + 1000;
        //cnt.printNZ();
        //cnt.sendNZ();
        if (sonarArray.getState() == SonarArray::SONAR_PING_ERROR)
        {
            sonarArray.startSonar(); // Try a restart
        }
    }

    // Wheel slow speed handling
    if (now > wheelCounterTime)
    {
        wheelCounterTime = now + WHEEL_MAX_INTERVAL_MS;
        axle.timeout();
    }

    if (sonarDelayFlag)
    {
        if (now > sonarDelayTime)
        {
            sonarDelayFlag = false;
            if (!sonarArray.nextSonar())
            {
                handlePingError();
                // Schedule a retry
                packet *retry = messageHandling.getEmptyPacket();
                retry->hdr.dst = ADDR_TEENSY;
                retry->hdr.src = ADDR_TEENSY;
                retry->hdr.cmd = CMD_SONAR_RETRY;
                retry->hdr.reserved = 0;
                messageHandling.putMainLoopPacket(retry);
            }
        }
    }
    // Drive the RX/TX messaging queues towards RPi
    messageHandling.serialPolling();
    // Handle main message queue
    handleMessageQueue();
}

/**
 * @brief Handling of messages sent to main loop.
 *
 * These messages can originate from RPi, or they can originate from
 * interrupt handling.
 */
void handleMessageQueue()
{
    byte id;

    packet *p = messageHandling.getMainLoopPacket();
    if (p != NULL)
    {
        switch ((command)p->hdr.cmd)
        {
        case CMD_PONG_RESP:
            Serial.print("PONG received, delay=");
            Serial.println(millis() - p->pp.timestamp1);
            break;

        case CMD_SET_SONAR_SEQ:
            if (p->sq.len <= 4*MAX_NO_OF_SONAR)
            {
                sonarArray.setSequence(p->sq.len, p->sq.sequence);
            } else {
                cnt.inc(badSeqCommand);
            }
            break;

        case CMD_SONAR_STOP:
            sonarArray.stopSonar();
            break;

        case CMD_SONAR_START:
            sonarArray.startSonar();
            sonarArray.nextSonar();
            break;

        case CMD_SONAR_STATUS:
            id = p->ds.sensor;
            if (id < MAX_NO_OF_SONAR)
            {
                sonarResults[id] = p->ds.distance;
                sonarTiming[id] = p->ds.when;
                sonarCounts[id]++;
                sonarUpdate++;
                // Put message on UART TX queue
                messageHandling.sonarEvent(p);
                p = NULL;
                // Start next sonar after a delay
                sonarDelayTime = millis() + sonarPingInterval;
                sonarDelayFlag = true;
            } else {
                cnt.inc(badSonarId);
            }
            break;

        case CMD_WHEEL_RESET:
            axle.resetOdometer();
            break;

        case CMD_GET_COUNTERS:
            cnt.sendNZ();
            break;

        case CMD_SONAR_RETRY:
            // Try starting the sonars again...
            if (!sonarArray.nextSonar())
            {
                handlePingError();
                // Next retry
                messageHandling.putMainLoopPacket(p);
                p = NULL;
            }
            break;

        case CMD_SONAR_WAIT:
            sonarPingInterval = p->sw.pause;
            break;

        default:
            Serial.print("Unknown CMD ");
            Serial.println(p->hdr.cmd);
            cnt.inc(unknownCommand);
            break;
        }
        if (p != NULL)
        {
            messageHandling.freePacket(p);
        }
    }
}

void rotationStatus()
{
    Serial.print(" ");
    Serial.print(axle.direction());
    Serial.print(" ");
    Serial.print(axle.odometer());
    Serial.print(" ");
    Serial.print(axle.pulsePerSec());
}

void sonarStatus()
{
    Serial.print(" > ");
    for (int i=0;i < sonarCount; i++) {
        Serial.print(sonarResults[i] / US_ROUNDTRIP_CM); // Ping returned, uS result in ping_result, convert to cm with US_ROUNDTRIP_CM.
        Serial.print(" cm ");
        //Serial.print(sonarCounts[i]);
    }
}

/**
 * @brief Callback from sonar timer handler - be quick!
 */
void mainSonarReport(int id, int value, unsigned long time_in_ms)
{
    //Serial.print('R');
    packet *p = messageHandling.getEmptyPacket();
    if (p)
    {
        p->ds.hdr.dst = ADDR_TEENSY;
        p->ds.hdr.src = ADDR_TEENSY;
        p->ds.hdr.cmd = CMD_SONAR_STATUS;
        p->ds.hdr.reserved = 0;
        p->ds.sensor = id;
        p->ds.distance = value;
        p->ds.when = time_in_ms;
        messageHandling.putMainLoopPacket(p);
    }
}

/**
 * @brief Callback from wheel sensor interrupt handler
 */
void mainWheelReport(rot_one left, rot_one right)
{
    messageHandling.wheelEvent(left, right);
}

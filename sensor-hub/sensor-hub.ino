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

// Sonar setup
void mainSonarReport(int id, int value, unsigned long time_in_ms);

const int maxDistance = 200; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
const int ultraSound1TrigPin = 2;
const int ultraSound2TrigPin = 3;
const int ultraSound3TrigPin = 4;
const int sonarPins[] = {ultraSound1TrigPin, ultraSound2TrigPin, ultraSound3TrigPin};
const int sonarCount = 3;//sizeof(sonarPins)/sizeof(sonarPins[0]);

volatile unsigned int sonarResults[MAX_NO_OF_SONAR] = {0};// Raw timing data in microseconds
volatile unsigned int sonarCounts[MAX_NO_OF_SONAR] = {0}; // Number of reports
volatile unsigned long sonarTiming[MAX_NO_OF_SONAR] = {0}; // Timestamp;
volatile int sonarUpdate = 0;      // sonar event counter - updated from interrupt context
int sonarTrack = 0;                // Compare with sonarUpdate to determine updates
SonarArray sonarArray(sonarCount, sonarPins, maxDistance, mainSonarReport);

unsigned loopTimer;
unsigned rotationUpdate=0;  // When was last time a wheel update was done

// Setup serial to RPi
Telemetry messageHandling(Serial1, 115200);
RotCalc rotLeft = RotCalc(ROT_LEFT);
RotCalc rotRight = RotCalc(ROT_RIGHT);

void setup()
{
    Serial.begin(115200);
    pinMode(ledPin, OUTPUT);

    // Wheel sensor configuration
    Rotation(hallPinL1, hallPinL2, hallPinR1, hallPinR2);
    loopTimer = millis(); // Start now.
    sonarArray.startSonar();
    // Control ultrasound sensor sequencing
    //int sequence[] = {2,1,1,0,0,0,1,1};
    //sonarArray.setSequence(8, sequence);
}

uint32_t errorCounterTime;
bool newWheelData = false;

void loop()
{
    bool newSonarData = (sonarTrack != sonarUpdate);

    if (millis() > errorCounterTime)
    {
        errorCounterTime = millis() + 1000;
        cnt.printNZ();
        cnt.sendNZ();
        if (sonarArray.getState() == SonarArray::SONAR_PING_ERROR)
        {
            // Try a restart
            sonarArray.startSonar();
        }
    }

    rotLeft.handleBuffer();
    rotRight.handleBuffer();
    if (rotLeft.newData() || rotRight.newData())
    {
        // When there is new data put a message on the queue for
        // transmission to RPi
        rot_one l, r;
        rotLeft.rotGetRec(l);
        rotRight.rotGetRec(r);
        messageHandling.wheelEvent(l, r);
        newWheelData = true;
    }

    // Display when new data is available
    if (false && (newWheelData || newSonarData))
    {
        rotationStatus();
        sonarStatus();
        sonarTrack = sonarUpdate;
        Serial.println("");
        newWheelData = false;
    }
    // Drive the RX/TX messaging queues towards RPi
    messageHandling.serialPolling();
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
            } else {
                cnt.inc(badSonarId);
            }
            if (!sonarArray.nextSonar())
            {
                // Schedule a retry
                packet *retry = messageHandling.getEmptyPacket();
                retry->hdr.dst = ADDR_TEENSY;
                retry->hdr.src = ADDR_TEENSY;
                retry->hdr.cmd = CMD_SONAR_RETRY;
                retry->hdr.reserved = 0;
                messageHandling.putMainLoopPacket(retry);
            }
            break;

        case CMD_WHEEL_RESET:
            rotLeft.resetOdometer();
            rotRight.resetOdometer();
            break;

        case CMD_GET_COUNTERS:
            cnt.sendNZ();
            break;

        case CMD_SONAR_RETRY:
            // Try starting the sonars again...
            if (!sonarArray.nextSonar())
            {
                messageHandling.putMainLoopPacket(p);
                p = NULL;
            }
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
    rotationUpdate = millis();
    Serial.print(" ");
    Serial.print(rotLeft.direction());
    Serial.print(" ");
    Serial.print(rotLeft.odometer());
    Serial.print(" ");
    Serial.print(rotLeft.pulsePerSec());
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

/**
 * @brief Hall-effect and ultrasound sensors processing
 */
#include "sonararray.h"
#include "rotation.h"
#include "telemetry.h"

void sonarReport(int id, int value, unsigned long time_in_ms);

// Hall-effect pin configuration
const int ledPin   = 13;  // Flash LED on hall-effect state change
const int hallPinL1 = 5;
const int hallPinL2 = 6;
const int hallPinR1 = -1;
const int hallPinR2 = -1;

// Sonar setup
const int maxDistance = 200; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
const int ultraSound1TrigPin = 10;
const int ultraSound2TrigPin = 11;
const int ultraSound3TrigPin = 12;
const int sonarPins[] = {ultraSound1TrigPin, ultraSound2TrigPin, ultraSound3TrigPin};
const int sonarCount = 3;//sizeof(sonarPins)/sizeof(sonarPins[0]);

volatile unsigned int sonarResults[MAX_NO_OF_SONAR] = {0};// Raw timing data in microseconds
volatile unsigned int sonarCounts[MAX_NO_OF_SONAR] = {0}; // Number of reports
volatile unsigned long sonarTiming[MAX_NO_OF_SONAR] = {0}; // Timestamp;
volatile int sonarUpdate = 0;      // sonar event counter - updated from interrupt context
int sonarTrack = 0;                // Compare with sonarUpdate to determine updates
SonarArray sonarArray(sonarCount, sonarPins, maxDistance, sonarReport);

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
    //sonarArray.startSonar();
    // Control ultrasound sensor sequencing
    //int sequence[] = {2,1,1,0,0,0,1,1};
    //sonarArray.setSequence(8, sequence);
    Serial.print("header:");
    Serial.println(sizeof(header));
    Serial.print("rot_one:");
    Serial.println(sizeof(rot_one));
    Serial.print("rotation:");
    Serial.println(sizeof(rotation));
    Serial.print("packet:");
    Serial.println(sizeof(packet));
}

uint32_t errorCounterTime;

void loop()
{
    bool newSonarData = (sonarTrack != sonarUpdate);

    if (millis() > errorCounterTime)
    {
        errorCounterTime = millis() + 1000;
        messageHandling.printErrorCounters(Serial1);
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
    }

    // Display when new data is available
    if (rotLeft.newData() || rotRight.newData() || newSonarData)
    {
        rotationStatus();
        sonarStatus();
        sonarTrack = sonarUpdate;
        Serial.println("");
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
        case CMD_PONG:
            Serial.print("PONG received, delay=");
            Serial.println(millis() - p->pp.timestamp1);
            break;

        case CMD_US_SET_SEQ:
            if (p->sq.len <= 4*MAX_NO_OF_SONAR)
            {
                sonarArray.setSequence(p->sq.len, p->sq.sequence);
            } else {
                // TODO malformed command error counter
            }
            break;

        case CMD_US_STOP:
            sonarArray.stopSonar();
            break;

        case CMD_US_START:
            sonarArray.startSonar();
            break;

        case CMD_US_STATUS:
            id = p->ds.sensor;
            if (id < MAX_NO_OF_SONAR)
            {
                sonarResults[id] = p->ds.distance;
                sonarTiming[id] = p->ds.when;
                sonarCounts[id]++;
                sonarUpdate++;
                // Put message on TX queue
                messageHandling.sonarEvent(p);
                p = NULL;
            } else {
                // TODO malformed command error counter
            }
            break;

        case CMD_ROT_RESET:
            rotLeft.resetOdometer();
            rotRight.resetOdometer();
            break;

        default:
            Serial.print("Unknown CMD ");
            Serial.println(p->hdr.cmd);
            // TODO malformed command error counter
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
    for (int i=0;i < sonarCount; i++) {
        Serial.print(" ");
        Serial.print(sonarResults[i] / US_ROUNDTRIP_CM); // Ping returned, uS result in ping_result, convert to cm with US_ROUNDTRIP_CM.
        Serial.print(" cm ");
        Serial.print(sonarCounts[i]);
    }
}

/**
 * @brief Callback from sonar timer handler - be quick!
 *
 */
void sonarReport(int id, int value, unsigned long time_in_ms)
{
    Serial1.print('=');
    packet *p = messageHandling.getEmptyPacket();
    if (p)
    {
        p->ds.hdr.dst = ADDR_TEENSY;
        p->ds.hdr.src = ADDR_TEENSY;
        p->ds.hdr.cmd = CMD_US_STATUS;
        p->ds.hdr.reserved = 0;
        p->ds.sensor = id;
        p->ds.distance = value;
        p->ds.when = time_in_ms;
        messageHandling.putMainLoopPacket(p);
    }
}

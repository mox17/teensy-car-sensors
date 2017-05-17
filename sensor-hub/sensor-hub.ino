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
SonarArray sa(sonarCount, sonarPins, maxDistance, sonarReport);

unsigned loopTimer;

// Setup serial to RPi
Telemetry messageHandling(Serial1, 115200);

unsigned rotationUpdate=0;  // When was last time a wheel update was done

RotCalc rotLeft = RotCalc(ROT_LEFT);
RotCalc rotRight = RotCalc(ROT_RIGHT);

void setup() 
{
    Serial.begin(115200);
    pinMode(ledPin, OUTPUT);
  
    // Wheel sensor configuration
    Rotation(hallPinL1, hallPinL2, hallPinR1, hallPinR2);
    loopTimer = millis(); // Start now.
  
    sa.startSonar();
    // Control ultrasound sensor sequencing
    //int sequence[] = {2,1,1,0,0,0,1,1};
    //sa.setSequence(8, sequence);
}

void loop()
{
    bool sonar = (sonarTrack != sonarUpdate);

    rotLeft.calculate();
    rotRight.calculate();
    if (rotLeft.newData() || rotRight.newData())
    {
        rot_one l,r;
        rotLeft.rotGetRec(l);
        rotRight.rotGetRec(r);
        messageHandling.wheelEvent(l,r);
    }
  
    // Display when new data is available
    if (rotLeft.newData() || rotRight.newData() || sonar) {
        rotationStatus();
        sonarStatus();
        sonarTrack = sonarUpdate;
        Serial.println("");
    }
    // Drive the RX/TX messaging queues towards RPi
    messageHandling.serialPolling();
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

// Callback from sonar timer handler - be quick!
void sonarReport(int id, int value, unsigned long time_in_ms)
{
    Serial1.print(char(48+id));
    sonarResults[id] = value;
    sonarTiming[id] = time_in_ms;
    sonarCounts[id]++;
    sonarUpdate++;
}

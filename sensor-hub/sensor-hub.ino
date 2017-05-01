/* 
 *  Hall-effect sensors and ultrasound sensors
 */
#include "sonararray.h"
#include "rotation.h"
#include <RingBuf.h>

// Hall-effect pin configuration
const int ledPin   = 13;  // Flash LED on hall-effect state change 
const int hallPinL1 = 5;
const int hallPinL2 = 6;
const int hallPinR1 = -1;
const int hallPinR2 = -1;
void rotationReport(uint8_t sensor, bool level, bool direction );

// Hall effect variables
volatile int tCount = 0;      // Total state changes
volatile bool forward = true; // Determined from phase between two sensors (overlapping pulses from sensors expected)


// Sonar setup
const int maxDistance = 200; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
const int ultraSound1TrigPin = 10; 
const int ultraSound2TrigPin = 11;
const int ultraSound3TrigPin = 12;
const int sonarPins[] = {ultraSound1TrigPin, ultraSound2TrigPin, ultraSound3TrigPin};
const int sonarCount = 3;//sizeof(sonarPins)/sizeof(sonarPins[0]);

unsigned int sonarResults[6] = {0};  // Raw timing data in microseconds
int sonarUpdate = 0;         // sonar event counter
SonarArray *sa;

// General purpose variable
unsigned loopTimer;

void sonarSetup()
{
    sa = new SonarArray(sonarCount, sonarPins, maxDistance, sonarReport);
}


void setup() 
{
    Serial.begin(115200);
    pinMode(ledPin, OUTPUT);
  
    Rotation(hallPinL1, hallPinL2, hallPinR1, hallPinR2, rotationReport);
    loopTimer = millis(); // Start now.
  
    sonarSetup();
    sa->startSonar();
    // Control ultrasound sensor sequencing
    int sequence[] = {2,1,1,0,0,0,1,1};
    //sa->setSequence(8, sequence);
}

int rotTrack = 0;
int sonarTrack = 0;
unsigned rotationUpdate=0;

void loop()                     
{
    bool rotation = rotTrack != tCount;
    bool sonar = sonarTrack != sonarUpdate;
  
    // Display when new data is available
    if (rotation || sonar) {
        rotationStatus();
        rotTrack = tCount;
        sonarStatus();
        sonarTrack = sonarUpdate;
        Serial.println("");
    }
  
    // This is needed to restart the measuring sequence if a sensor fails to return a result.
    if ((millis() - rotationUpdate) > 100)
    {
        Serial.println("restart sonar");
        sa->startSonar();
    }
}

void rotationStatus()
{
    rotationUpdate = millis();
    Serial.print(" ");
    Serial.print(RotationGetDirectionL());
    Serial.print(" ");
    Serial.print(tCount);
}

void sonarStatus()
{
    for (int i=0;i < sonarCount; i++) {
        Serial.print(" ");
        Serial.print(sonarResults[i] / US_ROUNDTRIP_CM); // Ping returned, uS result in ping_result, convert to cm with US_ROUNDTRIP_CM.
    }
    Serial.print("cm");
}

// Callback from sonar timer handler - be quick!
void sonarReport(int id, int value, unsigned long time_in_ms)
{
    sonarResults[id] = value;
    sonarUpdate++;
}


// Callback from interrupt handler - be quick
void rotationReport(uint8_t sensor, bool level, bool direction )
{
    tCount++;
    digitalWrite(ledPin, tCount & 1);

    if (2 & sensor)
    {
        // Right hand side
    }
    else
    {
        // Left hand side
    }
}


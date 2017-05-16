/* 
 *  Hall-effect sensors and ultrasound sensors
 */
#include "sonararray.h"
#include "rotation.h"

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

unsigned int sonarResults[6] = {0};// Raw timing data in microseconds
unsigned int sonarCounts[6] = {0}; // Number of reports
volatile int sonarUpdate = 0;      // sonar event counter - updated from interrupt context
SonarArray *sa;

unsigned loopTimer;

void sonarSetup()
{
    sa = new SonarArray(sonarCount, sonarPins, maxDistance, sonarReport);
}


void setup() 
{
    // Setup serial to RPi
    Serial1.begin(115200);
    Serial1.println("testing...");

    Serial.begin(115200);
    pinMode(ledPin, OUTPUT);
  
    Rotation(hallPinL1, hallPinL2, hallPinR1, hallPinR2);
    Serial.println("Hello1");
    loopTimer = millis(); // Start now.
  
    sonarSetup();

    sa->startSonar();
    // Control ultrasound sensor sequencing
    //int sequence[] = {2,1,1,0,0,0,1,1};
    //sa->setSequence(8, sequence);
    Serial.println("Hello3");
}

int sonarTrack = 0;
unsigned rotationUpdate=0;

RotCalc rotLeft = RotCalc(ROT_LEFT);
RotCalc rotRight = RotCalc(ROT_RIGHT);

void loop()
{
    bool sonar = (sonarTrack != sonarUpdate);

    rotLeft.calculate();
    rotRight.calculate();
  
    // Display when new data is available
    if (rotLeft.newData() || rotRight.newData() || sonar) {
        rotationStatus();
        sonarStatus();
        sonarTrack = sonarUpdate;
        Serial.println("");
    }
  
    // This is needed to restart the measuring sequence if a sensor fails to return a result.
    if ((millis() - rotationUpdate) > 100)
    {
        //Serial.println("restart sonar");
        //sa->startSonar();
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

// Callback from sonar timer handler - be quick!
void sonarReport(int id, int value, unsigned long time_in_ms)
{
    Serial1.print(char(48+id));
    sonarResults[id] = value;
    sonarCounts[id]++;
    sonarUpdate++;
}

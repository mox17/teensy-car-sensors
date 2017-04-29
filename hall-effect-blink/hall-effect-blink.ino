/* 
 *  Hall-effect sensors and ultrasound sensors
 */
#include <NewPing.h>
#include "sonararray.h"

// Hall-effect pin configuration
const int ledPin   = 13;  // Flash LED on hall-effect state change 
const int hallPin1 = 5;
const int hallPin2 = 6;

// Hall effect variables
volatile int tCount = 0;      // Total state changes
volatile bool forward = true; // Determined from phase between two sensors (overlapping pulses from sensors expected)
volatile bool flip=false;

// Sonar setup
const int maxDistance = 200; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
const int ultraSound1TrigPin = 10; 
const int ultraSound2TrigPin = 11;
const int ultraSound3TrigPin = 12;
const int sonarPins[] = {ultraSound1TrigPin, ultraSound2TrigPin, ultraSound3TrigPin};
const int sonarCount = sizeof(sonarPins)/sizeof(sonarPins[0]);

unsigned int sonarResults[6] = {0};  // Raw timing data
int sonarUpdate = 0;         // sonar event counter

SonarArray *sa;

// General purpose variable
unsigned loopTimer;

void hallEffectSetup()
{
  pinMode(hallPin1, INPUT_PULLUP);
  pinMode(hallPin2, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(hallPin1), hallChange1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(hallPin2), hallChange2, CHANGE);
}

void sonarSetup()
{
    sa = new SonarArray(sonarCount, sonarPins, maxDistance, sonarReport);
}

void setup() 
{
  Serial.begin(115200);
  hallEffectSetup();
  loopTimer = millis(); // Start now.
  sonarSetup();
  sa->startSonar();
}

int rotTrack = 0;
int sonarTrack = 0;

void loop()                     
{
  // Display when new data is available
  if ((rotTrack != tCount) || (sonarTrack != sonarUpdate)) {
    rotationStatus();
    rotTrack = tCount;
    sonarStatus();
    sonarTrack = sonarUpdate;
    Serial.println("");
  }
}

void rotationStatus()
{
  Serial.print(" ");
  Serial.print(forward);
  Serial.print(" ");
  Serial.print(tCount);
}

int hStatus1=0;
int hStatus2=0;

// ISR 
void hallChange1() 
{
  hStatus1 = digitalRead(hallPin1);
  tCount++;
  flip = !flip;
  digitalWrite(ledPin, flip);
}

// ISR 
void hallChange2() 
{
  hStatus2 = digitalRead(hallPin2);
  forward = hStatus1 == hStatus2;
  tCount++;
  flip = !flip;
  digitalWrite(ledPin, flip);
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


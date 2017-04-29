/* 
 *  Hall-effect sensors and ultrasound sensors
 */
#include <NewPing.h>

// Hall-effect pin configuration
const int ledPin   = 13;  // Flash LED on hall-effect state change 
const int hallPin1 = 5;
const int hallPin2 = 6;

// Hall effect variables
volatile int hCount = 0;      // high (rise) count
volatile int lCount = 0;      // low (fall) count
volatile int tCount = 0;      // Total state changes
volatile bool forward = true; // Determined from phase between two sensors (overlapping pulses from sensors expected)
volatile bool flip=false;

// Sonar setup
const int maxDistance = 200; // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

const int ultraSound1TrigPin = 10; 
const int ultraSound1EchoPin = 10;

const int ultraSound2TrigPin = 11;
const int ultraSound2EchoPin = 11;

const int ultraSound3TrigPin = 12;
const int ultraSound3EchoPin = 12;

const unsigned int sonarCount = 3;

NewPing sonarArray[sonarCount] = {
  NewPing(ultraSound1TrigPin, ultraSound1EchoPin, maxDistance),
  NewPing(ultraSound2TrigPin, ultraSound2EchoPin, maxDistance),
  NewPing(ultraSound3TrigPin, ultraSound3EchoPin, maxDistance)
};

unsigned int pingSpeed = 80; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
unsigned long pingTimer;     // Holds the next ping time.
unsigned int sonar = 0;      // Index active sonar 0..(sonarCount-1)
NewPing *currentSonar = &sonarArray[0];
unsigned int sonarResults[sonarCount] = {0};  // Raw timing data
int sonarUpdate = 0;         // sonar event counter

void hallEffectSetup()
{
  pinMode(hallPin1, INPUT_PULLUP);
  pinMode(hallPin2, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(hallPin1), hallChange1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(hallPin2), hallChange2, CHANGE);
}

void setup() 
{
  Serial.begin(115200);
  hallEffectSetup();
  pingTimer = millis(); // Start now.
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
  
  if (millis() >= pingTimer) {   // pingSpeed milliseconds since last ping, do another ping.
    pingTimer += pingSpeed;      // Set the next ping time.
    // Select next sonar
    sonar = ++sonar % sonarCount;
    currentSonar = &sonarArray[sonar];

    currentSonar->ping_timer(echoCheck); // Send out the ping, calls "echoCheck" function every 24uS where you can check the ping status.
    //Serial.print(sonar);
  }
}

void rotationStatus()
{
  Serial.print(" ");
  Serial.print(forward);
  Serial.print(" ");
  Serial.print(tCount);
  Serial.print(" ");
  Serial.print(lCount);
  Serial.print(" ");
  Serial.print(hCount);
}

int hStatus1=0;
int hStatus2=0;

// ISR 
void hallChange1() 
{
  hStatus1 = digitalRead(hallPin1);
  if (hStatus1==HIGH) {
    hCount++;
  } else {
    lCount++;
  }
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
  //Serial.print("Ping: ");
  for (int i=0;i < sonarCount; i++) {
    Serial.print(" ");
    Serial.print(sonarResults[i] / US_ROUNDTRIP_CM); // Ping returned, uS result in ping_result, convert to cm with US_ROUNDTRIP_CM.
  }
  Serial.print("cm");
}

// ISR 
void echoCheck() 
{ // Timer2 interrupt calls this function every 24uS where you can check the ping status.
  if (currentSonar->check_timer()) { // This is how you check to see if the ping was received.
    // Here's where you can add code.
    sonarResults[sonar] = currentSonar->ping_result;
    sonarUpdate++;
  }
}


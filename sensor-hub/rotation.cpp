#include <Arduino.h>
#include "rotation.h"

volatile bool hStatusL1=0;
volatile bool hStatusL2=0;
volatile bool hStatusR1=0;
volatile bool hStatusR2=0;

// Input pin numbers
uint8_t hallPinL1=-1;
uint8_t hallPinL2=-1;
uint8_t hallPinR1=-1;
uint8_t hallPinR2=-1;

bool leftSupported=false;
bool rightSupported=false;

bool forwardL;
bool forwardR;

void hallChangeL1();
void hallChangeL2();
void hallChangeR1();
void hallChangeR2();

void (*rotCallback)(uint8_t sensor, bool level, bool direction);

void Rotation(uint8_t left1, uint8_t left2, uint8_t right1, uint8_t right2, void (*rotationReport)(uint8_t sensor, bool level, bool direction ))
{
    hallPinL1 = left1;
    hallPinL2 = left2;
    hallPinR1 = right1;
    hallPinR2 = right2;
    rotCallback = rotationReport;

    if ((hallPinL1 >= 0) && (hallPinL2 >= 0))
    {
        leftSupported = true;
        pinMode(hallPinL1, INPUT_PULLUP);
        pinMode(hallPinL2, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(hallPinL1), hallChangeL1, CHANGE);
        attachInterrupt(digitalPinToInterrupt(hallPinL2), hallChangeL2, CHANGE);
    }

    if ((hallPinR1 >= 0) && (hallPinR2 >= 0))
    {
        rightSupported = true;
        pinMode(hallPinR1, INPUT_PULLUP);
        pinMode(hallPinR2, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(hallPinR1), hallChangeR1, CHANGE);
        attachInterrupt(digitalPinToInterrupt(hallPinR2), hallChangeR2, CHANGE);
    }

    // Create ringbuffer for rotation measurements
}

bool RotationGetDirectionL()
{
    return forwardL;
}

bool RotationGetDirectionR()
{
    return forwardR;
}

// Extract event from buffer
bool getRotEventL(struct rotEvent &event)
{
  
}

bool getRotEventR(struct rotEvent &event)
{
  
}

// Number of events in buffer (left and right)
int rotEventCountL()
{
  
}

int rotEventCountR(){
  
}


// ISR 
void hallChangeL1() 
{
    hStatusL1 = digitalRead(hallPinL1);
    rotCallback(0, hStatusL1, forwardL);
}

// ISR 
void hallChangeL2() 
{
    hStatusL2 = digitalRead(hallPinL2);
    forwardL = hStatusL1 == hStatusL2;
    rotCallback(1, hStatusL2, forwardL);
}

// ISR 
void hallChangeR1() 
{
    hStatusR1 = digitalRead(hallPinR1);
    rotCallback(2, hStatusR1, forwardR);
}

// ISR 
void hallChangeR2() 
{
    hStatusR2 = digitalRead(hallPinR2);
    forwardR = hStatusR1 == hStatusR2;
    rotCallback(3, hStatusR2, forwardR);
}


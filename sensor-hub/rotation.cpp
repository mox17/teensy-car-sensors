#include "rotation.h"
#include <Arduino.h>

int hStatusL1=0;
int hStatusL2=0;
int hStatusR1=0;
int hStatusR2=0;

int hallPinL1=-1;
int hallPinL2=-1;
int hallPinR1=-1;
int hallPinR2=-1;

bool leftSupported=false;
bool rightSupported=false;

bool forwardL;
bool forwardR;

void hallChangeL1();
void hallChangeL2();
void hallChangeR1();
void hallChangeR2();

void (*rotCallback)(int sensor);

void Rotation(int left1, int left2, int right1, int right2, void (*rotationReport)(int sensor))
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
}

bool RotationGetDirectionL()
{
    return forwardL;
}

bool RotationGetDirectionR()
{
    return forwardR;
}

// ISR 
void hallChangeL1() 
{
    hStatusL1 = digitalRead(hallPinL1);
    rotCallback(0);
}

// ISR 
void hallChangeL2() 
{
    hStatusL2 = digitalRead(hallPinL2);
    forwardL = hStatusL1 == hStatusL2;
    rotCallback(1);
}

// ISR 
void hallChangeR1() 
{
    hStatusR1 = digitalRead(hallPinR1);
    rotCallback(2);
}

// ISR 
void hallChangeR2() 
{
    hStatusR2 = digitalRead(hallPinR2);
    forwardR = hStatusR1 == hStatusR2;
    rotCallback(3);
}


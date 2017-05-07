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

volatile bool forwardL;
volatile bool forwardR;

void hallChangeL1();
void hallChangeL2();
void hallChangeR1();
void hallChangeR2();

void Rotation(uint8_t left1, uint8_t left2, uint8_t right1, uint8_t right2)
{
    hallPinL1 = left1;
    hallPinL2 = left2;
    hallPinR1 = right1;
    hallPinR2 = right2;

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

#define BUFSIZE 50
volatile byte headL=0;
volatile byte tailL=0;
volatile byte headR=0;
volatile byte tailR=0;
volatile uint32_t overflow=false;
volatile uint32_t rotCountL=0;
volatile uint32_t rotCountR=0;
// Because of the limited RAM on Teensy LC this is rotEvent data is split to avoid alignment waste.
uint32_t bufWhenL[BUFSIZE];
uint32_t bufCountL[BUFSIZE];
bool     bufDirectionL[BUFSIZE];
uint32_t bufWhenR[BUFSIZE];
uint32_t bufCountR[BUFSIZE];
bool     bufDirectionR[BUFSIZE];

bool rotAddRingBufL(uint32_t time, uint32_t count, bool dir)
{
    uint8_t h;

    h = headL + 1;
    if (h >= BUFSIZE)
    {
        h = 0;
    }
    if (h != tailL) 
    {                // if the buffer isn't full
        bufWhenL[h] = time;
        bufCountL[h] = count;
        bufDirectionL[h] = dir;            // put new data into buffer
        headL = h;
        return true;
    }
    else 
    {
        overflow = true;
        return false;
    }
}

bool rotAddRingBufR(uint32_t time, uint32_t count, bool dir)
{
    uint8_t h;

    h = headR + 1;
    if (h >= BUFSIZE)
    {
        h = 0;
    }
    if (h != tailR) 
    {                // if the buffer isn't full
        bufWhenR[h] = time;
        bufCountR[h] = count;
        bufDirectionR[h] = dir;            // put new data into buffer
        headR = h;
        return true;
    }
    else 
    {
        overflow = true;
        return false;
    }
}

bool rotGetEventL(struct rotEvent &event)
{
    uint8_t h, t;
    if (headL==tailL) 
    {
        return false;
    }

    do {
        h = headL;
        t = tailL;                   // wait for data in buffer
    } while (h == t);
    if (++t >= BUFSIZE)
    {
        t = 0;
    }
    event.when = bufWhenL[t];                // remove 1 sample from buffer
    event.count = bufCountL[t];
    event.direction = bufDirectionL[t];
    tailR = t;
    return true;
}

bool rotGetEventR(struct rotEvent &event)
{
    uint8_t h, t;
    if (headR==tailR) 
    {
        return false;
    }

    do {
        h = headR;
        t = tailR;                   // wait for data in buffer
    } while (h == t);
    if (++t >= BUFSIZE)
    {
        t = 0;
    }
    event.when = bufWhenR[t];                // remove 1 sample from buffer
    event.count = bufCountR[t];
    event.direction = bufDirectionR[t];
    tailR = t;
    return true;
}

uint32_t rotCheckOverflow()
{
    uint32_t ret = overflow;
    overflow = 0;
    return ret;
}

RotCalc::RotCalc(bool side) :
m_wHead(0),
m_wTail(0),
m_newData(false)
{
    m_window = new rotEvent[avgCount];
}

/*
 * Handle ringbuffers from ISRs
 */
bool RotCalc::calculate()
{
    bool data;
    struct rotEvent rec;
    bool newData = false;

    m_latest = millis();
    while ((data = (m_side ? rotGetEventL(rec) :  rotGetEventR(rec))))
    {
        m_wHead++;
        if (m_wHead >= avgCount)
        {
            m_wHead = 0;
        }
        m_window[m_wHead] = rec;
        m_latest = rec.when;
        newData = true;
    }
    m_wTail = m_wHead+1;
    if (m_wTail > avgCount)
    {
        m_wTail = 0;
    }
    m_deltaPulse = m_window[m_wHead].count - m_window[m_wTail].count;
    m_deltaMillis = m_latest - m_window[m_wTail].when;
    m_odometer = m_window[m_wHead].count;
    return newData;
}

float RotCalc::pulsePerSec()
{
    calculate();
    return ((float)m_deltaPulse) / ((float)m_deltaMillis);
}

uint32_t RotCalc::odometer()
{
    calculate();
    return m_odometer;
}

bool RotCalc::direction()
{
    return m_window[m_wHead].direction;
}

bool RotCalc::newData()
{
    bool ret = m_newData;
    m_newData = false;
    return ret;
}

// ISR 
void hallChangeL1() 
{
    hStatusL1 = digitalRead(hallPinL1);
    rotCountL++;
    rotAddRingBufL(millis(), rotCountL, forwardL);
}

// ISR 
void hallChangeL2() 
{
    hStatusL2 = digitalRead(hallPinL2);
    forwardL = hStatusL1 == hStatusL2;
    rotCountL++;
    rotAddRingBufL(millis(), rotCountL, forwardL);
}

// ISR 
void hallChangeR1() 
{
    hStatusR1 = digitalRead(hallPinR1);
    rotCountR++;
    rotAddRingBufR(millis(), rotCountR, forwardR);
}

// ISR 
void hallChangeR2() 
{
    hStatusR2 = digitalRead(hallPinR2);
    forwardR = hStatusR1 == hStatusR2;
    rotCountR++;
    rotAddRingBufR(millis(), rotCountR, forwardR);
}

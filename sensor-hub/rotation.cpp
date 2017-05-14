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
volatile uint32_t bufWhenL[BUFSIZE];
volatile uint32_t bufCountL[BUFSIZE];
volatile bool     bufDirectionL[BUFSIZE];
volatile uint32_t bufWhenR[BUFSIZE];
volatile uint32_t bufCountR[BUFSIZE];
volatile bool     bufDirectionR[BUFSIZE];

/*
 * This function is called from interrupt context
 */
bool rotAddRingBufL(uint32_t time, uint32_t count, bool dir)
{
    uint8_t t;

    t = tailL + 1;
    if (t >= BUFSIZE)
    {
        t = 0;
    }
    if (t != headL) 
    {                // if the buffer isn't full
        tailL = t;
        bufWhenL[t] = time;
        bufCountL[t] = count;
        bufDirectionL[t] = dir;            // put new data into buffer
        return true;
    }
    else 
    {
        overflow = true;
        return false;
    }
}

/*
 * This function is called from interrupt context
 */
bool rotAddRingBufR(uint32_t time, uint32_t count, bool dir)
{
    uint8_t t;

    t = tailR + 1;
    if (t >= BUFSIZE)
    {
        t = 0;
    }
    if (t != headR) 
    {                // if the buffer isn't full
        tailR = t;
        bufWhenR[t] = time;
        bufCountR[t] = count;
        bufDirectionR[t] = dir;            // put new data into buffer
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
    uint8_t h;
    if (headL==tailL) 
    {
        return false;
    }
    h = headL;
    if (++h >= BUFSIZE)
    {
        h = 0;
    }
    event.when = bufWhenL[h];          // remove 1 sample from buffer
    event.count = bufCountL[h];
    event.direction = bufDirectionL[h];
    headL = h;
    return true;
}

bool rotGetEventR(struct rotEvent &event)
{
    uint8_t h;
    if (headR==tailR) 
    {
        return false;
    }
    h = headR;
    if (++h >= BUFSIZE)
    {
        h = 0;
    }
    event.when = bufWhenR[h];         // remove 1 sample from buffer
    event.count = bufCountR[h];
    event.direction = bufDirectionR[h];
    headR = h;
    return true;
}

uint32_t rotCheckOverflow()
{
    uint32_t ret = overflow;
    overflow = 0;
    return ret;
}

RotCalc::RotCalc(RotSide side) :
m_wHead(0),
m_wTail(0),
m_newData(false)
{
    m_window = new rotEvent[avgCount];
}

/*
 * Process data put in ringbuffers by ISRs
 */
bool RotCalc::calculate()
{
    bool data;
    struct rotEvent rec;
    bool newData = false;

    m_latest = millis();
    while ((data = ((m_side == ROT_LEFT) ? rotGetEventL(rec) : rotGetEventR(rec))))
    {
        m_wTail++;
        if (m_wTail >= avgCount)
        {
            m_wTail = 0;
        }
        // If buffer full, discard old entries
        if (m_wTail == m_wHead) 
        {
            m_wHead = m_wHead+1;
            if (m_wHead >= avgCount)
            {
                m_wHead = 0;
            }
        }
        m_window[m_wTail] = rec;
        m_latest = rec.when;
        Serial1.print('c');
        newData = true;
    }
    if (newData)
    {
        Serial1.print('-');
        // ISR buffer is emptied (transferred to m_window)
        m_deltaPulse = m_window[m_wTail].count - m_window[m_wHead].count;
        m_deltaMillis = m_latest - m_window[m_wTail].when;
        m_odometer = m_window[m_wTail].count;
    }
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

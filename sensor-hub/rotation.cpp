#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "rotation.h"

// Input pin numbers
uint8_t hallPinL1=-1;
uint8_t hallPinL2=-1;
uint8_t hallPinR1=-1;
uint8_t hallPinR2=-1;

// Are there input pins given for both sides.
bool leftSupported=false;
bool rightSupported=false;

volatile RotDir forwardL;
volatile RotDir forwardR;

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
volatile byte headL=0;  // Circular ISR buffer indices LEFT
volatile byte tailL=0;
volatile byte headR=0;  // Circular ISR buffer indices RIGHT
volatile byte tailR=0;
volatile uint32_t overflow=false;
volatile uint32_t rotCountL=0;
volatile uint32_t rotCountR=0;
// Because of the limited RAM on Teensy LC this rotation data is split to avoid alignment waste.
volatile uint32_t bufWhenL[BUFSIZE];
volatile uint32_t bufCountL[BUFSIZE];
volatile RotDir   bufDirectionL[BUFSIZE];
volatile uint32_t bufWhenR[BUFSIZE];
volatile uint32_t bufCountR[BUFSIZE];
volatile RotDir   bufDirectionR[BUFSIZE];

volatile byte rStateL=0;
volatile byte rStateR=0;

/*
 * This function is called from interrupt context
 */
bool rotAddRingBufL(uint32_t time, uint32_t count, RotDir dir)
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
        Serial1.print('!');
        overflow = true;
        return false;
    }
}

/*
 * This function is called from interrupt context
 */
bool rotAddRingBufR(uint32_t time, uint32_t count, RotDir dir)
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
m_wHead(avgCount-1),
m_wTail(0),
m_wCount(0),
m_newData(false),
m_deltaPulse(0),
m_deltaMillis(0),
m_odometer(0)
{
    m_window = new rotEvent[avgCount];
}

/*
 * Process data put in ringbuffers by ISRs.
 * Keep up to avgCount-1 samples available.
 * Clear history when direction changes.
  */
bool RotCalc::calculate()
{
    bool dataAvailable;
    struct rotEvent rec;
    bool newData = false;
    byte newest;
    byte oldest;

    m_latest = millis();
    while ((dataAvailable = ((m_side == ROT_LEFT) ? rotGetEventL(rec) : rotGetEventR(rec))))
    {
        if (m_direction != rec.direction)
        {
            // When direction changes, speed passes through zero. Therefore clear averaging buffer.
            m_wTail = 0;
            m_wHead = avgCount-1;
            m_wCount = 0;
            Serial.print("\n----");
            Serial.print(m_direction);
            Serial.print(" - ");
            Serial.println(rec.direction);
        }
        m_direction = rec.direction;
        m_window[m_wTail] = rec;
        newest = m_wTail;
        m_wTail = (m_wTail+1) % avgCount;
        m_wCount++;
        // If buffer full, discard old entries
        if (m_wCount >= (avgCount-1))
        {
            //Serial1.print('H');
            m_wHead = (m_wHead+1) % avgCount;
            m_wCount--;
        }
        oldest = (newest-m_wCount+1) % avgCount;
        m_newData = true;
        newData = true;
    }
    if (newData)
    {
        // ISR buffer is emptied (transferred to m_window). Calculate speed.
        if (m_wCount == 1)
        {
            m_deltaPulse  = 0;
            m_deltaMillis = m_latest - m_window[newest].when;
        }
        else
        {
            m_deltaPulse  = m_window[newest].count - m_window[oldest].count;
            m_deltaMillis = m_latest - m_window[oldest].when; // Time window is from oldest sample to now
        }
        m_odometer    = m_window[newest].count;
        /*
        Serial.print("["); 
        Serial.print(m_wCount); 
        Serial.print("|"); 
        Serial.print(newest); 
        Serial.print("|"); 
        Serial.print(oldest); 
        Serial.print("|"); 
        Serial.print(m_wTail); 
        Serial.print("|"); 
        Serial.print(m_wHead); 
        Serial.print("|"); 
        Serial.print(m_deltaPulse); 
        Serial.print("|"); 
        Serial.print(m_deltaMillis); 
        Serial.print("]");
        */
    }
    return newData;
}

float RotCalc::pulsePerSec()
{
    calculate();
    if (m_deltaPulse && m_deltaMillis)
    {
        //Serial.print(m_deltaPulse); Serial.print(" "); Serial.println(m_deltaMillis);
        return (1000.0 * (float)m_deltaPulse / (float)m_deltaMillis);
    }
    else
    {
        return 0.0;
    }
}

uint32_t RotCalc::odometer()
{
    calculate();
    return m_odometer;
}

RotDir RotCalc::direction()
{
    return m_direction;
}

bool RotCalc::newData()
{
    bool ret = m_newData;
    m_newData = false;
    return ret;
}

/*
 * Calculate rotation direction based on sensor change.
 */
inline RotDir calcDirection(byte oldVal, byte newVal)
{
    // This table expresses the direction from two subsequent sensor readings. 1st sensor weight 1, 2nd sensor with weight 2
    const static RotDir states[4][4] = 
        {/*0*/{ROT_DIR_NONE,     ROT_DIR_FORWARD,  ROT_DIR_BACKWARD, ROT_DIR_ERROR},
         /*1*/{ROT_DIR_BACKWARD, ROT_DIR_NONE,     ROT_DIR_ERROR,    ROT_DIR_FORWARD},
         /*2*/{ROT_DIR_FORWARD,  ROT_DIR_ERROR,    ROT_DIR_NONE,     ROT_DIR_BACKWARD},
         /*3*/{ROT_DIR_ERROR,    ROT_DIR_BACKWARD, ROT_DIR_FORWARD,  ROT_DIR_NONE}};
    return states[oldVal][newVal];
}

// ISR 
void hallChangeL1() 
{
    byte old = rStateL;
    bool pinStatus;

    pinStatus = digitalRead(hallPinL1);
    rotCountL++;
    rStateL = pinStatus ? (rStateL | 0x1) : (rStateL & 0x2);
    //Serial1.print(char(48+rStateL));
    forwardL = calcDirection(old, rStateL);
    rotAddRingBufL(millis(), rotCountL, forwardL);
}

// ISR 
void hallChangeL2() 
{
    byte old = rStateL;
    bool pinStatus;

    pinStatus = digitalRead(hallPinL2);
    rStateL = pinStatus ? (rStateL | 0x2) : (rStateL & 0x1);
    //Serial1.print(char(48+rStateL));
    forwardL = calcDirection(old, rStateL);
    rotCountL++;
    rotAddRingBufL(millis(), rotCountL, forwardL);
}

// ISR 
void hallChangeR1() 
{
    byte old = rStateR;
    bool pinStatus;

    pinStatus = digitalRead(hallPinR1);
    rStateR = pinStatus ? (rStateR | 0x1) : (rStateR & 0x2);
    forwardR = calcDirection(old, rStateR);
    rotCountR++;
    rotAddRingBufR(millis(), rotCountR, forwardR);
}

// ISR 
void hallChangeR2() 
{
    byte old = rStateR;
    bool pinStatus;

    pinStatus = digitalRead(hallPinR2);
    rStateR = pinStatus ? (rStateR | 0x2) : (rStateR & 0x1);
    forwardR = calcDirection(old, rStateR);
    rotCountR++;
    rotAddRingBufR(millis(), rotCountR, forwardR);
}


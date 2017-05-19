#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "rotation.h"

const size_t BUFSIZE = 32;

/**
 * @brief Calculate rotation direction based on sensor change.
 */
inline rotDirection calcDirection(byte oldVal, byte newVal)
{
    // This table expresses the rotation direction from two subsequent sensor readings. 
    // Sensor redings combined into a two-bit number. 1st sensor weight 1, 2nd sensor with weight 2
    const static rotDirection states[4][4] = 
               /*0*/             /*1*/             /*2*/             /*3*/
        {/*0*/{ROT_DIR_NONE,     ROT_DIR_FORWARD,  ROT_DIR_BACKWARD, ROT_DIR_ERROR},
         /*1*/{ROT_DIR_BACKWARD, ROT_DIR_NONE,     ROT_DIR_ERROR,    ROT_DIR_FORWARD},
         /*2*/{ROT_DIR_FORWARD,  ROT_DIR_ERROR,    ROT_DIR_NONE,     ROT_DIR_BACKWARD},
         /*3*/{ROT_DIR_ERROR,    ROT_DIR_BACKWARD, ROT_DIR_FORWARD,  ROT_DIR_NONE}};
    return states[oldVal][newVal];
}

class CircularBuffer
{
public:
    CircularBuffer() :
        m_head(0),
        m_tail(0),
        m_overflowCount(0)
    {
    }

    void resetBuffer()
    {
        noInterrupts();
        m_head = 0;
        m_tail = 0;
        m_overflowCount = 0;
        interrupts();
    }

    bool push(uint32_t time, uint32_t count, rotDirection dir)
    {
        uint8_t t;

        t = m_tail;
        if (++t >= BUFSIZE)
        {
            t = 0;
        }
        if (t != m_head) 
        {
            m_tail = t;
            m_buffer[t].when = time;
            m_buffer[t].count = count;
            m_buffer[t].direction = dir;
            return true;
        }
        else 
        {
            m_overflowCount++;
            return false;
        }
    }

    bool pop(struct rotEvent &event)
    {
        uint8_t h;
        if (m_head == m_tail) 
        {
            return false;
        }
        h = m_head;
        if (++h >= BUFSIZE)
        {
            h = 0;
        }
        event = m_buffer[h];          // remove 1 sample from buffer
        m_head = h;
        return true;
    }

    uint32_t getOverflow()
    {
        uint32_t c = m_overflowCount;
        m_overflowCount = 0;
        return c;
    }

private:
    volatile byte m_head;
    volatile byte m_tail;
    volatile uint32_t m_overflowCount;
    rotEvent m_buffer[BUFSIZE];
};

class WheelSensor
{
public:
    /**
     * @brief Wheel sensor interrupt handling
     *
     * Since this code uses interrupts, 
     * the object pointer must be available as a global variable.
     *
     * Each WheelSensor instance will need two ISR handlers written
     * like this:
     *
     * void isrOne()
     * {
     *     global->isrHelper(ROT_SENSOR_ONE);
     * }
     *
     * void isrTwo()
     * {
     *     global->isrHelper(ROT_SENSOR_TWO);
     * }
     *
     */
    WheelSensor() :
        m_pinsState(0),
        m_rotCount(0),
        m_direction(ROT_DIR_NONE)
    {
    }

    /**
     * @brief Set up HW is separate method. 
     * 
     * This allows static allocation of WheelSensor object, while still
     * controlling when interrupts starts flowing in.
     */
    void activate(int pin1, int pin2, void (*isr1)(void), void (*isr2)(void))
    {
        m_inputPins[ROT_SENSOR_ONE] = pin1;
        m_inputPins[ROT_SENSOR_TWO] = pin2;
        pinMode(pin1, INPUT_PULLUP);
        pinMode(pin2, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(pin1), isr1, CHANGE);
        attachInterrupt(digitalPinToInterrupt(pin2), isr2, CHANGE);
    }
    /**
     * @brief This function is called from ISR handler. 
     */
    void isrHelper(sensorNo sensor)
    {
        byte old = m_pinsState;
        const static byte orMask[2]  = {0x01, 0x02};
        const static byte andMask[2] = {0x02, 0x01};

        m_rotCount++;
        m_pinsState = digitalRead(m_inputPins[sensor]) ? (m_pinsState | orMask[sensor]) : 
                                                         (m_pinsState & andMask[sensor]);
        m_direction = calcDirection(old, m_pinsState);
        m_buf.push(millis(), m_rotCount, m_direction);
    }

    bool getEvent(rotEvent &event)
    {
        return m_buf.pop(event);
    }

    void resetCounter()
    {
        m_rotCount = 0;
        m_buf.resetBuffer();
    }

private:
    byte m_inputPins[2];
    byte m_pinsState;
    volatile uint32_t m_rotCount;
    volatile rotDirection m_direction;
    CircularBuffer m_buf;
};

WheelSensor leftWheel;
WheelSensor rightWheel;

/*
 * @brief Rotation calculation class
 *
 * This class encapsulated the collection of data from interrupt driven buffers 
 * and calculation (averaging) of speed over a number of samples.
 */
RotCalc::RotCalc(rotSide side) :
    m_wHead(avgCount-1),
    m_wTail(0),
    m_wCount(0),
    m_newData(false),
    m_deltaPulse(0),
    m_deltaMillis(0),
    m_odometer(0)
{
}

/**
 * @brief Process data put in ringbuffers by ISRs.
 *
 * Keep up to avgCount-1 samples available.
 * Clear history when direction changes.
 */
bool RotCalc::handleBuffer()
{
    bool dataAvailable;
    struct rotEvent rec;
    bool newData = false;
    byte newest;
    byte oldest;

    m_latest = millis();
    while ((dataAvailable = ((m_side == ROT_LEFT) ? leftWheel.getEvent(rec) : rightWheel.getEvent(rec))))
    {
        if (m_direction != rec.direction)
        {
            // When direction changes, speed passes through zero. Therefore clear averaging buffer.
            m_wTail = 0;
            m_wHead = avgCount-1;
            m_wCount = 0;
            m_odoDirChg = m_odometer;
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

uint16_t RotCalc::pulsePerSec()
{
    if (m_deltaPulse && m_deltaMillis)
    {
        //Serial.print(m_deltaPulse); Serial.print(" "); Serial.println(m_deltaMillis);
        return (1000 * m_deltaPulse / m_deltaMillis);
    }
    else
    {
        return 0;
    }
}

uint32_t RotCalc::odometer()
{
    handleBuffer();
    return m_odometer;
}

rotDirection RotCalc::direction()
{
    return m_direction;
}

bool RotCalc::newData()
{
    bool ret = m_newData;
    m_newData = false;
    return ret;
}

/**
 * @brief Get rotation record.
 */
void RotCalc::rotGetRec(rot_one &rec)
{
    rec.speed = pulsePerSec();
    rec.direction = m_direction;
    rec.reserved = 0;
    rec.when = m_latest;
    rec.dist = m_odoDirChg;
    rec.dist_abs = m_odometer;
}

void RotCalc::resetOdometer()
{
    m_odoDirChg = 0;
    m_odometer = 0;
    leftWheel.resetCounter();
    rightWheel.resetCounter();
}

void isrLeftOne()
{
    leftWheel.isrHelper(ROT_SENSOR_ONE);
}

void isrLeftTwo()
{
    leftWheel.isrHelper(ROT_SENSOR_TWO);
}

void isrRightOne()
{
    rightWheel.isrHelper(ROT_SENSOR_ONE);
}

void isrRightTwo()
{
    rightWheel.isrHelper(ROT_SENSOR_TWO);
}

/**
 * @brief Set up main rotation sensor objects.
 */
void Rotation(uint8_t left1, uint8_t left2, uint8_t right1, uint8_t right2)
{
    if ((left1 >= 0) && (left2 >= 0))
    {
        leftWheel.activate(left1, left2, &isrLeftOne, &isrLeftTwo);
    }

    if ((right1 >= 0) && (right2 >= 0))
    {
        rightWheel.activate(right1, right2, &isrRightOne, &isrRightTwo);
    }
}

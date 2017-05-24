#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "rotation.h"
#include "counters.h"

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
        ring_head(0),
        ring_tail(0),
        m_overflowCount(0)
    {
    }

    void resetBuffer()
    {
        noInterrupts();
        ring_head = 0;
        ring_tail = 0;
        m_overflowCount = 0;
        interrupts();
    }

    bool push(uint32_t time, uint32_t count, rotDirection dir)
    {
        uint8_t next_head = (ring_head + 1) % BUFSIZE;
        if (next_head != ring_tail)
        {
            /* there is room */
            m_buffer[ring_head].when = time;
            m_buffer[ring_head].count = count;
            m_buffer[ring_head].direction = dir;
            ring_head = next_head;
            return true;
        } else {
            /* no room left in the buffer */
            m_overflowCount++;
            cnt.inc(rotBufFull);
            return false;
        }
    }

    bool pop(struct rotEvent &event)
    {
        if (ring_head != ring_tail)
        {
            event = m_buffer[ring_tail];
            ring_tail = (ring_tail + 1) % BUFSIZE;
            return true;
        } else {
            return false;
        }
    }

    uint32_t getOverflow()
    {
        uint32_t c = m_overflowCount;
        m_overflowCount = 0;
        return c;
    }

private:
    volatile byte ring_head;
    volatile byte ring_tail;
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
        const static byte bitMask[2] = {0x01, 0x02};

        m_rotCount++;
        byte level = digitalRead(m_inputPins[sensor]);
        m_pinsState = level ? (m_pinsState | bitMask[sensor]) : (m_pinsState & ~bitMask[sensor]);
        rotDirection d = calcDirection(old, m_pinsState);
        if (d == ROT_DIR_FORWARD || d == ROT_DIR_BACKWARD)
        {
            // Ignore errors for overall direction
            m_direction = d;
        }
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
    m_side = side;
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
    while ((dataAvailable = ((m_side == ROT_LEFT) ? leftWheel.getEvent(rec) :
                                                    rightWheel.getEvent(rec))))
    {
        if ((rec.direction == ROT_DIR_FORWARD ||
             rec.direction == ROT_DIR_BACKWARD) &&
            (m_direction != rec.direction))
        {
            // When direction changes, speed passes through zero. Therefore clear averaging buffer.
            m_wTail = 0;
            m_wHead = avgCount-1;
            m_wCount = 0;
            m_odoDirChg = m_odometer;
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
    }
    return newData;
}

uint16_t RotCalc::pulsePerSec()
{
    if (m_deltaPulse && m_deltaMillis)
    {
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
    switch (m_side)
    {
        case ROT_LEFT:
            leftWheel.resetCounter();
            break;
        case ROT_RIGHT:
            rightWheel.resetCounter();
            break;
    }
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

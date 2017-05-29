#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "rotation.h"
#include "counters.h"

void isrLeftOne();
void isrLeftTwo();
void isrRightOne();
void isrRightTwo();

/**
 * @brief Calculate rotation direction based on sensor change.
 * @param oldVal previous value of sensor1+2*sensor2
 * @param newVal Updated value of sensor1+2*sensor2
 * @return direction of rotation
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


CircularBuffer::CircularBuffer() :
    m_head(0),
    m_count(0)
{
}

/**
 * @brief Reset the circular buffer
 */
void CircularBuffer::resetBuffer()
{
    noInterrupts();
    m_head = 0;
    m_count = 0;
    interrupts();
}

/**
 * @brief Add data to buffer
 * @param time The time in milliseconds
 * @param count The total number of pulses so far
 * @param dir The direction of rotation
 */
void CircularBuffer::add(uint32_t time, uint32_t count, rotDirection dir)
{
    uint8_t next_head = (m_head + 1) % BUFSIZE;
    m_buffer[m_head].when = time;
    m_buffer[m_head].count = count;
    m_buffer[m_head].direction = dir;
    m_head = next_head;
    if (m_count < BUFSIZE)
    {
        m_count++;
    }
}

void CircularBuffer::dupLatest(uint32_t time)
{
    if (m_count)
    {
        noInterrupts();
        int idx = m_head - 1;
        if (idx < 0)
        {
            idx += BUFSIZE;
        }
        uint32_t count =  m_buffer[idx].count;
        rotDirection dir = m_buffer[idx].direction;
        add(time, count, dir);
        interrupts();
    }
}

/**
 * @brief Get old record
 * @param nTh of data (0==newest)
 */
uint32_t CircularBuffer::getData(int32_t nThOlder, rotEvent &event)
{
    uint32_t ret=0;
    int idx;
    noInterrupts();
    if (m_count)
    {
        ret = min((unsigned)(m_count-1), ((unsigned)nThOlder)); // There might not be requested amount of data
        idx = m_head-ret-1;
        if (idx<0)
        {
            idx += BUFSIZE;
        }
        event = m_buffer[idx];
    } else {
        // TODO handle if no data
        event.when = 0;
        event.count = 0;
        event.direction = ROT_DIR_NONE;
    }
    interrupts();
    return ret;
}

unsigned CircularBuffer::count()
{
return m_count;
}

WheelSensor::WheelSensor(rotSide side) :
    m_pinsState(0),
    m_rotCount(0),
    m_odoDirChg(0),
    m_direction(ROT_DIR_NONE)
{
    m_side = side;
}

/**
 * @brief Set up HW is separate method.
 *
 * This allows static allocation of WheelSensor object, while still
 * controlling when interrupts starts flowing in.
 * @param pin1 1st sensor pin
 * @param pin2 2nd sensor pin
 * @param isr1 1st ISR routine
 * @param isr2 2nd ISR routine
 */
void WheelSensor::activate(int pin1, int pin2,
                           void (*isr1)(void), void (*isr2)(void))
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
 * @param sensor The sensor id (ROT_SENSOR_ONE or ROT_SENSOR_TWO)
 */
void WheelSensor::isrHelper(sensorNo sensor)
{
    byte old = m_pinsState;
    const static byte bitMask[2] = {0x01, 0x02};
    uint32_t now = millis();

    m_rotCount++;
    byte level = digitalRead(m_inputPins[sensor]);
    m_pinsState = level ? (m_pinsState | bitMask[sensor]) : (m_pinsState & ~bitMask[sensor]);
    rotDirection d = calcDirection(old, m_pinsState);
    // Ignore errors for overall direction
    if (d == ROT_DIR_FORWARD || d == ROT_DIR_BACKWARD)
    {
        // No speed averaging when direction changes - clear buffer
        if (m_direction != d)
        {
            m_buf.resetBuffer();
            m_odoDirChg = m_rotCount; // remember direction change odometer
            m_direction = d;
        }
    }
    m_buf.add(now, m_rotCount, m_direction);
    m_owner->reportEvent(m_side);
}

void WheelSensor::reset()
{
    m_rotCount = 0;
    m_odoDirChg = 0;
    m_direction = ROT_DIR_NONE;
    m_buf.resetBuffer();
}

void WheelSensor::calculate(rot_one &rec)
{
    rotEvent latestRec, oldRec;
    unsigned count = m_buf.count();
    if (count == 0)
    {
        rec.speed = 0;
    }
    else if (count == 1)
    {
        m_buf.getData(0, latestRec);
        rec.when = latestRec.when;
        rec.speed = 0;
    } else {
        m_buf.getData(0, latestRec);
        m_buf.getData(WHEEL_SPEED_SAMPLES, oldRec);
        rec.when = latestRec.when;
        int32_t deltaPulse = latestRec.count-oldRec.count;
        int32_t deltaMillis = latestRec.when-oldRec.when;
        rec.speed = (deltaPulse && deltaMillis) ?
                    (1000 * deltaPulse / deltaMillis) : 0;
    }
    rec.direction = m_direction;
    rec.dist = m_odoDirChg;
    rec.dist_abs = m_rotCount;
}

void WheelSensor::setOwner(Axle* obj)
{
    m_owner = obj;
}

void WheelSensor::timeout()
{
    m_buf.dupLatest(millis());
}

// Global variable to help ISR routines
WheelSensor *leftWheel;  //!< Left wheel sensor object
WheelSensor *rightWheel; //!< Right wheel sensor object

Axle::Axle(void (*report)(rot_one left, rot_one right)) :
    m_leftWheel(ROT_LEFT),
    m_rightWheel(ROT_RIGHT),
    m_lastReport(0)
{
    m_report = report;  // Reporting function to main program
    leftWheel = &m_leftWheel;
    rightWheel = &m_rightWheel;
    m_leftWheel.setOwner(this);
    m_rightWheel.setOwner(this);
}

/**
 * @brief Reset the odometer for this side
 */
void Axle::resetOdometer()
{
    m_leftWheel.reset();
    m_rightWheel.reset();
}

/**
 * @brief Set up main rotation sensor objects.
 * @param left1 1st sensor left
 * @param left2 2nd sensor left
 * @param right1 1st sensor right
 * @param right2 2nd sensor right
 */
void Axle::config(uint8_t left1, uint8_t left2, uint8_t right1, uint8_t right2)
{
    if ((left1 >= 0) && (left2 >= 0))
    {
        m_leftWheel.activate(left1, left2, &isrLeftOne, &isrLeftTwo);
    }

    if ((right1 >= 0) && (right2 >= 0))
    {
        m_rightWheel.activate(right1, right2, &isrRightOne, &isrRightTwo);
    }
}

/**
 * @brief Called from interrupt context
 */
void Axle::reportEvent(rotSide side)
{
    uint32_t now = millis();

    if ((now-m_lastReport) >= WHEEL_MIN_INTERVAL_MS)
    {
        m_leftWheel.calculate(m_left);
        m_rightWheel.calculate(m_right);
        m_lastReport = now;
        m_report(m_left, m_right);
    } else {
        cnt.inc(rotUpdSkip);
    }
}

rotDirection Axle::direction()
{
    rotDirection l = m_leftWheel.direction();
    rotDirection r = m_rightWheel.direction();

    if (l==r)
    {
        return r;
    } else {
        return ROT_DIR_NONE;
    }
}

uint32_t Axle::odometer()
{
    return (m_leftWheel.odometer()+m_rightWheel.odometer())/2;
}

uint32_t Axle::pulsePerSec()
{
    return (m_left.speed+m_right.speed)/2;
}

void Axle::timeout()
{
    uint32_t now = millis();

    if ((now-m_lastReport) >= WHEEL_MAX_INTERVAL_MS)
    {
        m_leftWheel.timeout();
        m_rightWheel.timeout();
        m_leftWheel.calculate(m_left);
        m_rightWheel.calculate(m_right);
        m_lastReport = now;
        m_report(m_left, m_right);
    }
}

//!< @brief ISR
void isrLeftOne()
{
    leftWheel->isrHelper(ROT_SENSOR_ONE);
}

//!< @brief ISR
void isrLeftTwo()
{
    leftWheel->isrHelper(ROT_SENSOR_TWO);
}

//!< @brief ISR
void isrRightOne()
{
    rightWheel->isrHelper(ROT_SENSOR_ONE);
}

//!< @brief ISR
void isrRightTwo()
{
    rightWheel->isrHelper(ROT_SENSOR_TWO);
}

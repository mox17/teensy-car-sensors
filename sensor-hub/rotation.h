/**
 *  @brief Handling of rotation sensors.
 *  Two sensors per wheel.
 */
#ifndef ROTATION_H
#define ROTATION_H
#include "common.h"

#define WHEEL_MIN_INTERVAL_MS 5
#define WHEEL_MAX_INTERVAL_MS 200
#define WHEEL_SPEED_SAMPLES 10 // Averaging window for speed

const unsigned BUFSIZE = 32; //!< Buffer size for raw interrupts

//!< @brief Each wheel has two hall-effect sensors
enum sensorNo {
    ROT_SENSOR_ONE,
    ROT_SENSOR_TWO
};

//!< @brief Single wheel event structure
struct rotEvent
{
    uint32_t when;  //!< milliseconds
    uint32_t count; //!< Accumulated count
    rotDirection direction;
};

class Axle;

/**
 * @class Circular buffer used to collect rotation events.
 * Each wheel generates 20 interrupts per revolution.
 */
class CircularBuffer
{
public:
    CircularBuffer();
    void resetBuffer();
    void add(uint32_t time, uint32_t count, rotDirection dir);
    uint32_t getData(int32_t nThOlder, rotEvent &event);
    unsigned count();
    void dupLatest(uint32_t time);

private:
    volatile int32_t m_head; //!< newest item index
    volatile uint32_t m_count;
    rotEvent m_buffer[BUFSIZE]; //!< The actual buffer
};

/**
 * @class Wheel sensor interrupt handling
 *
 * Since this code uses interrupts,
 * the object pointer must be available as a global variable.
 */
class WheelSensor
{
public:
    WheelSensor(rotSide side);
    void setOwner(Axle* owner);
    void isrHelper(sensorNo sensor);
    void calculate(rot_one &rec);
    void activate(int pin1, int pin2,
                  void (*isr1)(void), void (*isr2)(void));
    void reset();
    rotDirection direction();
    uint32_t odometer();
    void timeout();

private:
    byte m_inputPins[2]; //!< Input pins for this side
    volatile byte m_pinsState;    //!< Combined state (to calculate direction)
    volatile uint32_t m_rotCount; //!< Total number of interrupts
    volatile uint32_t m_odoDirChg;  //!< odometer at direction change
    volatile rotDirection m_direction; //<! Which way are we rolling
    CircularBuffer m_buf;  //!< History buffer for events
    Axle* m_owner;
    rotSide m_side;

};

class Axle
{
public:
    Axle(void (*report)(rot_one left, rot_one right));
    void config(uint8_t left1, uint8_t left2, uint8_t right1, uint8_t right2);
    void timeout(); //!< called to ensure reporting when wheels are not moving
    void resetOdometer();
    void reportEvent(rotSide side);
    rotDirection direction();
    uint32_t odometer();
    uint32_t pulsePerSec();

private:
    void (*m_report)(rot_one left, rot_one right);
    WheelSensor m_leftWheel;
    WheelSensor m_rightWheel;
    uint32_t m_lastReport;
    rot_one m_left;
    rot_one m_right;
};

#endif // ROTATION_H

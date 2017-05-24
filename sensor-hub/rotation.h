/*
 *  Handling of rotation sensors.
 *  Two sensors per wheel.
 *
 */
#pragma once
#include "common.h"

//!< @brief Each wheel has two hall-effect sensors
enum sensorNo {
    ROT_SENSOR_ONE,
    ROT_SENSOR_TWO
};

void Rotation(uint8_t left1,  uint8_t left2,
                uint8_t right1, uint8_t right2);

/**
 * @brief Single wheel event structure
 */
struct rotEvent
{
    uint32_t when;  // milliseconds
    uint32_t count; // Accumulated count
    rotDirection direction;
};

class RotCalc
{
public:
    const static unsigned avgCount = 20; // 5 magnets and 2 sensors give 20 pulses per wheel revolution, even out alignment differences
    RotCalc(rotSide side);
    bool pulse(uint32_t time, bool direction);
    bool handleBuffer();
    uint16_t pulsePerSec();
    uint32_t odometer();
    rotDirection direction();
    bool newData();
    void rotGetRec(rot_one &rec);
    void resetOdometer();

private:
    rotEvent m_window[avgCount];    //!< Buffer for averaging samples
    byte m_wHead;                   //!< Oldest entry index
    byte m_wTail;                   //!< Newest entry index
    byte m_wCount;                  //!< Nbr of entries
    rotSide m_side;                 //!< Which side (left,right)
    bool m_newData;                 //!< New data sine last time?
    rotDirection m_direction;       //!< Rotation
    uint32_t m_latest;              //!< Timestamp
    uint32_t m_deltaPulse;          //!< Number of pulses in buffer
    uint32_t m_deltaMillis;         //!< Timespan of buffer
    uint32_t m_odometer;            //!< Distance
    uint32_t m_odoDirChg;           //!< odometer at direction change
};

/* 
 *  Handling of rotation sensors. 
 *  Two sensors per wheel.
 *  
 */
#pragma once
#include "common.h"

// Each wheel has two hall-effect sensors
enum sensorNo {
    ROT_SENSOR_ONE,
    ROT_SENSOR_TWO
};

void Rotation(uint8_t left1,  uint8_t left2, 
	            uint8_t right1, uint8_t right2);

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
	bool calculate();
	uint16_t pulsePerSec();
	uint32_t odometer();
	rotDirection direction();
	bool newData();
    void rotGetRec(rot_one &rec);
    void resetOdometer();

private:
    rotEvent m_window[avgCount]; // Buffer for averaging samples
    byte m_wHead;
    byte m_wTail;
    byte m_wCount;
    rotSide m_side;
    bool m_newData;
    rotDirection m_direction;
    uint32_t m_latest;
    uint32_t m_deltaPulse;
    uint32_t m_deltaMillis;
    uint32_t m_odometer;
    uint32_t m_odoDirChg; // odometer at direction change
};

/* 
 *  Handling of rotation sensors. 
 *  Two sensors per wheel.
 *  
 */

enum RotSide {
  ROT_LEFT,
  ROT_RIGHT
};

enum RotDir {
    ROT_DIR_NONE,
    ROT_DIR_ERROR,
    ROT_DIR_FORWARD,
    ROT_DIR_BACKWARD, 
};

void Rotation(uint8_t left1,  uint8_t left2, 
	            uint8_t right1, uint8_t right2);

struct rotEvent 
{
    uint32_t when;  // milliseconds
    uint32_t count; // Accumulated count
    RotDir direction;
};

// Extract event from buffer
bool rotGetEventL(struct rotEvent &event);
bool rotGetEventR(struct rotEvent &event);

uint32_t rotCheckOverflow();

class RotCalc
{
public:
	const unsigned avgCount = 20; // 5 magnets and 2 sensors give 20 pulses per wheel revolution, even out alignment differences
	RotCalc(RotSide side);
	bool pulse(uint32_t time, bool direction);
	bool calculate();
	float pulsePerSec();
	uint32_t odometer();
	RotDir direction();
	bool newData();

private:
    uint32_t avgSiz;
    rotEvent* m_window;
    byte m_wHead;
    byte m_wTail;
    byte m_wCount;
    RotSide m_side;
    bool m_newData;
    RotDir m_direction;
    uint32_t m_latest;
    uint32_t m_deltaPulse;
    uint32_t m_deltaMillis;
    uint32_t m_odometer;

};

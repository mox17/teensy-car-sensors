/* 
 *  Wrapping of rotation sensors. 
 *  Two sensors per wheel.
 *  
 */

void Rotation(uint8_t left1, uint8_t left2, 
	          uint8_t right1, uint8_t right2);

struct rotEvent 
{
    uint32_t when;
    uint32_t count;
    bool direction;
};

// Extract event from buffer
bool rotGetEventL(struct rotEvent &event);
bool rotGetEventL(struct rotEvent &event);

uint32_t rotCheckOverflow();

class RotCalc
{
public:
	const unsigned avgCount = 5; // 5 magnets per wheel, even out alignment differences
	RotCalc(bool side);
	bool pulse(uint32_t time, bool direction);
	bool calculate();
	float pulsePerSec();
	uint32_t odometer();
	bool direction();
	bool newData();

private:
    uint32_t avgSiz;
    rotEvent* m_window;
    byte m_wHead;
    byte m_wTail;
    bool m_side;
    bool m_newData;
    uint32_t m_latest;
    uint32_t m_deltaPulse;
    uint32_t m_deltaMillis;
    uint32_t m_odometer;

};
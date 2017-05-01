/* 
 *  Wrapping of rotation sensors. 
 *  Two sensors per wheel.
 *  
 */

void Rotation(uint8_t left1, uint8_t left2, uint8_t right1, uint8_t right2, void (*rotationReport)(uint8_t sensor, bool level, bool direction ));
bool RotationGetDirectionL(); // true for "forward", false for "backward"
bool RotationGetDirectionR(); // true for "forward", false for "backward"
int  RotationGetInstantSpeed();

struct rotEvent 
{
    uint32_t intervalmsec;  // Time from HIGH to HIGH transitions
    uint32_t when;
};

// Extract event from buffer
bool getRotEventL(struct rotEvent &event);
bool getRotEventR(struct rotEvent &event);

// Number of events in buffer (left and right)
int rotEventCountL();
int rotEventCountR();


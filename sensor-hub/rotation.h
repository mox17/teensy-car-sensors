/* 
 *  Wrapping of rotation sensors. 
 *  Two sensors per wheel.
 *  
 */

void Rotation(int left1, int left2, int right1, int right2, void (*rotationReport)(int sensor));
bool RotationGetDirectionL(); // true for "forward", false for "backward"
bool RotationGetDirectionR(); // true for "forward", false for "backward"
int  RotationGetInstantSpeed();


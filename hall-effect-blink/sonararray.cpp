/* 
 *  This class encapsulates the use of a HC-SR04 style sonar
 *  
 */

#include <NewPing.h>
#include "sonararray.h"

NewPing* SonarArray::m_currentSensor;

SonarArray::SonarArray(int noOfPins, const int* pins, int maxDistance)
{
    m_maxDistance = maxDistance;
    m_noOfPins = min(noOfPins, MAX_PINS);
    m_seqLen = m_noOfPins;
    for (int i=0; i < noOfPins; i++) {
        m_pins[i] = pins[i];
        m_sensorArray[i] = new NewPing(m_pins[i], m_pins[i], maxDistance);
        m_sequence[i] = i;
    }
}

void SonarArray::startSonar()
{
    if (m_seqLen > 0) {
        if (m_current >= m_seqLen) {
            m_current = 0;
        }
    }
}

void SonarArray::stopSonar()
{
  
}

void SonarArray::setSequence(int length, int seq[])
{
    m_seqLen = min(length,4*MAX_PINS);
    for (int i=0; i<m_seqLen; i++)
    {
        m_sequence[i] = seq[i];
    }
}

bool SonarArray::sonarRunning()
{
  return (state == SONAR_IDLE || state == SONAR_PING_SENT);
}

// Sonar timer interrupt callback 
void sonarEchoCheck() 
{ // Timer2 interrupt calls this function every 24uS where you can check the ping status.
  if (SonarArray::m_currentSensor->check_timer()) { // This is how you check to see if the ping was received.
    // Here's where you can add code.
  }
}


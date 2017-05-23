/**
 * @brief This class encapsulates the use of a HC-SR04 style sonar
 */
#include "iping.h"
#include "sonararray.h"
#include "counters.h"

void sonarEchoCheck();
void sonarReport(unsigned long microsec);

IPing* SonarArray::m_currentSensor;                                     // Pointer to current sensor - for use in callback
void(*SonarArray::m_report)(int id, int value, unsigned long time_in_ms); // reporting function
int sensorId;                                                             // ID to use from callback
SonarArray * SonarArray::m_instance;                                      // Getting the instance from interrupt
IntervalTimer sonarInterval;

SonarArray::SonarArray(int noOfPins, const int* pins, int maxDistance, void(*report)(int id, int value, unsigned long time_in_ms))
{
    m_maxDistance = maxDistance;
    m_noOfPins = min(noOfPins, MAX_PINS);
    m_seqLen = m_noOfPins;
    for (int i=0; i < noOfPins; i++)
    {
        m_pins[i] = pins[i];
        m_sensorArray[i] = new IPing(m_pins[i], m_pins[i], maxDistance);
        m_sequence[i] = i;
    }
    m_report = report;
    m_instance = this;
}

SonarArray* SonarArray::getInstance()
{
    return m_instance;
}

void SonarArray::startSonar()
{
    if (m_seqLen > 0)
    {
        if (m_current >= m_seqLen)
        {
            m_current = 0;
        }
        sensorId = m_sequence[m_current];
        SonarArray::m_currentSensor = m_sensorArray[sensorId];
        if (SonarArray::m_currentSensor->pingAsync(sonarReport))
        {
            m_state = SONAR_PING_SENT;
        } else {
            cnt.inc(pingFail);
        }
    }
}

void SonarArray::nextSonar()
{
    //Serial.println("nextSonar");
    if (m_state != SONAR_STOPPED)
    {
        ++m_current;
        startSonar();
    }
}

void SonarArray::stopSonar()
{
    m_state = SONAR_STOPPED;
}

void SonarArray::setSequence(byte length, byte seq[])
{
    m_seqLen = min(length,4*MAX_PINS);
    for (int i=0; i<m_seqLen; i++)
    {
        m_sequence[i] = seq[i];
    }
}

bool SonarArray::sonarRunning()
{
    return (m_state == SONAR_IDLE || m_state == SONAR_PING_SENT);
}

/**
 * @brief callback from ping ISR handler
 */
void sonarReport(unsigned long microsec)
{
    SonarArray::m_report(sensorId, microsec, millis());
}

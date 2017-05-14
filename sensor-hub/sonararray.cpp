/* 
 *  This class encapsulates the use of a HC-SR04 style sonar
 *  
 */
#include "ESPing.h"
#include "sonararray.h"

void sonarEchoCheck();

NewPing* SonarArray::m_currentSensor;                                     // Pointer to current sensor - for use in callback
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
        m_sensorArray[i] = new NewPing(m_pins[i], m_pins[i], maxDistance);
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
        //Serial.println("startSonar");
        if (m_current >= m_seqLen) 
        {
            m_current = 0;
        }
        sensorId = m_sequence[m_current];
        //Serial1.print(char(48+sensorId));
        SonarArray::m_currentSensor = m_sensorArray[sensorId];
        SonarArray::m_currentSensor->ping_timer(sonarEchoCheck);
        m_state = SONAR_PING_SENT;
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
    return (m_state == SONAR_IDLE || m_state == SONAR_PING_SENT);
}

void sonarEchoCheck() 
{ // Timer2 interrupt calls this function every 24uS where you can check the ping status.
    PingTimerReturn ct = SonarArray::m_currentSensor->check_timer();
    if (ct == PING_ECHO)
    {
        Serial1.print('E');
        SonarArray::m_report(sensorId, SonarArray::m_currentSensor->ping_result, millis());
        SonarArray::getInstance()->nextSonar();
    }
    else if (ct == PING_TIMEOUT)
    {
        //Serial1.print('T');
        SonarArray::getInstance()->nextSonar();
    }
}


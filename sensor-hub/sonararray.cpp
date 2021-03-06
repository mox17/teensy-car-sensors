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

/*
 * @param noOfPins How many sonar sensors
 * @param Array of input pins
 * @param maxDistance Maximum distance to measure
 * @param report Callback function to deliver results
 */
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

/**
 * @brief Start a sonar measurement, using the next sonar in the sequence.
 */
bool SonarArray::startSonar()
{
    sensorId = m_sequence[m_current];
    SonarArray::m_currentSensor = m_sensorArray[sensorId];
    if (SonarArray::m_currentSensor->pingAsync(sonarReport))
    {
        m_state = SONAR_PING_SENT;
    } else {
        cnt.inc(pingFail);
        m_state = SONAR_PING_ERROR;
        return false;
    }
    return true;
}

unsigned SonarArray::getId()
{
    return m_sequence[m_current];
}

/**
 * @brief Start the next sonar
 *
 * @return false if sonar failed, true if success or stopped
 */
bool SonarArray::nextSonar()
{
    if (m_state != SONAR_STOPPED)
    {
        ++m_current;
        if (m_current >= m_seqLen)
        {
            m_current = 0;
        }
        return startSonar();
    }
    return true;
}

void SonarArray::stopSonar()
{
    m_state = SONAR_STOPPED;
}

/**
 * @brief Set a new polling sequence for sonar array
 *
 * Do sanity checks for sequence length and sonar indices.
 * @param length Number of entries in sequence
 * @param seq The byte sequence
 */
void SonarArray::setSequence(byte length, byte seq[])
{
    if ((length > 0) && (length <= 4*MAX_PINS))
    {
        m_seqLen = min(length,4*MAX_PINS);
        if (m_current > m_seqLen)
        {
            m_current = 0;
        }
        for (int i=0; i<m_seqLen; i++)
        {
            if (seq[i] < MAX_PINS)
            {
                m_sequence[i] = seq[i];
            } else {
                cnt.inc(badSonarIdx);
            }
        }
    } else {
        cnt.inc(badSeqLen);
    }
}

/**
 * @brief Status of sonar measurements
 * @return running or not
 */
bool SonarArray::sonarRunning()
{
    return (m_state == SONAR_IDLE || m_state == SONAR_PING_SENT);
}

/**
 * @return sonar internal state
 */
SonarArray::sonarState SonarArray::getState()
{
    return m_state;
}

/**
 * @brief callback from ping ISR handler
 *
 * Invoke function pointer to custom handler.
 */
void sonarReport(unsigned long microsec)
{
    SonarArray::m_report(sensorId, microsec, millis());
}

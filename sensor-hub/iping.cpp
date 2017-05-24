#include "iping.h"
#include <IntervalTimer.h>

IntervalTimer IPing::itimer;
unsigned int IPing::m_maxEchoTime;
unsigned long IPing::m_maxTime;
unsigned long IPing::pingResult;

/**
 * @brief constructor
 *
 * @param trigPin the input pin to trigger the sonar
 * @param echoPin the output pin where a times pulse indicates cdistane.
 * This may be the same as trigPin.
 * @param maxDistCm The maximum distance that will be reported.
 */
IPing::IPing(uint8_t trigPin, uint8_t echoPin, int maxDistCm)
{
    m_trigBit = digitalPinToBitMask(trigPin);  // Get the port register bitmask for the trigger pin.
    m_echoBit = digitalPinToBitMask(echoPin);  // Get the port register bitmask for the echo pin.
    m_trigOut = portOutputRegister(digitalPinToPort(trigPin)); // Get the output port register for the trigger pin.
    m_echoIn = portInputRegister(digitalPinToPort(echoPin));   // Get the input port register for the echo pin.
    m_trigMode = (uint8_t *) portModeRegister(digitalPinToPort(trigPin)); // Get the port mode register for the trigger pin.
    pinMode(trigPin, INPUT);
    pinMode(echoPin, INPUT);
    // Calculate the maximum distance in uS.
    m_maxEchoTime = min(maxDistCm, MAX_SENSOR_DISTANCE) * US_ROUNDTRIP_CM + (US_ROUNDTRIP_CM / 2);
    m_echoPin = echoPin;
}

void isrEcho();
void pingTimeout();
byte currentEchoPin=0;

/**
 * @brief Send trigger signal to sonar
 *
 * If the sonar module does not respond with a positive pulse
 * within time, this will return false
 * @return Success of initiating a ping
 */
boolean IPing::pingTrigger()
{
    *m_trigMode |= m_trigBit;  // Set trigger pin to output.
    *m_trigOut &= ~m_trigBit;  // Set the trigger pin low, should already be low, but this will make sure it is.
    delayMicroseconds(4);      // Wait for pin to go low, testing shows it needs 4uS to work every time.
    *m_trigOut |= m_trigBit;   // Set trigger pin high, this tells the sensor to send out a ping.
    delayMicroseconds(10);     // Wait long enough for the sensor to realize the trigger pin is high. Sensor specs say to wait 10uS.
    *m_trigOut  &= ~m_trigBit; // Set trigger pin back to low.
    *m_trigMode &= ~m_trigBit; // Set trigger pin to input (when using one Arduino pin this is technically setting the echo pin to input as both are tied to the same Arduino pin).

    m_maxTime = micros() + m_maxEchoTime + MAX_SENSOR_DELAY;   // Set a timeout for the ping to trigger.
    while ((*m_echoIn & m_echoBit) && (micros() <= m_maxTime))
    { // Wait for echo pin to clear.
    }
    digitalWrite(13,true);  // Make time spent here visible on a scope
    while (!(*m_echoIn & m_echoBit))
    {
        // Wait for ping to start.
        // TODO: this could be interrupt driven
        if (micros() > m_maxTime)
        {
            digitalWrite(13,false); // Make time spent here visible on a scope
            return false;
        }
    }
    digitalWrite(13,false); // Make time spent here visible on a scope
    m_maxTime = micros() + m_maxEchoTime;
    timerMicroS(m_maxTime, pingTimeout);
    // Use interrupt to determine echo time
    currentEchoPin = m_echoPin;
    attachInterrupt(currentEchoPin, isrEcho, FALLING);
    return true;
}

void (*reportFunction)(unsigned long);

/**
 * @brief Start an asynchronous ping
 *
 * @param report The callback where the result will be delivered.
 */
bool IPing::pingAsync(void (*report)(unsigned long))
{
    reportFunction = report; // Callback (interrupt context)
    bool result = pingTrigger();
    return result;
}

void (*itimerCallback)();

/**
 * @brief Micro second repeating timer
 *
 * @param microseconds Desired delay.
 * @param userFunc Callback function for timeout handling.
 */
void IPing::timerMicroS(unsigned int microseconds, void (*userFunc)(void))
{
    itimer.end();
    itimerCallback = userFunc; // User's function to call when there's a timer event.
    itimer.begin(itimerCallback, microseconds);
}

/**
 * @brief Stop running timer
 */
void IPing::timerStop()
{
    itimer.end();
}

/**
 * @brief Calculate duration from ping to echo
 */
void IPing::calculateTime()
{
    pingResult = (micros() - (m_maxTime - m_maxEchoTime) - 13);
}

/**
 * @brief Interrupt handler for falling edge of echo signal
 */
void isrEcho()
{
    if (currentEchoPin)
    {
        IPing::timerStop();
        detachInterrupt(currentEchoPin);
        currentEchoPin = 0;
        IPing::calculateTime();
        if (reportFunction)
        {
            reportFunction(IPing::pingResult);
        }
    }
}

/**
 * @brief Timeout handling when ISR has not detected an echo
 */
void pingTimeout()
{
    // Timeout happened before FALLING interrupt
    if (currentEchoPin)
    {
        detachInterrupt(currentEchoPin);
    }
    IPing::itimer.end();
    currentEchoPin = 0;
    IPing::pingResult = 0;
    if (reportFunction)
    {
        reportFunction(IPing::pingResult);
    }
}

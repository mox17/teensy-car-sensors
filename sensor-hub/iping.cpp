#include "iping.h"
#include <IntervalTimer.h>

IntervalTimer IPing::itimer;
unsigned int IPing::m_maxEchoTime;
unsigned long IPing::m_maxTime;
unsigned long IPing::pingResult;

IPing::IPing(uint8_t trigPin, uint8_t echoPin, int maxDistCm)
{
    m_trigBit = digitalPinToBitMask(trigPin); // Get the port register bitmask for the trigger pin.
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

/*
 * @brief Send trigger signal to sonar
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
    while (!(*m_echoIn & m_echoBit))
    {
        // Wait for ping to start.
        if (micros() > m_maxTime)
        {
            return false;
        }
    }
    m_maxTime = micros() + m_maxEchoTime; // Ping started, set the timeout.
    timerMicroS(m_maxTime, pingTimeout);
    // Use interrupt to determine echo time
    currentEchoPin = m_echoPin;
    attachInterrupt(currentEchoPin, isrEcho, FALLING);
    return true;                         // Ping started successfully.
}

void (*reportFunction)(unsigned long);

bool IPing::pingAsync(void (*report)(unsigned long))
{
    reportFunction = report; // Callback (interrupt context)
    bool result = pingTrigger();
    return result;
}

void (*itimerCallback)();

void IPing::timerMicroS(unsigned int microseconds, void (*userFunc)(void))
{
    itimer.end();
    itimerCallback = userFunc; // User's function to call when there's a timer event.
    itimer.begin(itimerCallback, microseconds);
}

void IPing::timerStop()
{
    itimer.end();
}

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
        IPing::calculateTime();
        detachInterrupt(currentEchoPin);
        Serial.println(char(97+currentEchoPin));
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
    Serial.println(char(64+currentEchoPin));
    currentEchoPin = 0;
    IPing::pingResult = 0;
    if (reportFunction)
    {
        reportFunction(IPing::pingResult);
    }
}

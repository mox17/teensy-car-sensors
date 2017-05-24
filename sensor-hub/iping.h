#pragma once

#if defined(ARDUINO) && ARDUINO >= 100
    #include <Arduino.h>
#else
    #include <WProgram.h>
    #include <pins_arduino.h>
#endif

#include <avr/io.h>
#include <avr/interrupt.h>

// Shoudln't need to changed these values unless you have a specific need to do so.
#define MAX_SENSOR_DISTANCE 500 // Maximum sensor distance can be as high as 500cm, no reason to wait for ping longer than sound takes to travel this distance and back.
#define US_ROUNDTRIP_CM 57      // Microseconds (uS) it takes sound to travel round-trip 1cm (2cm total), uses integer to save compiled code space.
#define DISABLE_ONE_PIN false   // Set to "true" to save up to 26 bytes of compiled code space if you're not using one pin sensor connections.

// Probably shoudln't change these values unless you really know what you're doing.
#define NO_ECHO 0               // Value returned if there's no ping echo within the specified MAX_SENSOR_DISTANCE or max_cm_distance.
#define MAX_SENSOR_DELAY 18000  // Maximum uS it takes for sensor to start the ping (SRF06 is the highest measured, just under 18ms).

class IPing
{
public:
    IPing(uint8_t trigPin, uint8_t echoPin, int maxDistCm = MAX_SENSOR_DISTANCE);
	bool pingAsync(void (*reportFunc)(unsigned long));
    static unsigned long pingResult;
    static void timerMicroS(unsigned int period, void (*userFunc)(void));
    static void timerStop();
	static void calculateTime();
    static IntervalTimer itimer;
private:
    boolean pingTrigger();
    uint8_t m_trigBit;  //!< bitmask for trigger I/O
    uint8_t m_echoBit;  //!< bitmask for echo I/O
    volatile uint8_t *m_trigOut;
    volatile uint8_t *m_trigMode;
    volatile uint8_t *m_echoIn;
    static unsigned int m_maxEchoTime;
    static unsigned long m_maxTime;
	uint8_t m_echoPin;  //!< Used for setting and removing interrupt handling
};

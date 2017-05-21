// ---------------------------------------------------------------------------
// Created by Tim Eckel - teckel@leethost.com
// Copyright 2012 License: GNU GPL v3 http://www.gnu.org/licenses/gpl-3.0.html
//
// See "ESPing.h" for purpose, syntax, version history, links, and more.
// ---------------------------------------------------------------------------
#include "ESPing.h"
#include <IntervalTimer.h>
IntervalTimer ESPing::itimer;

// ---------------------------------------------------------------------------
// ESPing constructor
// ---------------------------------------------------------------------------
ESPing::ESPing(uint8_t trigger_pin, uint8_t echo_pin, int max_cm_distance)
{
    _triggerBit = digitalPinToBitMask(trigger_pin); // Get the port register bitmask for the trigger pin.
    _echoBit = digitalPinToBitMask(echo_pin);       // Get the port register bitmask for the echo pin.

    _triggerOutput = portOutputRegister(digitalPinToPort(trigger_pin)); // Get the output port register for the trigger pin.
    _echoInput = portInputRegister(digitalPinToPort(echo_pin));         // Get the input port register for the echo pin.

    _triggerMode = (uint8_t *) portModeRegister(digitalPinToPort(trigger_pin)); // Get the port mode register for the trigger pin.

    // on Teensy 3.x (ARM), pins default to disabled
    // at least one pinMode() is needed for GPIO mode
    pinMode(trigger_pin, INPUT);
    pinMode(echo_pin, INPUT);
    _maxEchoTime = min(max_cm_distance, MAX_SENSOR_DISTANCE) * US_ROUNDTRIP_CM + (US_ROUNDTRIP_CM / 2); // Calculate the maximum distance in uS.

#if DISABLE_ONE_PIN == true
    *_triggerMode |= _triggerBit; // Set trigger pin to output.
#endif
}

// ---------------------------------------------------------------------------
// Standard ping methods
// ---------------------------------------------------------------------------
unsigned int ESPing::ping()
{
    if (!ping_trigger())
    {
        return NO_ECHO;                // Trigger a ping, if it returns false, return NO_ECHO to the calling function.
    }
    while (*_echoInput & _echoBit)
    {                      // Wait for the ping echo.
        if (micros() > _max_time)
        {
            return NO_ECHO;       // Stop the loop and return NO_ECHO (false) if we're beyond the set maximum distance.
        }
    }
    // Calculate ping time, 5uS of overhead.
	return (micros() - (_max_time - _maxEchoTime) - 5);
}

unsigned int ESPing::ping_cm()
{
    unsigned int echoTime = ESPing::ping();          // Calls the ping method and returns with the ping echo distance in uS.
    return PingConvert(echoTime, US_ROUNDTRIP_CM); // Convert uS to centimeters.
}

// ---------------------------------------------------------------------------
// Standard ping method support functions (not called directly)
// ---------------------------------------------------------------------------
boolean ESPing::ping_trigger()
{
#if DISABLE_ONE_PIN != true
    *_triggerMode |= _triggerBit;    // Set trigger pin to output.
#endif
    *_triggerOutput &= ~_triggerBit; // Set the trigger pin low, should already be low, but this will make sure it is.
    delayMicroseconds(4);            // Wait for pin to go low, testing shows it needs 4uS to work every time.
    *_triggerOutput |= _triggerBit;  // Set trigger pin high, this tells the sensor to send out a ping.
    delayMicroseconds(10);           // Wait long enough for the sensor to realize the trigger pin is high. Sensor specs say to wait 10uS.
    *_triggerOutput &= ~_triggerBit; // Set trigger pin back to low.
#if DISABLE_ONE_PIN != true
    *_triggerMode &= ~_triggerBit;   // Set trigger pin to input (when using one Arduino pin this is technically setting the echo pin to input as both are tied to the same Arduino pin).
#endif

    _max_time =  micros() + _maxEchoTime + MAX_SENSOR_DELAY;   // Set a timeout for the ping to trigger.
    while ((*_echoInput & _echoBit) && (micros() <= _max_time))
    { // Wait for echo pin to clear.
    }
    while (!(*_echoInput & _echoBit))
    {
        // Wait for ping to start.
        if (micros() > _max_time)
        {
            return false;                // Something went wrong, abort.
        }
    }
    _max_time = micros() + _maxEchoTime; // Ping started, set the timeout.
    return true;                         // Ping started successfully.
}

// ---------------------------------------------------------------------------
// Timer interrupt ping methods (won't work with ATmega8 and ATmega128)
// ---------------------------------------------------------------------------
void ESPing::ping_timer(void (*userFunc)(void))
{
    if (!ping_trigger())
    {
        return;         // Trigger a ping, if it returns false, return without starting the echo timer.
    }
    timer_us(ECHO_TIMER_FREQ, userFunc); // Set ping echo timer check every ECHO_TIMER_FREQ uS.
}

PingTimerReturn ESPing::check_timer()
{
    unsigned now = micros();
    if (now > _max_time)
    { // Outside the timeout limit.
        timer_stop();           // Disable timer interrupt
        return PING_TIMEOUT;    // Cancel ping timer.
    }
    if (!(*_echoInput & _echoBit))
    { // Ping echo received.
        timer_stop();                // Disable timer interrupt
        ping_result = (micros() - (_max_time - _maxEchoTime) - 13); // Calculate ping time, 13uS of overhead.
        return PING_ECHO;            // Return ping echo true.
    }
    return PING_WAITING; // Return false because there's no ping echo yet.
}

// ---------------------------------------------------------------------------
// Timer2/Timer4 interrupt methods (can be used for non-ultrasonic needs)
// ---------------------------------------------------------------------------

// Variables used for timer functions
void (*intFunc)();
void (*intFunc2)();
unsigned long _ms_cnt_reset;
volatile unsigned long _ms_cnt;

void ESPing::timer_us(unsigned int microseconds, void (*userFunc)(void))
{
    timer_setup();      // Configure the timer interrupt.
    intFunc = userFunc; // User's function to call when there's a timer event.
    itimer.begin(userFunc, microseconds);
}

void ESPing::timer_ms(unsigned long frequency, void (*userFunc)(void))
{
    timer_setup();                       // Configure the timer interrupt.
    intFunc = ESPing::timer_ms_cntdwn;  // Timer events are sent here once every ms till user's frequency is reached.
    intFunc2 = userFunc;                 // User's function to call when user's frequency is reached.
    _ms_cnt = _ms_cnt_reset = frequency; // Current ms counter and reset value.

    itimer.begin(ESPing::timer_ms_cntdwn, 1000.0);
}

void ESPing::timer_stop()
{ // Disable timer interrupt.
    itimer.end();
}

// ---------------------------------------------------------------------------
// Timer2/Timer4 interrupt method support functions (not called directly)
// ---------------------------------------------------------------------------
void ESPing::timer_setup()
{
    timer_stop();
}

void ESPing::timer_ms_cntdwn()
{
    if (!_ms_cnt--)
    {            // Count down till we reach zero.
        intFunc2();              // Scheduled time reached, run the main timer event function.
        _ms_cnt = _ms_cnt_reset; // Reset the ms timer.
    }
}

static void unusedfunction()
{
    if (intFunc)
    {
        intFunc(); // If wrapped function is set, call it.
    }
}

// ---------------------------------------------------------------------------
// Conversion methods (rounds result to nearest inch or cm).
// ---------------------------------------------------------------------------

unsigned int ESPing::convert_cm(unsigned int echoTime)
{
    return PingConvert(echoTime, US_ROUNDTRIP_CM); // Convert uS to centimeters.
}

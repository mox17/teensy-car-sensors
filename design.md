# Design notes

# Wheel rotation sensors
The wheel sensor is based on four hall-effect sensors, two on each rear wheel.
With two sensors on a wheel, the direction of rotation can detected through the phase difference between signals from the sensors.
Each wheel has 5 magnets. When a magnet passes a sensor, there is a pulse.
Each sensor will see 10 transitions for a full wheel revolution and with two sensors, this is 20 transitions per revolution.

Diameter approx | 9.5cm
Circumference | pi x 9.5 = 29.8cm 
5 magnets, 2 sensors | 20 pulses per rev.
Resolution | 29.8/20 = 1.5cm/pulse

## Estimating event timings
A useful baseline for an autonomous model car is walking speed 5km/h = 1.39m/s

So the pulserate for a single wheel at walking speed would be:
1.39/0.015 = 92.6 pulses/s

2 wheels -> approx 200 pulses/s @ 5km/h

Another assumption is the max speed we expect the car to drive. 30km/h seems a reasonable number to calculate.

Max speed 30km/h => 1200 pulses/s
0.83ms between wheel sensor interrupts.

The control software will not need wheel rotation updates every millisecond, so next a number of limit are defined for the event rate supplied to main computer. 
Testing may show a need to adjust this, but for now these are the working assumptions.

Max update speed 30 updates/s  33ms
Min update speed 5 updates/s.  200ms

@30km/h during 33ms 27.5cm is travelled.


The sensor data is communicates to the main car computer (currently a raspberry pi) over a serial link running 115200 baud.

## Speed calculation
The speed is reported as pulses per second. One pulse/s is lowest speed and corresponds to 1.5cm/s = 0.015m/s = 0.054km/h = 54m/h

## Event handling

### Sensor interrupt
Whenever an interrupt from one of the sensors is received, the following happens.

1. The overall pulse count for the wheel (left or right) is incremented.
2. The overall timestamp is updated.
3. The direction is calculated
   * If direction is valid and the same as previous direction, a record with (time, pulsecount) is added to a circular buffer. The buffer holds up to N latest entries. The count is updated, but never exceeds N, as old entried are discarded.
   * If the direction is valid but different from previous direction, the buffer is cleared and a single new record saved.
4. The rate of wheel events to main computer is limited to max one update per 5ms. If time since last update (there is a special timestamp for this) is 5ms or more, a new update is calculated and put on the TX queue for wheel events. This calculation includes speed in pulses/second, based on the first and last entry in the buffer. If only one sample is in the buffer, the speed is defined as 0.

### Timer events
The wheel events are sent at least every 200ms. A timer handler is called from main loop every 200ms. 

An interval timer could also be used, resources permitting.

If no wheel update have been sent for 200ms, then no interrupts have been received for 200ms.

A new event record with same pulse count and direction, but different timestamp is added to buffer, and speed is calculated from this.

### What is a proper definition of speed?
When coming to a stop, the timer events cause up to N events with a non-zero speed, even if no movement has been detected over the last N times 200ms.

This will cause the contradiction in the data that speed is non-zero, while pulse count is static.

Any dead-reckoning logic based on the wheel events should use the pulse counts and timestamps as basis for position calculation.
The speed is not valid for calculating position, but can be used for dynamic adjustments, such as motor throttle control.

# Sonar sensors
There are 6 sonar sensors. It is assumed that all sensors behave the same and are interfaced the same way.

These are modified [HC-SR04](http://www.electroschematics.com/8902/hc-sr04-datasheet/) sensors.
The modification is combining the TRIG input with the ECHO output in a way that is compatible with 3.3V levels used by the [Teensy-LC](https://www.pjrc.com/teensy/teensyLC.html).
This is done by removing 10K pull-ups on the HC-SR04 board from TRIG and ECHO and and connecting ECHO to trig via a 18K over 30K voltage divider.

The end result is a sensor which can be driven over a single GPIO pin connected to TRIG input.

To do a measurement the following steps are needed:
1. Set GPIO to output.
2. Set pin low for 4microseconds.
3. Set pin high for 10 microseconds.
4. Set GPIO to input.
5. Wait for input to go high (this is when ping has been sent).
  * Keep timestamp, let's call it pingSent.
  * This waiting can be done with an interrupt handler.
6. Wait for input to go low.
  * When this happens time is pingReceived.
  * This waiting can be done with an interrupt handler.

In the normal case, the result is pingReceived-pingSent. This is the round-trip time for the signal. Using the speed of sound a distance can be calculated.

A number of things can go wrong:
1. The sonar may be disconnected.
2. The sonar may not be ready for a new measurement yet.
3. No echo is ever received - how does the HC-SR04 react?
4. The sonar is defective and has an extremely short ECHO pulse.
5. ... (more failure modes to be discovered)



~~~~c++
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
~~~~


## Measurement scheduling
Since sonars all use the same audio frequencies, only one sonar can be active at a time.
If a sonar returns a result because of a very quick echo (some obstacle is close by), and the next sonar is immediately started, the 2nd sonar may receive a "late" secondary echo from the 1st sonar, and this can count as a false measurement from sonar #2.

It is therefore important to separate the sonar measurements in time to avoid this.
The sonars are specified to measure up to 400cm. With a speed of sound of approx 340m/s, an echo from 4m away will take about 24ms to arrive.

So each ping should be sent no closer than 24ms apart.
It will still be possible for a ping to return from more than 4m and be picked up by the next sonar, but the likelihood is diminishing because of the loss of audio signal stregth.

So a slightly conservative sonar measurement rate is one per 30ms, which is 33 measurements per second.

The default setting is using each sensor in turn, in a round-robin pattern.
^^^^^^^^
# Design notes

The wheel sensor is based on four hall-effect sensors, two on each rear wheel.
With two sensors on a wheel, the direction of rotation can detected through the phase difference between signals from the sensors.
Each wheel has 5 magnets. When a magnet passes a sensor, there is a pulse.
Each sensor will see 10 transitions for a full wheel revolution and with two sensors, this is 20 transitions per revolution.

Diameter approx 9.5cm
Circumference pi x 9.5 = 29.8cm
5 magnets, 2 sensors = 20 pulses per rev.
29.8/20 = 1.5cm/pulse

Walking speed 5km/h = 1.39m/s

1.39/0.015 = 92.6 pulses/s

2 wheels -> approx 200 pulses/s @ 5km/h
Max speed 30km/h => 1200 pulses/s
0.83ms between wheel sensor interrupts.

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


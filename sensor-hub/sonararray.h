/*
 *  This class encapsulates the use of a HC-SR04 style sonar
 *
 */
#ifndef SONARARRAY_H
#define SONARARRAY_H
#include "iping.h"
#include "common.h"

#define SONAR_PING_INTERVAL 50  // time between echo and next ping

class SonarArray {
public:
    enum sonarState {
        SONAR_STOPPED,   //!< No pinging
        SONAR_IDLE,      //!< Waiting to ping (timer running)
        SONAR_PING_SENT, //!< Waiting for echo
        SONAR_PING_ERROR //!< Error when initiating ping
    };
    SonarArray(int noOfPins, const int* pins, int maxDistance, void(*report)(int id, int value, unsigned long time_in_ms));
    bool startSonar();
    void stopSonar();
    bool sonarRunning();
    void setSequence(byte length, byte seq[]);
    bool nextSonar();
    unsigned getId();

    static IPing* m_currentSensor;
    static void(*m_report)(int id, int value, unsigned long time_in_ms);
    static SonarArray* getInstance();
    sonarState getState();

private:
    static const int MAX_PINS=MAX_NO_OF_SONAR;
    int m_noOfPins;
    int m_pins[MAX_PINS];
    IPing *m_sensorArray[MAX_PINS];
    int m_maxDistance;
    int m_sequence[4*MAX_PINS];  //!< a sequence of up to 24
    int m_seqLen;                //!< Length of current sequence
    volatile int m_current;      //!< index into sequence
    sonarState m_state=SONAR_STOPPED;
    static SonarArray *m_instance; // This class is a singleton
};

#endif // SONARARRAY_H

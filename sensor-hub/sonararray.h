/* 
 *  This class encapsulates the use of a HC-SR04 style sonar
 *  
 */
#include <NewPing.h>

class SonarArray {

public:
    enum sonarState {
        SONAR_STOPPED,   // No pinging
        SONAR_IDLE,      // Waiting to ping (timer running)
        SONAR_PING_SENT, // Waiting for echo
    };
    SonarArray(int noOfPins, const int* pins, int maxDistance, void(*report)(int id, int value, unsigned long time_in_ms));
    void startSonar();
    void stopSonar();
    bool sonarRunning();
    void setSequence(int length, int seq[]);
    void nextSonar();
    
    static NewPing* m_currentSensor;
    static void(*m_report)(int id, int value, unsigned long time_in_ms);
    static SonarArray* getInstance();

private:
    static const int MAX_PINS=6;
    int m_noOfPins;
    int m_pins[MAX_PINS];
    NewPing *m_sensorArray[MAX_PINS];
    int m_maxDistance;
    int m_sequence[4*MAX_PINS];  // a sequence of up to 24
    int m_seqLen;   // Length of current sequence
    int m_current;  // index into sequence
    sonarState m_state=SONAR_STOPPED;
    static SonarArray *m_instance;
};


/* 
 *  This class encapsulates the use of a HC-SR04 style sonar
 *  
 */

class SonarArray {

public:
    enum sonarState {
        SONAR_STOPPED,   // No pinging
        SONAR_IDLE,      // Waiting to ping (timer running)
        SONAR_PING_SENT, // Waiting for echo
    };
    SonarArray(int noOfPins, const int* pins, int maxDistance);
    void startSonar();
    void stopSonar();
    bool sonarRunning();
    void setSequence(int length, int seq[]);

    static NewPing* m_currentSensor;

private:
    static const int MAX_PINS=6;
    int m_noOfPins;
    int m_pins[MAX_PINS];
    NewPing *m_sensorArray[MAX_PINS];
    int m_maxDistance;
    int m_sequence[4*MAX_PINS];
    int m_seqLen;
    int m_current;
    sonarState state=SONAR_STOPPED;
};


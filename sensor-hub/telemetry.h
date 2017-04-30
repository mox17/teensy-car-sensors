/* 
 * Data message formats
 * | 7e | dest | src | cmd | reserved | data ... | chksum | 7e |
 * 
 * Wire format has 7e and checksum. these are removed at reception. 
 * Packets with wrong checksum are dropped and error counter increased.
 * 
 * dest, src = address. RPi = 1. Teensy = 2, ALL = 255, invalid = 0
 * reserved = 0 (filler byte)
 * data = 0 or more bytes
 * 
 */
#include <Arduino.h>

#define ADDR_RPI 1
#define ADDR_TEENSY 2
#define ADDR_ALL 255

enum command 
{
    CMD_PING       = 1,  // 4 bytes payload (time stamp for example)
    CMD_PONG       = 2,  // 4 byte payload from ping is copied back.
    CMD_US_SET_SEQ = 3,  // up to 24 bytes. unused bytes are 0xff
    CMD_US_STOP    = 4,  // Stop Ultrasound sensors
    CMD_US_START   = 5,  // Start Ultrasound sensors 
    CMD_US_STATUS  = 6,  // 6 distances (cm, 16bit)
    CMD_ROT_STATUS = 7,  // direction, speed, odo
    CMD_ROT_RESET  = 8   // Clear odometer
};

// For indexing rot fird in payload
enum {
  ROT_LEFT,
  ROT_RIGHT
};

typedef struct _rotation 
{
    word speed;         //
    bool direction;    // forward
    byte reserved;
    uint32_t dist;     // since direction change
    uint32_t dist_abs; // absolute direction travelled
} rotation;

typedef union _SensorData 
    {
      uint32_t timestamp;    // PING & PONG
      byte seq[24];          // SET_SEQ
      uint32_t distances[6]; // US_STATUS
      rotation rot[2];       // ROT_STATUS Left & Right
    } SensorData;

class Payload 
{
private:
    byte dst;
    byte src;
    byte cmd;
    byte reserved;
    SensorData data;

public:
    size_t getPacketLength()
    {
        switch (cmd) 
        {
        case CMD_PING:  
        case CMD_PONG:
            return 4;
            break;
        case CMD_US_SET_SEQ:
            return sizeof(Payload);
            break;
        case CMD_US_STOP:
        case CMD_US_START:
            return 4;
            break;
        case CMD_US_STATUS:
            return 4+6*4;
            break;
        case CMD_ROT_STATUS:
            return 4+12*2;
            break;
        case CMD_ROT_RESET:
            return 4;
            break;
        }
    }
  
    void setPing(uint32_t val)
    {
        dst = ADDR_RPI;
        src = ADDR_TEENSY;
        cmd = CMD_PING;
        reserved = 0;
        data.timestamp = val;
    }
  
};

class Packet 
{
public:
    
};

void serialPolling();

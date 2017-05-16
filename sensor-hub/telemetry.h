/* 
 * Data message formats
 * | 0x7e | dest | src | cmd | reserved | data ... | chksum | 0x7e |
 * 
 * Wire format has 0x7e and checksum. these are removed at reception. 
 * Packets with wrong checksum are dropped and error counter increased.
 * 
 * dest, src = address. RPi = 0x01. Teensy = 0x02, ALL = 0xff, invalid = 0x00
 * cmd = command opcode, see enumeration below.
 * reserved = 0x00 (filler byte)
 * data = 0 or more bytes
 * 
 */
#include <Arduino.h>
#include "common.h"
#include "sonararray.h"

#define ADDR_RPI    0x01
#define ADDR_TEENSY 0x02
#define ADDR_ALL    0xff

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

struct header
{
    byte dst;
    byte src;
    byte cmd;
    byte reserved;
};

struct pingpong
{
    struct header hdr;
    uint32_t timestamp;  
};

struct sequence
{
    struct header hdr;
    byte sequence[4*MAX_NO_OF_SONAR];
};

struct distances
{
    struct header hdr;
    uint16_t distances[MAX_NO_OF_SONAR];
};

struct rot_one
{
    word speed;        // Pulses per second (lowest possible speed is 20 seconds for one wheel revolution)
    byte direction;    // enumeration rotDirection is used
    byte reserved;     // Filler for alignment
    uint32_t dist;     // since direction change
    uint32_t dist_abs; // absolute direction travelled
};

struct rotation
{
    struct header hdr;
    struct rot_one rot[2]; // Indexed with enumeration rotSide
};

// This union is only for calculating max message size
union payload
{
    struct pingpong pp;
    struct sequence sq;
    struct distances ds;
    struct rotation rt;
};

#define MAX_MSG_SIZE sizeof(payload)

/*
 * This union contains all possible messages.
 */
typedef union 
{
    byte raw[MAX_MSG_SIZE];
    struct header hdr;
    struct pingpong pp;
    struct sequence sq;
    struct distances ds;
    struct rotation rt;
} packet;

void serialPolling();

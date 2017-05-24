/**
 * Common definitions between Raspberry Pi and Teensy.
 * This covers message frame format and defined messages.
 *
 * RPi <-> Teensy Protocol
 * Data message formats
 * | 0x7e | dest | src | cmd | reserved | data ... | chksum | 0x7e |
 *
 *  - Wire format has PPP style framing (0x7e,0x7d) and checksum.
 *  - The frame delimiter (0x7e) can occur as a single delimiter between two
 *    packets sets back-to-back.
 *  - Framing and checksum is removed at reception.
 *  - Packets or zero length are removed.
 *  - Packets with wrong checksum are dropped and error counter increased.
 *  - Packet length is implicit.
 *
 * Message header:
 *   dest, src = address. RPi = 0x01. Teensy = 0x02, ALL = 0xff, invalid = 0x00
 *   cmd = command opcode, see enumeration below.
 *   reserved = 0x00 (filler byte)
 * Payload:
 *   data = 0 or more bytes
 * Checksum:
 *
 */
#pragma once
#include "queuelist.h"
// Packet framing (inspired by PPP)
#define FRAME_START_STOP  0x7e
#define FRAME_DATA_ESCAPE 0x7d
#define FRAME_XOR         0x20

// Defined addresses
#define ADDR_RPI    0x01
#define ADDR_TEENSY 0x02
#define ADDR_ALL    0xff

/**
 * @brief Command opcodes
 */
enum command
{
    CMD_PING_QUERY    = 1,  // timestamp exchange
    CMD_PONG_RESP     = 2,  // timestamp exchange
    CMD_SET_SONAR_SEQ = 3,  // up to 24 bytes. unused bytes are 0xff
    CMD_SONAR_STOP    = 4,  // Stop Ultrasound sensors
    CMD_SONAR_START   = 5,  // Start Ultrasound sensors
    CMD_SONAR_STATUS  = 6,  // sensor id + distance
    CMD_WHEEL_STATUS  = 7,  // direction, speed, odo
    CMD_WHEEL_RESET   = 8,  // Clear odometer
    CMD_ERROR_COUNT   = 9,  // Error counter name and value
    CMD_GET_COUNTERS  = 10, // Ask teensy to send all non-zero counters
    CMD_SONAR_RETRY   = 11, // Do a repeat nextSonar() (for internal stall recovery)
};

struct header
{
    uint8_t dst;
    uint8_t src;
    uint8_t cmd;
    uint8_t reserved;
} __attribute__((packed));

/**
 * @brief Estimate transition time through exchange of timestamps.
 */
struct pingpong
{
    struct header hdr;
    uint32_t timestamp1;  // ping sender fill in this (pong sender copies)
    uint32_t timestamp2;  // ping sets to 0 (pong sender fill this)
} __attribute__((packed));

// Sonar sensor definitions
const unsigned MAX_NO_OF_SONAR = 6;

/**
 * Define polling sequence of sonars.
 * Same sonar id can be repeated up to 4 times.
 */
struct sequence
{
    struct header hdr;
    uint8_t len;  // Valid bytes in sequence array
    uint8_t sequence[4*MAX_NO_OF_SONAR];
} __attribute__((packed));

/**
 * @brief Report distance from a single sonar.
 */
struct distance
{
    struct header hdr;
    uint8_t sensor;       // 0 .. MAX_NO_OF_SONAR-1
    uint8_t filler;       // Alignment
    uint16_t distance; // Measurements in microseconds
    uint32_t when;     // Time when data was measured
} __attribute__((packed));

// Wheel sensor definitions
enum rotSide {
    ROT_LEFT,
    ROT_RIGHT
};

/*
 * The rotation enumeration represents the possible states when using
 * phase difference between two sensors.
 * If active periods are not overlapping, ambiguity can occur.
 */
enum rotDirection {
    ROT_DIR_FORWARD,
    ROT_DIR_BACKWARD,
    ROT_DIR_NONE,
    ROT_DIR_ERROR, // this can occur if active phases from the two sensors are not overlapping
};

struct rot_one
{
    uint16_t speed;        // Pulses per second (lowest possible speed is 20 seconds for one wheel revolution)
    uint8_t direction;    // Enumeration rotDirection is used
    uint8_t reserved;     // Filler for alignment
    uint32_t when;     // Timestamp for measurement
    uint32_t dist;     // Odometer when direction changed
    uint32_t dist_abs; // Absolute distance travelled
} __attribute__((packed));

struct rotation
{
    struct header hdr;
    struct rot_one rot[2]; // Indexed with enumeration rotSide
} __attribute__((packed));

struct errorcount
{
    struct header hdr;
    uint32_t count;  // Counter value
    char name[24];   // ASCIIZ name
} __attribute__((packed));

// This union is only for calculating max message size
union payload
{
    struct pingpong pp;
    struct sequence sq;
    struct distance ds;
    struct rotation rt;
    struct errorcount ec;
} __attribute__((packed));

const size_t MAX_MSG_SIZE = sizeof(payload);

struct packet : node
{
    union
    {
        uint8_t raw[MAX_MSG_SIZE+4]; // Raw bytes (for UART RX and RX). Allow 1 byte extra for checksum
        struct header hdr;        // Minimal header for decoding
        struct pingpong pp;       // message specific layouts...
        struct sequence sq;
        struct distance ds;
        struct rotation rt;
        struct errorcount ec;
    };
} __attribute__((packed));

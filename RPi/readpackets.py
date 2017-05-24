#!/usr/bin/env python
from __future__ import print_function
from collections import namedtuple
from datetime import datetime
import serial


def locate(user_string, x=0, y=0):
        # Don't allow any user errors. Python's own error detection will check for
        # syntax and concatination, etc, etc, errors.
        x=int(x)
        y=int(y)
        if x>=255: x=255
        if y>=255: y=255
        if x<=0: x=0
        if y<=0: y=0
        HORIZ=str(x)
        VERT=str(y)
        # Plot the user_string at the starting at position HORIZ, VERT...
        print("\033["+VERT+";"+HORIZ+"f"+user_string)

def prettyPrintDist(dist):
    sensor = dist.sensor
    locate(' {:10d} cm'.format(dist.distance), 15*sensor, 1)
    locate(' {:10} ms'.format(dist.when), 1, 8)
    locate("",0,9)
    return

def prettyPrintWheels(left, right):
    locate(' {:10d} pulses  {:10d} pulses'.format(left.dist, right.dist), 1, 3)
    locate(' {:10} p/s     {:10} p/s'.format(left.speed,     right.speed), 1, 4)
    locate(' {:10} dir     {:10} dir'.format(left.direction, right.direction), 1, 5)
    locate(' {:10} turn    {:10} turn'.format(left.turn,      right.turn), 1, 6)
    locate("",0,9)
    return

# Utility access functions 
def get32(bytes):
    val = bytes[0] + 0x100*bytes[1] + 0x10000*bytes[2] + 0x1000000*bytes[3]
    return val

def get16(bytes):
    val = bytes[0] + 0x100*bytes[1]
    return val

def getStr(bytes):
    s = ""
    for b in bytes:
        if b != 0:
            s = s + chr(b)
        else:
            break
    return s

# helper function
def enum(**enums):
    return type('Enum', (), enums)

# Protocol commands
Cmd = enum(\
    CMD_PING_QUERY    = 1,  #// timestamp exchange
    CMD_PONG_RESP     = 2,  #// timestamp exchange
    CMD_SET_SONAR_SEQ = 3,  #// up to 24 bytes. unused bytes are 0xff
    CMD_SONAR_STOP    = 4,  #// Stop Ultrasound sensors
    CMD_SONAR_START   = 5,  #// Start Ultrasound sensors
    CMD_SONAR_STATUS  = 6,  #// sensor id + distance
    CMD_WHEEL_STATUS  = 7,  #// direction, speed, odo
    CMD_WHEEL_RESET   = 8,  #// Clear odometer
    CMD_ERROR_COUNT   = 9,  #// Error counter name and value
    CMD_GET_COUNTERS  = 10, #// Ask teensy to send all non-zero counters
    CMD_SONAR_RETRY   = 11, #// Do a repeat nextSonar() (for internal stall recovery)
    )       

# Packet framing bytes
FRAME_START_STOP  = 0x7e
FRAME_DATA_ESCAPE = 0x7d
FRAME_XOR         = 0x20

# Defined addresses
ADDR_RPI    = 0x01
ADDR_TEENSY = 0x02
ADDR_ALL    = 0xff

# Protocol data fields
header     = namedtuple("header", "dst src cmd rsv")
pingpong   = namedtuple("pingpong", "timestamp1 timestamp2")
distance   = namedtuple("distance", "sensor distance when")
rotation   = namedtuple("rotation", "speed direction when turn dist")
errorcount = namedtuple("errorcount", "count name")

def getPingPong(frm):
    """
    struct pingpong
    {
        struct header hdr;
        uint32_t timestamp1;  // ping sender fill in this (pong sender copies)
        uint32_t timestamp2;  // ping sets to 0 (pong sender fill this)
    } __attribute__((packed));
    """
    pp = pingpong(get32(frm), get32(frm[4:]))
    return pp

def getSonarStatus(frm):
    """
    struct distance
    {
        struct header hdr;
        uint8_t sensor;    // 0 .. MAX_NO_OF_SONAR-1
        uint8_t filler;    // Alignment
        uint16_t distance; // Measurements in microseconds
        uint32_t when;     // Time when data was measured
    } __attribute__((packed));
    """
    dist = distance(frm[0], get16(frm[2:])/57, get32(frm[4:]))
    return dist

def getRotation(frm):
    """
    struct rot_one
    {
        uint16_t speed;    // 0 Pulses per second (lowest possible speed is 20 seconds for one wheel revolution)
        uint8_t direction; // 2 Enumeration rotDirection is used
        uint8_t reserved;  // 3 Filler for alignment
        uint32_t when;     // 4 Timestamp for measurement
        uint32_t dist;     // 8 Odometer when direction changed
        uint32_t dist_abs; //12 Absolute distance travelled
    } __attribute__((packed));
    """
    rot = rotation(get16(frm), frm[2], get32(frm[4:]), get32(frm[8:]), get32(frm[12:]))
    return rot

def getErrorCount(frm):
    """
    struct errorcount
    {
        struct header hdr;
        uint32_t count;  // Counter value
        char name[24];   // ASCIIZ name
    } __attribute__((packed));
    """
    err = errorcount(get32(frm), getStr(frm[4:]))
    return err

def decodeFrame(frm):
    global Cmd
    if len(frm)< 4:
        return
    hdr = header(frm[0], frm[1], frm[2], frm[3])
    #print(hdr)
    frm = frm[4:]  # remove header and checksum
    if hdr.cmd == Cmd.CMD_PING_QUERY :
        pp = getPingPong(frm)
        print("PING", pp)
    elif hdr.cmd == Cmd.CMD_PONG_RESP:
        pp = getPingPong(frm)
        print("PONG", pp)
    elif hdr.cmd == Cmd.CMD_SET_SONAR_SEQ:
        pass
    elif hdr.cmd == Cmd.CMD_SONAR_STOP:
        pass
    elif hdr.cmd == Cmd.CMD_SONAR_START:
        pass
    elif hdr.cmd == Cmd.CMD_SONAR_STATUS:
        dist = getSonarStatus(frm)
        prettyPrintDist(dist)
        #print(dist)
    elif hdr.cmd == Cmd.CMD_WHEEL_STATUS:
        left  = getRotation(frm)
        right = getRotation(frm[16:])
        #print("Rotation", left, right)
        prettyPrintWheels(left, right)
        pass
    elif hdr.cmd == Cmd.CMD_WHEEL_RESET:
        pass
    elif hdr.cmd == Cmd.CMD_ERROR_COUNT:
        err = getErrorCount(frm)
        print(err)
        
    return

# Possible receiver states 
rxStateBegin = 0
rxStateData  = 1

escapeFlag = False
rxState = rxStateBegin

rawData = []
checksum = 0

def checksumCalc(b):
    global checksum
    checksum += b
    checksum += (checksum >> 8);
    checksum = checksum & 0xff
    return

def checksumNew():
    global checksum
    checksum = 0
    return

def handleFrame():
    global rawData
    if (len(rawData) > 0):
        #print(len(rawData), rawData)
        if checksum != 0xff:
            print("checksum error:", checksum, "\nerror data:", len(rawData), rawData)
        else:
            decodeFrame(rawData[:-1])
    return

def newFrame():
    global rawData
    rawData = []
    checksumNew()
    return

def decodeByte(character):
    global rxState, escapeFlag
    b = ord(character)
    if rxState == rxStateBegin:
        if b == FRAME_START_STOP:
            rxState = rxStateData
            newFrame()
    elif rxState == rxStateData:
        if b == FRAME_START_STOP:
            handleFrame()
            newFrame()
            return
        elif b == FRAME_DATA_ESCAPE:
            escapeFlag = True
            return
        else:
            if escapeFlag == True:
                escapeFlag = False
                b = b ^ FRAME_XOR
            checksumCalc(b)
            rawData.append(b)
    else:
        assert("Internal error")
    return

def buildHeader(dst, src, cmd, buf=None):
    if buf == None:
        buf = []
    buf.append(dst)
    buf.append(src)
    buf.append(cmd)
    buf.append(0)
    return buf
    
def add32(buf, value):
    buf.append(value & 0xff)
    value >>= 0xff
    buf.append(value & 0xff)
    value >>= 0xff
    buf.append(value & 0xff)
    value >>= 0xff
    buf.append(value & 0xff)
    return buf

def add16(buf, value):
    buf.append(value & 0xff)
    value >>= 0xff
    buf.append(value & 0xff)
    return buf

def sendPing(timestamp1):
    buffer = buildHeader(ADDR_TEENSY, ADDR_RPI, Cmd.CMD_PING)
    dt = datetime.now()
    milliSecond = dt.microsecond / 1000
    buffer = add32(buf, milliSecond) # timestamp1
    buffer = add32(buf, 0)           # timestamp2
    # Put buffer on tx queue
    return

def getTxByte():
    return [False,None]

def main():
    for n in range(0, 64, 1): print("\r\n")
    connected = False
    port = '/dev/ttyUSB0'
    baud = 115200
    
    ser = serial.Serial(port, baud, timeout=0)
    
    while not connected:
        #serin = ser.read()
        connected = True
    
        while (True):
            if (ser.inWaiting()>0):
                data_str = ser.read(ser.inWaiting())
                for b in data_str:
                    decodeByte(b)
            txData = getTxByte()
            if txData[0]:
                ser.write(txData[1])
            #Put the rest of your code you want here
        return

if __name__ == '__main__':
    main()

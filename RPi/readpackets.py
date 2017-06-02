#!/usr/bin/env python
from __future__ import print_function
from collections import namedtuple
from datetime import datetime
import serial
import Queue
import select
import sys
import termios
import atexit

def getMilliSeconds():
    dt = datetime.now()
    return dt.second * 1000 + dt.microsecond / 1000

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
    return

def prettyPrintDist(dist):
    sensor = dist.sensor
    locate(' {:10} cm'.format(dist.distance), 15*sensor, 1)
    locate(' {:10} ms'.format(dist.when), 1, 8)
    locate("",0,9)
    return

def prettyPrintWheels(left, right):
    locate(' {:10} pulses  {:10} pulses'.format(left.dist, right.dist), 1, 3)
    locate(' {:10} p/s     {:10} p/s'.format(left.speed,     right.speed), 1, 4)
    locate(' {:10} dir     {:10} dir'.format(left.direction, right.direction), 1, 5)
    locate(' {:10} turn    {:10} turn'.format(left.turn,      right.turn), 1, 6)
    locate(' {:10} ms'.format(left.when), 1, 8)
    locate("",0,9)
    return

def prettyPrintPong(resp, now):
    locate('{:10}ms  {:10}ms'.format(now -resp.timestamp1, resp.timestamp2), 25, 8)
    return

def clearScreen():
    print("\033[2J")
    return

# Utility access functions 
def get32(bytes):
    val = bytes[0] + (bytes[1]<<8) + (bytes[2]<<16) + (bytes[3]<<24)
    return val

def get16(bytes):
    val = bytes[0] + (bytes[1]<<8)
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
    CMD_SONAR_WAIT    = 12, #// Set waiting time between sonar pings in ms
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
#sonarwait  = namedtuple("pause")

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

def rxDecodeFrame(frm):
    global Cmd

    if len(frm)< 4:
        return
    hdr = header(frm[0], frm[1], frm[2], frm[3])
    #print(hdr)
    frm = frm[4:]  # remove header and rxChecksum
    if hdr.cmd == Cmd.CMD_PING_QUERY :
        pp = getPingPong(frm)
        print("PING", pp)
    elif hdr.cmd == Cmd.CMD_PONG_RESP:
        pp = getPingPong(frm)
        prettyPrintPong(pp, getMilliSeconds())
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
    elif hdr.cmd == Cmd.CMD_WHEEL_RESET:
        pass
    elif hdr.cmd == Cmd.CMD_ERROR_COUNT:
        err = getErrorCount(frm)
        print("\033[K",err)
        
    return

# Possible receiver states
rxs = enum(
    RS_BEGIN = 0,
    RS_DATA  = 1,
    )

rxEscapeFlag = False
rxState = rxs.RS_BEGIN

rxRawData = []
rxChecksum = 0

def rxChecksumCalc(b):
    global rxChecksum
    rxChecksum += b
    rxChecksum += (rxChecksum >> 8);
    rxChecksum = rxChecksum & 0xff
    return

def rxChecksumNew():
    global rxChecksum
    rxChecksum = 0
    return

def rxHandleFrame():
    global rxRawData
    if (len(rxRawData) > 0):
        if rxChecksum != 0xff:
            print("rxChecksum error:", rxChecksum, "\nerror data:", len(rxRawData), rxRawData)
        else:
            rxDecodeFrame(rxRawData[:-1])
    return

def rxNewFrame():
    global rxRawData
    rxRawData = []
    rxChecksumNew()
    return

def rxRawByte(character):
    global rxState, rxEscapeFlag
    b = ord(character)
    if rxState == rxs.RS_BEGIN:
        if b == FRAME_START_STOP:
            rxState = rxs.RS_DATA
            rxNewFrame()
    elif rxState == rxs.RS_DATA:
        if b == FRAME_START_STOP:
            rxHandleFrame()
            rxNewFrame()
            return
        elif b == FRAME_DATA_ESCAPE:
            rxEscapeFlag = True
            return
        else:
            if rxEscapeFlag == True:
                rxEscapeFlag = False
                b = b ^ FRAME_XOR
            rxChecksumCalc(b)
            rxRawData.append(b)
    else:
        assert("Internal error")
    return

# Low-level packet helpers
def buildHeader(dst, src, cmd, buf=None):
    if buf == None:
        buf = []
    buf.append(dst)
    buf.append(src)
    buf.append(cmd)
    buf.append(0) # Reserved field
    return buf

def add32(buf, value):
    buf.append(value & 0xff)
    value >>= 8
    buf.append(value & 0xff)
    value >>= 8
    buf.append(value & 0xff)
    value >>= 8
    buf.append(value & 0xff)
    return buf

def add16(buf, value):
    buf.append(value & 0xff)
    value >>= 8
    buf.append(value & 0xff)
    return buf

# High-level packet sending helpers

def sendPing():
    buffer = buildHeader(ADDR_TEENSY, ADDR_RPI, Cmd.CMD_PING_QUERY)
    buffer = add32(buffer, getMilliSeconds()) # timestamp1
    buffer = add32(buffer, 0)           # timestamp2
    txQueue.put(buffer)
    return

def sendSonarStop():
    buffer = buildHeader(ADDR_TEENSY, ADDR_RPI, Cmd.CMD_SONAR_STOP)
    txQueue.put(buffer)
    return

def sendSonarStart():
    buffer = buildHeader(ADDR_TEENSY, ADDR_RPI, Cmd.CMD_SONAR_START)
    txQueue.put(buffer)
    return

def sendWheelReset():
    buffer = buildHeader(ADDR_TEENSY, ADDR_RPI, Cmd.CMD_WHEEL_RESET)
    txQueue.put(buffer)
    return

def sendGetCounters():
    buffer = buildHeader(ADDR_TEENSY, ADDR_RPI, Cmd.CMD_GET_COUNTERS)
    txQueue.put(buffer)
    return

def sendSonarWait(pauseInMs):
    buffer = buildHeader(ADDR_TEENSY, ADDR_RPI, Cmd.CMD_SONAR_WAIT)
    buffer = add32(buffer, pauseInMs)
    txQueue.put(buffer)
    return

def sendSonarSequence(sequence):
    buffer = buildHeader(ADDR_TEENSY, ADDR_RPI, Cmd.CMD_SET_SONAR_SEQ)
    l = len(sequence)
    if (l > 0 and l <= 24):
        buffer.append(l)
        buffer = buffer + sequence
        txQueue.put(buffer)
    return

# TX side of packet handling
# The TX queue have byte arrays 
txQueue = Queue.Queue()
txCurrPacket = None

def txAppendChecksum(buf):
    sum = 0
    for b in buf:
        sum += b
        sum += sum >> 8
        sum = sum & 0xff
    buf.append(0xff-sum)
    return buf

def txGetNextPacket():
    global txQueue
    global txCurrPacket

    if txCurrPacket == None or len(txCurrPacket) == 0:
        if not txQueue.empty():
            txCurrPacket = txAppendChecksum(txQueue.get())
            return True
    return False

def txGetDataByte():
    # return tuple (dataAvailable, byteToSend)
    global txCurrPacket

    if txCurrPacket == None:
        txGetNextPacket()
    if txCurrPacket != None:
        if len(txCurrPacket) > 0:
            b = txCurrPacket[0]
            txCurrPacket = txCurrPacket[1:]
            return [True, b]
    return [False,None]

def txDataAvailable():
    if txState != txs.TS_BEGIN or (txCurrPacket != None and len(txCurrPacket) > 0):
        return True
    else:
        return txGetNextPacket()

# TX state machine states
txs = enum(
        TS_BEGIN  = 1,  #//!< Nothing sent yet, deliver 0x7e
        TS_DATA   = 2,  #//!< Sending normal data
        TS_ESCAPE = 3,  #//!< Escape has been sent, txEscByte is next
        )

txState = txs.TS_BEGIN
txEscByte = None

def txGetByte():
    global txState
    global txEscByte

    if txState == txs.TS_BEGIN:
        if txDataAvailable():
            txState = txs.TS_DATA
            return [True, FRAME_START_STOP]
        else:
            return [False,None]

    elif txState == txs.TS_DATA:
        dataAvailable, byte = txGetDataByte()
        if dataAvailable == True:
            if (byte == FRAME_START_STOP) or \
               (byte == FRAME_DATA_ESCAPE):
                txEscByte = byte ^ FRAME_XOR
                byte = FRAME_DATA_ESCAPE
                txState = txs.TS_ESCAPE
            return [True, byte]
        else:
            # This packet done - next one back-to-back?
            if txGetNextPacket():
                txState = txs.TS_DATA
            else:
                txState = txs.TS_BEGIN
            return [True, FRAME_START_STOP]

    elif txState == txs.TS_ESCAPE:
        txState = txs.TS_DATA
        return [True, txEscByte]

    return [False,None]

# User input handling
def handleKeyPress(key):
    key = key.upper()
    status = \
    { 'P': sendPing() or True,
      'N': sendSonarStop() or True,
      'S': sendSonarStart() or True,
      'R': sendWheelReset() or True,
      'C': sendGetCounters() or True,
      '\x0C' : clearScreen() or True,
      'X': sendSonarSequence([0,1,5]) or True,
      'Y': sendSonarSequence([0,1,2,3,4,5]) or True,
      'Q': False,
    }[key]
    return status


old_settings=None

def init_anykey():
    global old_settings
    old_settings = termios.tcgetattr(sys.stdin)
    new_settings = termios.tcgetattr(sys.stdin)
    new_settings[3] = new_settings[3] & ~(termios.ECHO | termios.ICANON) # lflags
    new_settings[6][termios.VMIN] = 0  # cc
    new_settings[6][termios.VTIME] = 0 # cc
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, new_settings)
    atexit.register(term_anykey)
    return

def term_anykey():
    global old_settings
    if old_settings:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    return

def main():
    clearScreen()

    connected = False
    port = '/dev/ttyUSB0'
    baud = 115200
    ser = serial.Serial(port, baud, timeout=0)
    
    init_anykey()
    while True:
        if txDataAvailable():
            txList = [ser]
        else:
            txList = []
        r, w, e = select.select([sys.stdin, ser], txList, [])

        # data coming from keyboard?
        if sys.stdin in r:
            if not handleKeyPress(sys.stdin.read(1)):
                break  # bail out

        # data coming from serial?
        if ser in r:
            data_str = ser.read(ser.inWaiting())
            for b in data_str:
                rxRawByte(b)

        # Any data to transmit over serial?
        if ser in w:
            flagAndByte = txGetByte()
            if flagAndByte[0]:
                ser.write(chr(flagAndByte[1]))

    return

if __name__ == '__main__':
    main()

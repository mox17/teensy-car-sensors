#!/usr/bin/env python
from __future__ import print_function
from collections import namedtuple
from datetime import datetime
import Queue

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

def getMilliSeconds():
    dt = datetime.now()
    return dt.second * 1000 + dt.microsecond / 1000

# helper function
def enum(**enums):
    return type('Enum', (), enums)

# Protocol data fields
header     = namedtuple("header", "dst src cmd rsv")
pingpong   = namedtuple("pingpong", "timestamp1 timestamp2")
distance   = namedtuple("distance", "sensor distance when")
rotation   = namedtuple("rotation", "speed direction when turn dist")
errorcount = namedtuple("errorcount", "count name")
#sonarwait  = namedtuple("pause")

def unpackPingPong(frm):
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

def unpackSonarStatus(frm):
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
    dist = distance(frm[0], get16(frm[2:])*10/57, get32(frm[4:]))
    return dist

def unpackRotation(frm):
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

def unpackErrorCount(frm):
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

# Defined addresses
ADDR_RPI    = 0x01
ADDR_TEENSY = 0x02

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

class Sensorhub:
    # Packet framing bytes
    FRAME_START_STOP  = 0x7e
    FRAME_DATA_ESCAPE = 0x7d
    FRAME_XOR         = 0x20
    
    # Possible receiver states
    rxs = enum(
        RS_BEGIN = 0,
        RS_DATA  = 1,
        )

    # TX state machine states
    txs = enum(
            TS_BEGIN  = 1,  # Nothing sent yet, deliver 0x7e
            TS_DATA   = 2,  # Sending normal data
            TS_ESCAPE = 3,  # Escape has been sent, txEscByte is next
            )
    
    def __init__(self):
        self.rxEscapeFlag = False
        self.rxState = Sensorhub.rxs.RS_BEGIN
        self.rxRawData = []
        self.rxChecksum = 0
        self.txQueue = Queue.Queue() # The TX queue have byte arrays 
        self.txCurrPacket = None
        self.txState = Sensorhub.txs.TS_BEGIN
        self.txEscByte = None
        self.ping = None
        self.pong = None
        self.wheel = None
        self.distance = None
        self.errorcount = None
        return

    def _rxDecodeFrame(self, frm):
        if len(frm)< 4:
            return
        hdr = header(frm[0], frm[1], frm[2], frm[3])
        frm = frm[4:]  # remove header and rxChecksum
        if hdr.cmd == Cmd.CMD_PING_QUERY :
            pp = unpackPingPong(frm)
            if self.ping != None:
                self.ping(pp)
        elif hdr.cmd == Cmd.CMD_PONG_RESP:
            pp = unpackPingPong(frm)
            if self.pong != None:
                self.pong(pp)
        elif hdr.cmd == Cmd.CMD_SET_SONAR_SEQ:
            pass
        elif hdr.cmd == Cmd.CMD_SONAR_STOP:
            pass
        elif hdr.cmd == Cmd.CMD_SONAR_START:
            pass
        elif hdr.cmd == Cmd.CMD_SONAR_STATUS:
            dist = unpackSonarStatus(frm)
            if self.distance != None:
                self.distance(dist)
        elif hdr.cmd == Cmd.CMD_WHEEL_STATUS:
            left  = unpackRotation(frm)
            right = unpackRotation(frm[16:])
            if self.wheel != None:
                self.wheel(left, right)
        elif hdr.cmd == Cmd.CMD_WHEEL_RESET:
            pass
        elif hdr.cmd == Cmd.CMD_ERROR_COUNT:
            err = unpackErrorCount(frm)
            if self.errorcount != None:
                self.errorcount(err)
        return

    def _rxChecksumCalc(self, b):
        self.rxChecksum += b
        self.rxChecksum += (self.rxChecksum >> 8);
        self.rxChecksum = self.rxChecksum & 0xff
        return

    def _rxChecksumNew(self):
        self.rxChecksum = 0
        return

    def _rxHandleFrame(self):
        if (len(self.rxRawData) > 0):
            if self.rxChecksum != 0xff:
                print("rxChecksum error:", rxChecksum, "\nerror data:", len(rxRawData), rxRawData)
            else:
                self._rxDecodeFrame(self.rxRawData[:-1])
        return

    def _rxNewFrame(self):
        self.rxRawData = []
        self._rxChecksumNew()
        return

    def rxRawByte(self, character):
        b = ord(character)
        if self.rxState == Sensorhub.rxs.RS_BEGIN:
            if b == Sensorhub.FRAME_START_STOP:
                self.rxState = Sensorhub.rxs.RS_DATA
                self._rxNewFrame()
        elif self.rxState == Sensorhub.rxs.RS_DATA:
            if b == Sensorhub.FRAME_START_STOP:
                self._rxHandleFrame()
                self._rxNewFrame()
                return
            elif b == Sensorhub.FRAME_DATA_ESCAPE:
                self.rxEscapeFlag = True
                return
            else:
                if self.rxEscapeFlag == True:
                    self.rxEscapeFlag = False
                    b = b ^ Sensorhub.FRAME_XOR
                self._rxChecksumCalc(b)
                self.rxRawData.append(b)
        else:
            assert("Internal error")
        return

    # High-level packet sending helpers
    def sendPing(self):
        buffer = buildHeader(ADDR_TEENSY, ADDR_RPI, Cmd.CMD_PING_QUERY)
        buffer = add32(buffer, getMilliSeconds()) # timestamp1
        buffer = add32(buffer, 0)           # timestamp2
        self.txQueue.put(buffer)
        return
    
    def sendSonarStop(self):
        buffer = buildHeader(ADDR_TEENSY, ADDR_RPI, Cmd.CMD_SONAR_STOP)
        self.txQueue.put(buffer)
        return

    def sendSonarStart(self):
        buffer = buildHeader(ADDR_TEENSY, ADDR_RPI, Cmd.CMD_SONAR_START)
        self.txQueue.put(buffer)
        return
    
    def sendWheelReset(self):
        buffer = buildHeader(ADDR_TEENSY, ADDR_RPI, Cmd.CMD_WHEEL_RESET)
        self.txQueue.put(buffer)
        return
    
    def sendGetCounters(self):
        buffer = buildHeader(ADDR_TEENSY, ADDR_RPI, Cmd.CMD_GET_COUNTERS)
        self.txQueue.put(buffer)
        return
    
    def sendSonarWait(self, pauseInMs):
        buffer = buildHeader(ADDR_TEENSY, ADDR_RPI, Cmd.CMD_SONAR_WAIT)
        buffer = add32(buffer, pauseInMs)
        self.txQueue.put(buffer)
        return
    
    def sendSonarSequence(self, sequence):
        buffer = buildHeader(ADDR_TEENSY, ADDR_RPI, Cmd.CMD_SET_SONAR_SEQ)
        l = len(sequence)
        if (l > 0 and l <= 24):
            buffer.append(l)
            buffer = buffer + sequence
            self.txQueue.put(buffer)
        return

    # TX side of packet handling
    def _txAppendChecksum(self, buf):
        sum = 0
        for b in buf:
            sum += b
            sum += sum >> 8
            sum = sum & 0xff
        buf.append(0xff-sum)
        return buf
    
    def _txGetNextPacket(self):
        if self.txCurrPacket == None or len(self.txCurrPacket) == 0:
            if not self.txQueue.empty():
                self.txCurrPacket = self._txAppendChecksum(self.txQueue.get())
                return True
        return False
    
    def _txGetDataByte(self):
        # return tuple (dataAvailable, byteToSend)
        if self.txCurrPacket == None:
            _txGetNextPacket()
        if self.txCurrPacket != None:
            if len(self.txCurrPacket) > 0:
                b = self.txCurrPacket[0]
                self.txCurrPacket = self.txCurrPacket[1:]
                return [True, b]
        return [False, None]
    
    def txDataAvailable(self):
        if self.txState != Sensorhub.txs.TS_BEGIN or (self.txCurrPacket != None and len(self.txCurrPacket) > 0):
            return True
        else:
            return self._txGetNextPacket()


    def txGetByte(self):
        if self.txState == Sensorhub.txs.TS_BEGIN:
            if self.txDataAvailable():
                self.txState = Sensorhub.txs.TS_DATA
                return [True, Sensorhub.FRAME_START_STOP]
            else:
                return [False, None]
    
        elif self.txState == Sensorhub.txs.TS_DATA:
            dataAvailable, byte = self._txGetDataByte()
            if dataAvailable == True:
                if (byte == Sensorhub.FRAME_START_STOP) or \
                   (byte == Sensorhub.FRAME_DATA_ESCAPE):
                    self.txEscByte = byte ^ Sensorhub.FRAME_XOR
                    byte = Sensorhub.FRAME_DATA_ESCAPE
                    self.txState = Sensorhub.txs.TS_ESCAPE
                return [True, byte]
            else:
                # This packet done - next one back-to-back?
                if self._txGetNextPacket():
                    self.txState = Sensorhub.txs.TS_DATA
                else:
                    self.txState = Sensorhub.txs.TS_BEGIN
                return [True, Sensorhub.FRAME_START_STOP]
    
        elif self.txState == Sensorhub.txs.TS_ESCAPE:
            self.txState = Sensorhub.txs.TS_DATA
            return [True, self.txEscByte]
    
        return [False, None]

    def setOutputHandlers(self, ping, pong, wheel, distance, errorcount):
        self.ping = ping
        self.pong = pong
        self.wheel = wheel
        self.distance = distance
        self.errorcount = errorcount
        return

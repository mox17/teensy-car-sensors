# sensorhub class
The Sensorhub class in `sensorhub.py` provides access to the Teensy sensors.

The interface requires byte RX and TX. The user can install handers that are called when specific messages are received from the Teensy, typically sensor readings.

This code snippet below illustrates how custom handlers are set up in the `sh.setOutputHandlers()` call.

~~~~python
def main():
    sh = sensorhub.Sensorhub()
    sh.setOutputHandlers(None, prettyPrintPong, prettyPrintWheels, prettyPrintDist, prettyPrintError)

    port = '/dev/ttyUSB0'
    baud = 115200
    ser = serial.Serial(port, baud, timeout=0)
    
    while True:
        if sh.txDataAvailable():
            txList = [ser]
        else:
            txList = []
        r, w, e = select.select([sys.stdin, ser], txList, [])
        
        if sys.stdin in r: # data coming from keyboard?
            if not handleKeyPress(sh, sys.stdin.read(1)):
                break  # exit program
        
        if ser in r:  # data coming from serial?
            data_str = ser.read(ser.inWaiting())
            for b in data_str:
                sh.rxRawByte(b)
        
        if ser in w:  # Any data to transmit over serial?
            flagAndByte = sh.txGetByte()
            if flagAndByte[0]:
                ser.write(chr(flagAndByte[1]))
    return
~~~~

Additionally there are a number of functions, that can send commands to the Teensy.
Below is a key handler that illustrates the use of the functions.

~~~~python
# User input handling
def handleKeyPress(sensor, key):
    key = key.upper()
    status = True
    if key == 'P':
        sensor.sendPing()
    elif key == 'N':
        sensor.sendSonarStop()
    elif key == 'S':
        sensor.sendSonarStart()
    elif key == 'R':
        sensor.sendWheelReset()
    elif key == 'C':
        sensor.sendGetCounters()
    elif key == '\x0C':
        clearScreen()
    elif key == 'X':
        sensor.sendSonarSequence([0,1,5])
    elif key == 'Y':
        sensor.sendSonarSequence([0,1,2,3,4,5])
    elif key == 'Q':
        status = False
    return status
~~~~


'''
115200 baud
frame:
    8 data bits
    0 parity bits
    1 stop bit
    (8N1)
ASCII data, 21 fields, separated by commas
'''

import threading
import time
import serial
import serial.tools.list_ports

PHASES = ['@', 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J']  # NOTE: state 'J' means that the simulation is over (won't happend IRL)
PHASE_DESC = { '@':"No flight state reached", 'A':"Liftoff", 'B':"Meco", 'C':"Separation", 'D':"Coast_Start", 'E':"Apogee", 'F':"Coast_End", 'G':"Under_Chutes", 'H':"Landing", 'I':"Safing", 'J':"Simulation Finished"}
TICKS_PER_PHASE = { '@':10, 'A':10, 'B':10, 'C':10, 'D':10, 'E':10, 'F':10, 'G':10, 'H':10, 'I':10, 'J':10}
DATA_FREQUENCY = 0.1  # time between data bursts (in s)

def main():
    devices = serial.tools.list_ports.comports()
    device = None
    if len(devices) > 0:
        i = 0
        while i < len(devices) and "ttyACM0" not in devices[i].description:
            print(type(devices[i]))
            i += 1
        if i == len(devices):
            print("No Arduino connected")
            return

        device = devices[i]

        try:
            ser = serial.Serial(port=device.device, baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                                timeout=0)  # setting this timeout allows for write to be non-blocking
        except serial.SerialException:
            print("error connecting to " + device.description + ", perhaps it is already in use?")
            return
        print("transmitting to " + device.description)

        # tick
        startTime = time.time()

        for phase in PHASES:
            print("[" + phase + "] " + PHASE_DESC[phase])
            for i in range(TICKS_PER_PHASE[phase]):
                # TODO: replace these with numbers that change
                dataString = phase + "," + "{:.2f}".format(time.time()) + ",9697.791016,-216.117355,0.239193,-0.373560,32.779297,0.000000,0.000000,-0.272123,-0.004113,0.000209,-0.001000,0.000000,0.000000,0,0,0,1,0,0"
                ser.write(dataString.encode("ASCII"))
                # print(dataString.encode("ASCII"))
                time.sleep(DATA_FREQUENCY)
        
        # tock
        endTime = time.time()
        
        print(str(endTime - startTime) + " seconds elapsed")
        ser.close()

    else:
        print("no serial devices found")

main()

from collections.abc import MutableMapping
import sys
import serial
if sys.version_info.major == 3 and sys.version_info.minor >= 10:
 import collections
 setattr(collections,"MutableMapping",collections.abc.MutableMapping)
from dronekit import connect , VehicleMode , LocationGlobalRelative , APIException
import time
import socket
import exception
import math
import argparse

def connectMyCopter ():
    parser  =  argparse.ArgumentParser(description="commands")
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string  = args.connect
    baud_rate  =  57600

    vehicle = connect(connection_string , baud=baud_rate , wait_ready=True)
    return vehicle

def arm():
    while vehicle.is_armable==False:
        print("Waiting for vehicle to become armable ..")
        time.sleep(1)

    print("Vehicle is now armed")
    print("OMG Props are spinning look out")

    print("ajjubhai")
    vehicle.armed = True

    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=.5)
    i = 0
    while True:
        i += 1
        if i > 1:
           break
        ser.write(b'1 1827 \r\n')  # Note the b before the string to send bytes
        ser.write(b'2 928 \r\n')
        incoming = ser.readline().strip().decode('utf-8')  # Decode bytes to string
        print('Received Data: ' + incoming)

    time.sleep(10)
    vehicle.armed = False
    return None

vehicle = connectMyCopter()
arm()
print("End of script")

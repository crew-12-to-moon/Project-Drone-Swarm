from collections.abc import MutableMapping
import sys
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
    print('k')
    return vehicle

def arm():
    while vehicle.is_armable==False:
        print("Waiting for vehicle to become armable ..")
        time.sleep(0.5)
    print("Vehicle is now armed")
    print("OMG Props are spinning look out")

    print("ajjubhai")
    vehicle.armed = True
    #return None
    time.sleep(10)
    vehicle.armed = False
    return None


import serial
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=.5)
i = 0
while True:
    incoming = ser.readline().strip().decode('utf-8')  # Decode bytes to string

    if incoming == "":
        print('Received data: ' + incoming)

    elif incoming[0] == "2":
        i += 1
        if i > 1:
                break

        print('Received data: ' + incoming)
        vehicle = connectMyCopter()
        print('l')
        arm()
        print("End of script")

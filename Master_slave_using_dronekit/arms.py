from collections.abc import MutableMapping
import sys
if sys.version_info.major == 3 and sys.version_info.minor >= 10:
 import collections
 setattr(collections,"MutableMapping",collections.abc.MutableMapping)
from dronekit import connect , VehicleMode , LocationGlobalRelative , APIExcepti                                                                             on
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
    return None

vehicle = connectMyCopter()
arm()
print("End of script")

from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import socket
import exceptions


"""connection to vehicle"""
def connection(addr, baud):
    try:
        # Connect to the Vehicle
        print
        'Connecting to vehicle on: %s' % addr
        vehicle = connect(addr, baud, wait_ready=True)
        return vehicle
    # Bad TCP connection
    except socket.error:
        print
        'No server exists!'

    # Bad TTY connection
    except exceptions.OSError as e:
        print
        'No serial exists!'

    # API Error
    except dronekit.APIException:
        print
        'Timeout!'

    # Other error
    except:
        print
        'Some other error!'

"""derive the vehicle at a speed arbitrary of 0.5"""
def derive(vehicle):
    print('Check mode')

    if vehicle.mode != "OFFBOARD":
        print('Vehicule not in offboard, stay on stabilze')
        vehicle.mode = VehicleMode("STABILIZE")

    print('Begin to derive gently')

    # Set groundspeed using attribute
    vehicle.groundspeed = 0.5  # m/s


def stop(vehicle):
    print('stopping the vehicle')
    vehicle.groundspeed = 0
    # Hover for 10 seconds
    time.sleep(5)

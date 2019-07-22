from dronekit import connect, Command, LocationGlobal, VehicleMode
from pymavlink import mavutil
import treeRecognition
import time, sys, argparse, math

GO_SWITCH = False

try:
    # Connect to the Vehicle
    print("Connecting to vehicle on: ttyAMA0 ")
    # Connect to the Vehicle (in this case a UDP endpoint)
    vehicle = connect('/dev/ttyAMA0', wait_ready=True, baud=921600)
    # Bad TCP connection
except socket.error:
    print("No vehicle exists!")

    # Bad TTY connection
except exceptions.OSError as e:
    print("No serial exists!")

    # API Error
except dronekit.APIException:
    print("Timeout!")

    # Other error
except:
    print("Some other error!")

# Display basic vehicle state
print(" Type: %s" % vehicle._vehicle_type)
print(" System status: %s" % vehicle.system_status.state)
print(" GPS: %s" % vehicle.gps_0)


@vehicle.on_message('RC_CHANNELS')
def listener(vehicle, name, message):
    global GO_SWITCH
    if message.chan12_raw > 1500:
        GO_SWITCH = True
    else:
        GO_SWITCH = False


print("wait for switch")
while not GO_SWITCH:
    print(".")
    time.sleep(1)

print("LOITER switch detected")

vehicle.mode = VehicleMode("LOITER")
print("Loiter mode success")
time.sleep(20)
print("quit....")
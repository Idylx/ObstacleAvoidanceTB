from dronekit import connect, Command, LocationGlobal, VehicleMode
from pymavlink import mavutil
import treeRecognition
import time, sys, argparse, math


MAV_MODE_AUTO = 4
MAV_MODE_STABILIZE = 16
MAX_LENGTH = 30
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
print
" Type: %s" % vehicle._vehicle_type
print
" Armed: %s" % vehicle.armed
print
" System status: %s" % vehicle.system_status.state
print
" GPS: %s" % vehicle.gps_0



def PX4setMode(mavMode):
    vehicle._master.mav.command_long_send(vehicle._master.target_system, vehicle._master.target_component,
                                               mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
                                               mavMode,
                                               0, 0, 0, 0, 0, 0)



def get_location_offset_meters(original_location, dNorth, dEast, alt):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the
    specified `original_location`. The returned Location adds the entered `alt` value to the altitude of the `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt+alt)



@vehicle.on_message('RC_CHANNELS')
def listener(vehicle, name, message):
        global GO_SWITCH
        if message.chan12_raw > 1500:
                GO_SWITCH = True
                PX4setMode(MAV_MODE_AUTO)
        else:
                GO_SWITCH = False
                PX4setMode(MAV_MODE_STABILIZE)


print("wait for switch")
while not GO_SWITCH:
       print(".")
       time.sleep(1)


print("mission switch detected")

# Change to AUTO mode
PX4setMode(MAV_MODE_AUTO)
time.sleep(1)


# Load commands
cmds = vehicle.commands
cmds.clear()

relativePosition = vehicle.location.global_relative_frame

#for every meter to west
for endPoint in range(0, MAX_LENGTH):
    wp = get_location_offset_meters(relativePosition, 0, endPoint, 0)
    cmd = Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0,
                  0, 0, 0, wp.lat, wp.lon, wp.alt)
    print(cmd)
    cmds.add(cmd)


# takeoff to 10 meters
#wp = get_location_offset_meters(home, 0, 0, 10);
#cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
#cmds.add(cmd)

# move 10 meters north
# wp = get_location_offset_meters(wp, 10, 0, 0);
# cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
# cmds.add(cmd)

# move 10 meters east
# wp = get_location_offset_meters(wp, 0, 10, 0);
# cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
# cmds.add(cmd)

# move 10 meters south
# wp = get_location_offset_meters(wp, -10, 0, 0);
# cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
# cmds.add(cmd)

# move 10 meters west
# wp = get_location_offset_meters(relativePosiition, 0, -10, 0);
# cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
# cmds.add(cmd)


# land
# wp = get_location_offset_meters(home, 0, 0, 10);
# cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
# cmds.add(cmd)

# Upload mission
cmds.upload()
time.sleep(2)

cpt = 0

# monitor mission execution
nextwaypoint = vehicle.commands.next
while nextwaypoint < len(vehicle.commands):
    print(nextwaypoint)
    if cpt > 6:
        # check if need to stop tree
        # if treeRecognition.activateRecognitionOnePicture('yolo-custom', 0.1, 0.2):
        print("Stopping vehicle")
        PX4setMode(MAV_MODE_STABILIZE)
        cmds.clear
        break


    if vehicle.commands.next > nextwaypoint:
        cpt += 1
        display_seq = vehicle.commands.next+1
        print
        "Moving to waypoint %s" % display_seq
        nextwaypoint = vehicle.commands.next

    time.sleep(1)

cmds.clear



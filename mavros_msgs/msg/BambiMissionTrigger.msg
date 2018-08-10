# Current autopilot state
#
# Known modes listed here:
# http://wiki.ros.org/mavros/CustomModes
#
# For system_status values
# see http://mavlink.org/messages/common MAV_STATE
#


std_msgs/Header header
bool startStop   #true:mission start (takeoff), false: mission abort

float64 latitude  # Orthophoto shoot position latitude WGS84
float64 longitude # Orthophoto shoot position longitude WGS84
float32 altitude	# Takeoff/RTL altitude in meters, relative to ground





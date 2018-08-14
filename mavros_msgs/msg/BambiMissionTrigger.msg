# Current autopilot state
#
# Known modes listed here:
# http://wiki.ros.org/mavros/CustomModes
#
# For system_status values
# see http://mavlink.org/messages/common MAV_STATE
#


std_msgs/Header header
bool startStop                  #true:mission start (takeoff), false: mission abort
float32 altitudeTakeoff         # Takeoff/RTL altitude in meters, relative to home position
float32 latitudeOrthophoto      # Orthophoto shoot position latitude (WGS84)
float32 longitudeOrthophoto     # Orthophoto shoot position longitude (WGS84)
float32 altitudeOrthophoto      # Orthophoto shhoot position altitude (meters AMSL)
float32 altitudeScanning        # Scanning altitude (meters over the ground)
float32 SensorFootprintMinDim   #Sensor Footprint minimum Dimension (in meters)




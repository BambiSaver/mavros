# Current autopilot state
#
# Known modes listed here:
# http://wiki.ros.org/mavros/CustomModes
#
# For system_status values
# see http://mavlink.org/messages/common MAV_STATE
#

std_msgs/Header header
bool startStop
geographic_msgs/GeoPoint missionBasePoint
uint8 startAltitutdeOverGround
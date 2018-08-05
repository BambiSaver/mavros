/**
 * @brief LocalPosition plugin
 * @file local_position.cpp
 * @author Vladimir Ermakov <vooon341@gmail.com>
 * @author Glenn Gregory
 * @author Eddy Scott <scott.edward@aurora.aero>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014,2016 Vladimir Ermakov.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros_msgs/BambiMissionTrigger.h>



#include <eigen_conversions/eigen_msg.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>

#include <nav_msgs/Odometry.h>

namespace mavros {
namespace std_plugins {
/**
 * @brief Local position plugin.
 * Publish local position to TF, PositionStamped, TwistStamped
 * and Odometry
 */
class BambiPlugin : public plugin::PluginBase {
public:
  BambiPlugin() : PluginBase(),
    bambi_nh("~bambi")
  { }

  void initialize(UAS &uas_)
  {
    PluginBase::initialize(uas_);
    ROS_INFO("!!!!!!!!!!!!BAMBI INITIALIZATION!!!!!!!!");
    mission_trigger = bambi_nh.advertise<mavros_msgs::BambiMissionTrigger>("missionTrigger", 2, false);
  }

  Subscriptions get_subscriptions() {
    return {
           make_handler(&BambiPlugin::handle_mission_trigger)
    };
  }

private:
  ros::NodeHandle bambi_nh;

  ros::Publisher mission_trigger;

  void handle_mission_trigger(const mavlink::mavlink_message_t *msg, mavlink::common::msg::COMMAND_LONG &command)
  {
    ROS_INFO("BAMBI GOT A MESSAGE!!!!!!!!");
    ROS_INFO("TARGET (%d, %d) COMMAND_ID: %d", command.target_system, command.target_component, command.command);

    // only accept 0,1 and 240
    // TODO: change to MYID somehow
    if (command.target_system > 1 || command.target_component != 240) {
      return;
    }

    using mavlink::common::MAV_CMD;

    switch(command.command) {
    // should be MAV_CMD::BAMBI....
    case 2720:
      boost::shared_ptr<mavros_msgs::BambiMissionTrigger> mission_trigger_msg = boost::make_shared<mavros_msgs::BambiMissionTrigger>();

      mission_trigger_msg->startStop = true;
      //mission_trigger->missionBasePoint 3xdouble
      mission_trigger_msg->startAltitutdeOverGround = 30;

      mission_trigger.publish(mission_trigger_msg);
      break;
    }
  }
};
} // namespace std_plugins
} // namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::BambiPlugin, mavros::plugin::PluginBase)

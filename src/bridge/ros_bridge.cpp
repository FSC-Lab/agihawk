#include "agihawk/bridge/ros_bridge.hpp"

#include "agihawk/ros_eigen.hpp"
#include "agihawk/time.hpp"
#include "agiros_msgs/Command.h"
#include "std_msgs/Bool.h"

namespace agi {

RosBridge::RosBridge(const ros::NodeHandle& nh, const ros::NodeHandle& pnh,
                     const TimeFunction time_function,
                     const std::string& command_topic,
                     const std::string& armed_topic, const Scalar timeout,
                     const int n_max_timeouts)
  : BridgeBase("ROS Bridge", time_function, timeout, n_max_timeouts),
    nh_(nh),
    pnh_(pnh) {
  command_pub_ = pnh_.advertise<agiros_msgs::Command>(command_topic, 10);
  armed_pub_ = pnh_.advertise<std_msgs::Bool>(armed_topic, 10);
  voltage_watchdog_.disable();  // not a real bridge, disable voltage watchdog
}

bool RosBridge::sendCommand(const Command& command, const bool active) {
  agiros_msgs::Command ros_command = toRosCommand(command);
  std_msgs::Bool armed_msg;
  armed_msg.data = active;
  command_pub_.publish(ros_command);
  armed_pub_.publish(armed_msg);
  return true;
}

}  // namespace agi

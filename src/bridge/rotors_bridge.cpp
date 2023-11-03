#include "agihawk/bridge/rotors_bridge.hpp"

#include "agilib/math/math.hpp"
#include "mav_msgs/Actuators.h"

namespace agi {

RotorsBridge::RotorsBridge(const ros::NodeHandle& nh,
                           const ros::NodeHandle& pnh, const Quadrotor& quad,
                           const TimeFunction time_function)
  : BridgeBase("RotorSBridge", time_function), nh_(nh), pnh_(pnh), quad_(quad) {
  rotor_omega_pub_ =
    nh_.advertise<mav_msgs::Actuators>("command/motor_speed", 1);
}

bool RotorsBridge::sendCommand(const Command& command, const bool active) {
  if (!command.isSingleRotorThrusts()) {
    ROS_ERROR("RotorS bridge only allows SRT commands!");
    return false;
  }

  const Vector<4> motor_omegas =
    quad_.clampMotorOmega(quad_.motorThrustToOmega(command.thrusts));

  mav_msgs::Actuators msg;
  msg.header.stamp = ros::Time::now();  //(command.t);

  for (int i = 0; i < 4; i++) {
    msg.angular_velocities.push_back(motor_omegas[i]);
  }

  rotor_omega_pub_.publish(msg);
  return true;
}


}  // namespace agi

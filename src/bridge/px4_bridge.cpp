#include "agihawk/bridge/px4_bridge.hpp"

#include "agilib/math/math.hpp"

namespace agi {

PixhawkBridge::PixhawkBridge(const ros::NodeHandle& nh,
                             const ros::NodeHandle& pnh, const Quadrotor& quad,
                             const TimeFunction time_function)
  : BridgeBase("PixhawkBridge", time_function),
    nh_(nh),
    pnh_(pnh),
    quad_(quad) {
  state_sub_ = nh_.subscribe("mavros/state", 1, &PixhawkBridge::stateCallback, this);

  command_pub_ = nh_.advertise<mavros_msgs::AttitudeTarget>(
    "mavros/setpoint_raw/attitude", 1);

  status_pub_ = nh_.advertise<mavros_msgs::CompanionProcessStatus>(
    "mavros/companion_process/status", 1);

  mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  
  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

  pnh_.getParam("thrust_ratio", thrust_ratio_);
}

void PixhawkBridge::stateCallback(const mavros_msgs::StateConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mutex_);
  armed_ = msg->armed;
  offboard_ = (msg->mode == "OFFBOARD") ? true : false;
}

bool PixhawkBridge::sendCommand(const Command& command, const bool active) {
  std::lock_guard<std::mutex> lock(mutex_);

  if (!command.isRatesThrust()) {
    ROS_ERROR("Pixhawk bridge only allows bodyrate commands!");
    return false;
  }

  mavros_msgs::CompanionProcessStatus status_msg;
  status_msg.header.stamp = ros::Time(time_function_());
  status_msg.component = 196;  // MAV_COMPONENT_ID_AVOIDANCE
  if (!active) {
    status_msg.state = PilotState::STANDBY;
    status_pub_.publish(status_msg);

    if (armed_) {
      mavros_msgs::CommandBool arm_cmd;
      arm_cmd.request.value = false;
      arming_client_.call(arm_cmd);
    }

    return true;
  } else {
    if (!offboard_) {
      mavros_msgs::SetMode offb_set_mode;
      offb_set_mode.request.custom_mode = "OFFBOARD";
      offb_set_mode.request.base_mode = 0;

      mode_client_.call(offb_set_mode);
    }

    if (offboard_ && !armed_) {
      mavros_msgs::CommandBool arm_cmd;
      arm_cmd.request.value = true;
      arming_client_.call(arm_cmd);
    }
  }

  status_msg.state = PilotState::ACTIVE;
  status_pub_.publish(status_msg);

  mavros_msgs::AttitudeTarget omg_msg;
  omg_msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;
  omg_msg.header.stamp = ros::Time(time_function_());  //(command.t);
  omg_msg.orientation.w = 1.0;
  omg_msg.orientation.x = 0.0;
  omg_msg.orientation.y = 0.0;
  omg_msg.orientation.z = 0.0;
  omg_msg.body_rate.x = command.omega(0);
  omg_msg.body_rate.y = command.omega(1);
  omg_msg.body_rate.z = command.omega(2);

  Scalar scaled_thrust =
    thrust_ratio_ * command.collective_thrust / quad_.collective_thrust_max();
  omg_msg.thrust = std::clamp(scaled_thrust, MIN_THROTTLE, MAX_THROTTLE);

  command_pub_.publish(omg_msg);

  return true;
}


}  // namespace agi

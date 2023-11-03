#pragma once

#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "agilib/bridge/bridge_base.hpp"
#include "agilib/types/quadrotor.hpp"


namespace agi {

class PixhawkBridge : public BridgeBase {
 public:
  static constexpr Scalar MAX_THROTTLE = 1.0;
  static constexpr Scalar MIN_THROTTLE = 0.05;

  enum PilotState : unsigned int {
    UNINIT = 0,
    BOOT = 1,
    CALIBRATING = 2,
    STANDBY = 3,
    ACTIVE = 4,
    CRITICAL = 5,
    EMERGENCY = 6,
    POWEROFF = 7,
    FLIGHT_TERMINATION = 8,
  };

  PixhawkBridge(const ros::NodeHandle& nh, const ros::NodeHandle& pnh,
                const Quadrotor& quad, const TimeFunction time_function);

  void stateCallback(const mavros_msgs::StateConstPtr& msg);

 protected:
  virtual bool sendCommand(const Command& command, const bool active) override;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber state_sub_;
  ros::Publisher command_pub_;
  ros::Publisher status_pub_;
  ros::ServiceClient arming_client_;
  ros::ServiceClient mode_client_;
  const Quadrotor quad_;
  Scalar thrust_ratio_{1.0};
  bool armed_{false};
  bool offboard_{false};
  std::mutex mutex_;
};

}  // namespace agi

#pragma once

#include <ros/ros.h>

#include <string>

#include "agilib/bridge/bridge_base.hpp"
#include "agilib/types/quadrotor.hpp"

namespace agi {

class RotorsBridge : public BridgeBase {
 public:
  RotorsBridge(const ros::NodeHandle& nh_, const ros::NodeHandle& pnh,
               const Quadrotor& quad, const TimeFunction time_function);

 protected:
  virtual bool sendCommand(const Command& command, const bool active) override;

  ros::NodeHandle nh_, pnh_;
  ros::Publisher rotor_omega_pub_;

  const Quadrotor quad_;

  Scalar time_last_valid_nonzero_command_{0.0};
};

}  // namespace agi

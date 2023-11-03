#pragma once

#include <ros/ros.h>

#include "agilib/bridge/bridge_base.hpp"
#include "agilib/utils/logger.hpp"
#include "agilib/utils/timer.hpp"

namespace agi {

class RosBridge : public BridgeBase {
 public:
  explicit RosBridge(const ros::NodeHandle& nh_, const ros::NodeHandle& pnh,
                     const TimeFunction time_function,
                     const std::string& command_topic = "command",
                     const std::string& armed_topic = "armed",
                     const Scalar timeout = 0.1, const int n_max_timeouts = 10);

 protected:
  virtual bool sendCommand(const Command& command, const bool active);

  ros::NodeHandle nh_, pnh_;
  ros::Publisher command_pub_;
  ros::Publisher armed_pub_;
};

}  // namespace agi

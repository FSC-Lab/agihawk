#pragma once

#include <ros/ros.h>

#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

#include "agilib/bridge/ctrl/ctrl_bridge.hpp"

namespace agi {

class CtrlFeedbackPublisher {
 public:
  CtrlFeedbackPublisher(const ros::NodeHandle& nh, const ros::NodeHandle& pnh);
  ~CtrlFeedbackPublisher();

  void addFeedback(const Feedback& feedback);

 private:
  void processFeedbackThread();
  void processFeedback(const Feedback& feedback);
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  // for CTRL bridge publish the feedback
  ros::Publisher battery_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher control_mode_pub_;
  ros::Publisher rotor_feedback_pub_;

  unsigned int seq{0};
  Scalar last_feedback_time{0};

  bool shutdown_{false};
  std::vector<Feedback> feedbacks_;
  std::mutex feedback_queue_mtx_;
  std::thread process_feedback_thread_;
  std::mutex process_feedback_mtx_;
  std::condition_variable process_feedback_cv_;
};
}  // namespace agi

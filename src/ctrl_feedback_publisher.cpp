#include "agihawk/ctrl_feedback_publisher.hpp"

#include <mutex>

#include "agihawk/ros_eigen.hpp"
#include "agiros_msgs/RotorFeedbackStamped.h"
#include "agiros_msgs/UInt8Stamped.h"
#include "sensor_msgs/BatteryState.h"
#include "sensor_msgs/Imu.h"

namespace agi {


CtrlFeedbackPublisher::CtrlFeedbackPublisher(const ros::NodeHandle& nh,
                                             const ros::NodeHandle& pnh)
  : nh_(nh), pnh_(pnh) {
  battery_pub_ = nh_.advertise<sensor_msgs::BatteryState>("battery", 1);
  imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 1);
  control_mode_pub_ =
    nh_.advertise<agiros_msgs::UInt8Stamped>("control_mode", 1);
  rotor_feedback_pub_ =
    nh_.advertise<agiros_msgs::RotorFeedbackStamped>("rotor_feedback", 1);

  process_feedback_thread_ =
    std::thread(&CtrlFeedbackPublisher::processFeedbackThread, this);
}

CtrlFeedbackPublisher::~CtrlFeedbackPublisher() {
  if (process_feedback_thread_.joinable()) {
    shutdown_ = true;
    process_feedback_thread_.join();
  }
}

void CtrlFeedbackPublisher::addFeedback(const Feedback& feedback) {
  {
    std::lock_guard<std::mutex> guard{feedback_queue_mtx_};
    feedbacks_.push_back(feedback);
  }
  process_feedback_cv_.notify_all();
}

void CtrlFeedbackPublisher::processFeedbackThread() {
  while (!shutdown_ && ros::ok()) {
    std::unique_lock<std::mutex> lk(process_feedback_mtx_);
    process_feedback_cv_.wait_for(lk, std::chrono::seconds(1));
    if (shutdown_ || !ros::ok()) break;
    std::vector<Feedback> feedbacks;
    {
      std::lock_guard<std::mutex> guard{feedback_queue_mtx_};
      if (feedbacks_.empty()) continue;
      feedbacks = feedbacks_;
      feedbacks_.clear();
    }
    for (const Feedback& feedback : feedbacks) {
      if (feedback.valid()) processFeedback(feedback);
    }
  }
}

void CtrlFeedbackPublisher::processFeedback(const Feedback& feedback) {
  // first check if the feedback time is different from the old one

  if (last_feedback_time == feedback.t) return;

  last_feedback_time = feedback.t;

  // battery state publishing
  sensor_msgs::BatteryState battery_msg;
  bool has_battey_info = false;
  if (feedback.isBatteryVoltageValid()) {
    battery_msg.voltage = feedback.voltage;
    has_battey_info = true;
  }
  if (feedback.isBatteryCurrentValid()) {
    battery_msg.current = feedback.current;
    has_battey_info = true;
  }
  if (has_battey_info) {
    battery_msg.header.stamp = ros::Time(feedback.t);
    battery_msg.header.seq = seq;
    battery_msg.header.frame_id = "quad";
    battery_pub_.publish(battery_msg);
  }

  // imu publishing
  if (feedback.isImuValid()) {
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = ros::Time(feedback.t);
    imu_msg.header.seq = seq;
    imu_msg.header.frame_id = "quad";
    imu_msg.orientation = toRosQuaternion(feedback.attitude);
    imu_msg.angular_velocity = toRosVector(feedback.imu.omega);
    imu_msg.linear_acceleration = toRosVector(feedback.imu.acc);
    imu_pub_.publish(imu_msg);
  }

  // control mode publishing
  agiros_msgs::UInt8Stamped control_mode_msg;
  control_mode_msg.header.stamp = ros::Time(feedback.t);
  control_mode_msg.header.seq = seq;
  control_mode_msg.header.frame_id = "quad";
  control_mode_msg.data = (uint8_t)feedback.control_mode;
  control_mode_pub_.publish(control_mode_msg);

  // rotor feedback publishing
  agiros_msgs::RotorFeedbackStamped rotor_feedback_msg;
  rotor_feedback_msg.data.resize(4);
  rotor_feedback_msg.type = (uint8_t)feedback.rotor_feedback_type;
  bool rotor_feedback_valid = false;
  switch (feedback.rotor_feedback_type) {
    case Feedback::ROTORFEEDBACKTYPE::THROTTLE:
      for (int i = 0; i < 4; ++i)
        rotor_feedback_msg.data[i] = feedback.rotor_value[i];
      rotor_feedback_valid = true;
      break;
    case Feedback::ROTORFEEDBACKTYPE::SPEED:
      for (int i = 0; i < 4; ++i)
        rotor_feedback_msg.data[i] = feedback.rotor_speed_rads[i];
      rotor_feedback_valid = true;
      break;
    case Feedback::ROTORFEEDBACKTYPE::THRUST:
      for (int i = 0; i < 4; ++i)
        rotor_feedback_msg.data[i] = feedback.rotor_thrust_newton[i];
      rotor_feedback_valid = true;
      break;
    default:
      ROS_WARN_STREAM_THROTTLE(5.0, "no rotor speed feedback");
      break;
  }
  if (rotor_feedback_valid) {
    rotor_feedback_msg.header.stamp = ros::Time(feedback.t);
    rotor_feedback_msg.header.seq = seq;
    rotor_feedback_msg.header.frame_id = "quad";
    rotor_feedback_pub_.publish(rotor_feedback_msg);
  }
}

}  // namespace agi

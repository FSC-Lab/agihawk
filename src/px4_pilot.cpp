#include "agihawk/px4_pilot.hpp"

#include <chrono>
#include <functional>

#include "agilib/reference/velocity_reference.hpp"
#include "agilib/types/pose.hpp"
#include "agilib/utils/filesystem.hpp"
#include "agilib/utils/logger.hpp"
#include "agihawk/bridge/ros_bridge.hpp"
#include "agihawk/bridge/rotors_bridge.hpp"
#include "agihawk/ctrl_feedback_publisher.hpp"
#include "agihawk/time.hpp"
#include "agiros_msgs/DebugMsg.h"
#include "agiros_msgs/QuadState.h"
#include "agiros_msgs/Telemetry.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Float32.h"

#include "agihawk/bridge/px4_bridge.hpp"


namespace agi {

static inline PilotParams loadParams(const ros::NodeHandle& nh) {
  std::string pilot_config;
  const bool got_config = nh.getParam("pilot_config", pilot_config);

  std::string agi_param_dir;
  const bool got_directory = nh.getParam("agi_param_dir", agi_param_dir);

  std::string ros_param_dir;
  nh.getParam("ros_param_dir", ros_param_dir);

  ROS_WARN_STREAM("Pilot Config:        " << pilot_config);
  ROS_WARN_STREAM("Agi Param Directory: " << agi_param_dir);
  ROS_WARN_STREAM("ROS Param Directory: " << ros_param_dir);

  if (!got_config) ROS_FATAL("No parameter directory given!");
  if (!got_directory) ROS_FATAL("No Pilot config file given!");
  if (!got_config || !got_directory) ros::shutdown();
  ROS_INFO("Loading Pilot Params from %s in %s", pilot_config.c_str(),
           ros_param_dir.c_str());

  // path to px4_sim_pilot.yaml: ros_param_dir/pilot_config
  // path to quadrotor.yaml: ros_param_dir/quads/
  return PilotParams(fs::path(ros_param_dir) / fs::path(pilot_config),
                     agi_param_dir, fs::path(ros_param_dir) / "quads" / "");
}

Px4Pilot::Px4Pilot(const ros::NodeHandle& nh, const ros::NodeHandle& pnh)
  : nh_(nh),
    pnh_(pnh),
    params_(loadParams(pnh_)),
    pilot_(params_, RosTime),
    reference_visualizer_("references", nh_, pnh_, params_.traj_viz_dt_,
                          params_.traj_viz_sphere_size_),
    active_reference_visualizer_("active_reference", nh_, pnh_),
    outer_setpoints_visualizer_("outer_setpoints", nh_, pnh_),
    inner_setpoints_visualizer_("inner_setpoints", nh_, pnh_) {
  ROS_INFO_STREAM("Loaded pipeline:\n" << params_.pipeline_cfg_);
  ROS_INFO("Loaded Guard Type:    %s", params_.guard_cfg_.type.c_str());
  if (!params_.guard_cfg_.file.empty())
    ROS_INFO("with parameter file:    %s", params_.guard_cfg_.file.c_str());

  // Pose & Odometry subscribers
  pose_estimate_sub_ =
    pnh_.subscribe("pose_estimate", 1, &Px4Pilot::poseEstimateCallback, this,
                   ros::TransportHints().tcpNoDelay());
  odometry_estimate_sub_ =
    pnh_.subscribe("odometry_estimate", 1, &Px4Pilot::odometryEstimateCallback,
                   this, ros::TransportHints().tcpNoDelay());
  state_estimate_sub_ =
    pnh_.subscribe("state_estimate", 1, &Px4Pilot::stateEstimateCallback, this,
                   ros::TransportHints().tcpNoDelay());
  imu_sub_ = pnh_.subscribe("imu_in", 1, &Px4Pilot::imuCallback, this,
                            ros::TransportHints().tcpNoDelay());
  motor_speed_sub_ =
    pnh_.subscribe("motor_speed", 1, &Px4Pilot::motorSpeedCallback, this,
                   ros::TransportHints().tcpNoDelay());
  guard_pose_estimate_sub_ = pnh_.subscribe(
    "guard_pose_estimate", 1, &Px4Pilot::guardPoseEstimateCallback, this,
    ros::TransportHints().tcpNoDelay());
  guard_odometry_estimate_sub_ = pnh_.subscribe(
    "guard_odometry_estimate", 1, &Px4Pilot::guardOdometryEstimateCallback,
    this, ros::TransportHints().tcpNoDelay());
  guard_state_estimate_sub_ = pnh_.subscribe(
    "guard_state_estimate", 1, &Px4Pilot::guardStateEstimateCallback, this,
    ros::TransportHints().tcpNoDelay());
  // Logic subscribers
  start_sub_ = pnh_.subscribe("start", 1, &Px4Pilot::startCallback, this);
  force_hover_sub_ =
    pnh_.subscribe("force_hover", 1, &Px4Pilot::forceHoverCallback, this);
  go_to_pose_sub_ =
    pnh_.subscribe("go_to_pose", 1, &Px4Pilot::goToPoseCallback, this);
  velocity_sub_ =
    pnh_.subscribe("velocity_command", 1, &Px4Pilot::velocityCallback, this);
  feedthrough_command_sub_ = pnh_.subscribe(
    "feedthrough_command", 1, &Px4Pilot::feedthroughCommandCallback, this,
    ros::TransportHints().tcpNoDelay());

  land_sub_ = pnh_.subscribe("land", 1, &Px4Pilot::landCallback, this);
  off_sub_ = pnh_.subscribe("off", 1, &Px4Pilot::offCallback, this);
  enable_sub_ = pnh_.subscribe("enable", 1, &Px4Pilot::enableCallback, this);
  guard_sub_ =
    pnh_.subscribe("trigger_guard", 1, &Px4Pilot::triggerGuardCallback, this);

  // Trajectory subscribers
  trajectory_sub_ =
    pnh_.subscribe("trajectory", 1, &Px4Pilot::trajectoryCallback, this);

  // Telemetry subscribers
  voltage_sub_ =
    nh_.subscribe("mavros/battery", 1, &Px4Pilot::voltageCallback, this);

  // Publishers
  state_pub_ = pnh_.advertise<agiros_msgs::QuadState>("state", 1);
  state_odometry_pub_ = pnh_.advertise<nav_msgs::Odometry>("odometry", 1);
  telemetry_pub_ = pnh_.advertise<agiros_msgs::Telemetry>("telemetry", 1);
  cmd_pub_ = pnh_.advertise<agiros_msgs::Command>("mpc_command", 1);

  if (params_.pipeline_cfg_.bridge_cfg.type == "RotorS") {
    pilot_.registerExternalBridge(
      std::make_shared<RotorsBridge>(nh_, pnh_, params_.quad_, RosTime));
  } else if (params_.pipeline_cfg_.bridge_cfg.type == "ROS") {
    pilot_.registerExternalBridge(
      std::make_shared<RosBridge>(nh_, pnh_, RosTime));
  } else if (params_.pipeline_cfg_.bridge_cfg.type == "PX4") {
    pilot_.registerExternalBridge(
      std::make_shared<PixhawkBridge>(nh_, pnh_, params_.quad_, RosTime));
  }

  if (params_.pipeline_cfg_.bridge_cfg.type == "CTRL") {
    ctrl_feedback_publisher_ =
      std::make_unique<CtrlFeedbackPublisher>(nh_, pnh_);
    pilot_.registerFeedbackCallback(
      std::bind(&CtrlFeedbackPublisher::addFeedback,
                ctrl_feedback_publisher_.get(), std::placeholders::_1));
  }

  pilot_.registerPipelineCallback(std::bind(
    &Px4Pilot::pipelineCallback, this, std::placeholders::_1,
    std::placeholders::_2, std::placeholders::_3, std::placeholders::_4,
    std::placeholders::_5, std::placeholders::_6, std::placeholders::_7));

  // for RotorS, it run at 100Hz
  run_pipeline_timer_ = nh_.createTimer(
    ros::Duration(pilot_.getParams().dt_min_), &Px4Pilot::runPipeline, this);

  reference_publishing_thread_ =
    std::thread(&Px4Pilot::referencePublisher, this);
}

Px4Pilot::~Px4Pilot() { shutdown_ = true; }

void Px4Pilot::runPipeline(const ros::TimerEvent& event) {
  pilot_.runPipeline(event.current_real.toSec());
}

void Px4Pilot::poseEstimateCallback(
  const geometry_msgs::PoseStampedConstPtr& msg) {
  Pose pose;
  pose.t = msg->header.stamp.toSec();
  pose.position = fromRosVec3(msg->pose.position);
  pose.attitude = fromRosQuaternion(msg->pose.orientation);

  pilot_.odometryCallback(pose);
}

void Px4Pilot::stateEstimateCallback(const agiros_msgs::QuadState& msg) {
  QuadState state;
  state.setZero();
  state.t = msg.header.stamp.toSec();
  state.p = fromRosVec3(msg.pose.position);
  state.q(Quaternion(msg.pose.orientation.w, msg.pose.orientation.x,
                     msg.pose.orientation.y, msg.pose.orientation.z));
  state.v = fromRosVec3(msg.velocity.linear);
  state.w = fromRosVec3(msg.velocity.angular);
  state.a = fromRosVec3(msg.acceleration.linear);
  state.tau = fromRosVec3(msg.acceleration.angular);
  state.bw = fromRosVec3(msg.gyr_bias);
  state.ba = fromRosVec3(msg.acc_bias);
  state.mot = motor_speeds_;

  pilot_.odometryCallback(state);
}

void Px4Pilot::odometryEstimateCallback(const nav_msgs::OdometryConstPtr& msg) {
  QuadState state;
  // std::cout << "odometryEstimateCallback" << std::endl;

  state.setZero();
  state.t = msg->header.stamp.toSec();
  state.p = fromRosVec3(msg->pose.pose.position);
  state.q(Quaternion(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                     msg->pose.pose.orientation.y,
                     msg->pose.pose.orientation.z));
  state.v = fromRosVec3(msg->twist.twist.linear);
  state.w = fromRosVec3(msg->twist.twist.angular);
  state.mot = motor_speeds_;
  pilot_.odometryCallback(state);
}

void Px4Pilot::guardPoseEstimateCallback(
  const geometry_msgs::PoseStampedConstPtr& msg) {
  Pose pose;
  pose.t = msg->header.stamp.toSec();
  pose.position = fromRosVec3(msg->pose.position);
  pose.attitude = fromRosQuaternion(msg->pose.orientation);

  pilot_.guardOdometryCallback(pose);
}

void Px4Pilot::guardStateEstimateCallback(const agiros_msgs::QuadState& msg) {
  QuadState state;
  state.setZero();
  state.t = msg.header.stamp.toSec();
  state.p = fromRosVec3(msg.pose.position);
  state.q(Quaternion(msg.pose.orientation.w, msg.pose.orientation.x,
                     msg.pose.orientation.y, msg.pose.orientation.z));
  state.v = fromRosVec3(msg.velocity.linear);
  state.w = fromRosVec3(msg.velocity.angular);
  state.a = fromRosVec3(msg.acceleration.linear);
  state.tau = fromRosVec3(msg.acceleration.angular);
  state.bw = fromRosVec3(msg.gyr_bias);
  state.ba = fromRosVec3(msg.acc_bias);
  state.mot = motor_speeds_;

  pilot_.guardOdometryCallback(state);
}

void Px4Pilot::guardOdometryEstimateCallback(
  const nav_msgs::OdometryConstPtr& msg) {
  QuadState state;

  state.setZero();
  state.t = msg->header.stamp.toSec();
  state.p = fromRosVec3(msg->pose.pose.position);
  state.q(Quaternion(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                     msg->pose.pose.orientation.y,
                     msg->pose.pose.orientation.z));
  state.v = fromRosVec3(msg->twist.twist.linear);
  state.w = fromRosVec3(msg->twist.twist.angular);
  state.mot = motor_speeds_;
  pilot_.guardOdometryCallback(state);
}

void Px4Pilot::imuCallback(const sensor_msgs::ImuConstPtr& msg) {
  pilot_.imuCallback(ImuSample(msg->header.stamp.toSec(),
                               fromRosVec3(msg->linear_acceleration),
                               fromRosVec3(msg->angular_velocity)));
}

void Px4Pilot::motorSpeedCallback(const mav_msgs::Actuators& msg) {
  motor_speeds_ =
    (Vector<4>() << msg.angular_velocities[0], msg.angular_velocities[1],
     msg.angular_velocities[2], msg.angular_velocities[3])
      .finished();
  pilot_.motorSpeedCallback(motor_speeds_);
}

void Px4Pilot::startCallback(const std_msgs::EmptyConstPtr& msg) {
  ROS_INFO("START command received!");
  pilot_.start();
}

void Px4Pilot::forceHoverCallback(const std_msgs::EmptyConstPtr& msg) {
  ROS_INFO("FORCE_HOVER command received!");
  pilot_.forceHover();
}

void Px4Pilot::goToPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  ROS_INFO("GO_TO_POSE command received!");
  QuadState end_state;
  end_state.setZero();
  end_state.p =
    Vector<3>(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  end_state.q(Quaternion(msg->pose.orientation.w, msg->pose.orientation.x,
                         msg->pose.orientation.y, msg->pose.orientation.z));
  pilot_.goToPose(end_state);
}

void Px4Pilot::velocityCallback(
  const geometry_msgs::TwistStampedConstPtr& msg) {
  pilot_.setVelocityReference(fromRosVec3(msg->twist.linear),
                              msg->twist.angular.z);
}

void Px4Pilot::landCallback(const std_msgs::EmptyConstPtr& msg) {
  ROS_INFO("LAND command received!");
  pilot_.land();
}

void Px4Pilot::offCallback(const std_msgs::EmptyConstPtr& msg) {
  ROS_INFO("OFF command received!");
  pilot_.off();
}

void Px4Pilot::enableCallback(const std_msgs::BoolConstPtr& msg) {
  ROS_INFO("Computing active: %s!", msg->data ? "true" : "false");
  pilot_.enable(msg->data);
}

void Px4Pilot::triggerGuardCallback(const std_msgs::EmptyConstPtr& msg) {
  ROS_WARN("Triggering Guard!");
  pilot_.triggerGuard();
}

void Px4Pilot::trajectoryCallback(const agiros_msgs::ReferenceConstPtr& msg) {
  ROS_INFO("Received sampled trajectory!");
  SetpointVector sampled_trajectory;
  sampled_trajectory.reserve(msg->points.size());
  const Scalar t_shift = ros::Time::now().toSec() - msg->points[0].state.t;
  for (auto point : msg->points) {
    sampled_trajectory.emplace_back(Setpoint());
    sampled_trajectory.back().state.setZero();
    // if points[0].state.t = 0, then t_shift = ros::Time::now().toSec()
    sampled_trajectory.back().state.t = point.state.t + t_shift;
    sampled_trajectory.back().state.p = fromRosVec3(point.state.pose.position);
    sampled_trajectory.back().state.q(
      fromRosQuaternion(point.state.pose.orientation));
    sampled_trajectory.back().state.v =
      fromRosVec3(point.state.velocity.linear);
    sampled_trajectory.back().state.w =
      fromRosVec3(point.state.velocity.angular);
    sampled_trajectory.back().state.a =
      fromRosVec3(point.state.acceleration.linear);
    sampled_trajectory.back().state.j = fromRosVec3(point.state.jerk);
    sampled_trajectory.back().state.s = fromRosVec3(point.state.snap);

    sampled_trajectory.back().input.t = point.state.t + t_shift;

    if (point.command.is_single_rotor_thrust) {
      if (!fromRosThrusts(point.command.thrusts,
                          &sampled_trajectory.back().input.thrusts)) {
        break;
      }
    } else {
      sampled_trajectory.back().input.collective_thrust =
        point.command.collective_thrust;
      sampled_trajectory.back().input.omega =
        fromRosVec3(point.command.bodyrates);
    }
  }
  pilot_.addSampledTrajectory(sampled_trajectory);
}

void Px4Pilot::voltageCallback(const sensor_msgs::BatteryStateConstPtr& msg) {
  pilot_.voltageCallback(msg->voltage);
}

void Px4Pilot::feedthroughCommandCallback(
  const agiros_msgs::CommandConstPtr& msg) {
  pilot_.setFeedthroughCommand(fromRosCommand(*msg, pilot_.getTime()));
}

void Px4Pilot::pipelineCallback(const QuadState& state,
                                const Feedback& feedback,
                                const ReferenceVector& references,
                                const SetpointVector& setpoints,
                                const SetpointVector& outer_setpoints,
                                const SetpointVector& inner_setpoints,
                                const Command& command) {
  agiros_msgs::QuadState msg = toRosQuadState(state);

  nav_msgs::Odometry msg_odo;
  msg_odo.header = msg.header;
  msg_odo.pose.pose = msg.pose;
  msg_odo.twist.twist = msg.velocity;

  cmd_pub_.publish(toRosCommand(command));
  state_odometry_pub_.publish(msg_odo);
  state_pub_.publish(msg);

  agiros_msgs::Telemetry telemetry_msg;
  telemetry_msg.t = state.t;
  telemetry_msg.header = msg.header;
  telemetry_msg.bridge_type.data = pilot_.getActiveBridgeType();
  telemetry_msg.bridge_armed.data = feedback.armed;
  telemetry_msg.guard_triggered.data = pilot_.guardTriggered();

  // get the current reference from the pilot
  if (!setpoints.empty()) {
    telemetry_msg.reference.pose.position =
      toRosPoint(setpoints.front().state.p);
    telemetry_msg.reference.pose.orientation =
      toRosQuaternion(setpoints.front().state.q());
    telemetry_msg.reference.velocity.linear =
      toRosVector(setpoints.front().state.v);
    telemetry_msg.reference.velocity.angular =
      toRosVector(setpoints.front().state.w);
    active_reference_visualizer_.visualize(setpoints, "active");
  }

  if (!outer_setpoints.empty())
    outer_setpoints_visualizer_.visualize(outer_setpoints, "outer_setpoints");
  if (!inner_setpoints.empty())
    inner_setpoints_visualizer_.visualize(inner_setpoints, "inner_setpoints");

  if (references != references_) {
    references_ = references;
    reference_publishing_cv_.notify_all();
  }

  const int number_of_references = (int)references.size();
  telemetry_msg.num_references_in_queue = number_of_references;
  if (number_of_references > 0) {
    telemetry_msg.reference_left_duration =
      references.back()->getEndTime() - ros::Time::now().toSec();
  } else {
    telemetry_msg.reference_left_duration = 0.0;
  }
  telemetry_msg.rmse = updateRmse(state, setpoints, references);
  telemetry_msg.voltage = pilot_.getVoltage();

  telemetry_pub_.publish(telemetry_msg);
  if (params_.publish_log_var_) {
    publishLoggerDebugMsg();
  }
}

void Px4Pilot::referencePublisher() {
  while (!shutdown_ && ros::ok()) {
    std::unique_lock<std::mutex> lk(reference_publishing_mtx_);
    reference_publishing_cv_.wait_for(lk, std::chrono::seconds(1));
    if (shutdown_ || !ros::ok()) break;
    const ReferenceVector references = references_;
    reference_visualizer_.visualize(references);
  }
}

Scalar Px4Pilot::updateRmse(const QuadState& state,
                            const SetpointVector& setpoints,
                            const ReferenceVector& references) const {
  static Scalar cumulative_weighted_square_sum = 0.0;
  static Scalar cumulative_time = 0.0;
  static Scalar t_last_sample = NAN;
  static std::shared_ptr<ReferenceBase> last_active_reference;

  // No references given, therefore reset.
  if (references.empty()) {
    last_active_reference.reset();
    t_last_sample = NAN;
    cumulative_weighted_square_sum = 0.0;
    cumulative_time = 0.0;
    return 0.0;
  }

  // Active reference changed, therefore reset.
  if (references.front() != last_active_reference) {
    last_active_reference = references.front();
    t_last_sample = NAN;
    cumulative_weighted_square_sum = 0.0;
    cumulative_time = 0.0;
    return 0.0;
  }

  // Last sample time unknown, therefore reset and set last sample time.
  if (!std::isfinite(t_last_sample)) {
    t_last_sample = state.t;
    cumulative_weighted_square_sum = 0.0;
    cumulative_time = 0.0;
    return 0.0;
  }

  // If no setpoints given, skip.
  if (!setpoints.empty()) {
    const Scalar dt = state.t - t_last_sample;
    t_last_sample = state.t;

    // If time not monotonically increasing, skip.
    if (dt > 0.0) {
      const Vector<3> error = state.p - setpoints.front().state.p;
      cumulative_weighted_square_sum += dt * error.transpose() * error;
      cumulative_time += dt;
    }
  }

  // If samples gathered, return RMSE, otherwise return 0.
  return cumulative_time > 0.0
           ? sqrt(cumulative_weighted_square_sum / cumulative_time)
           : 0.0;
}

Command Px4Pilot::getCommand() const { return pilot_.getCommand(); }

void Px4Pilot::advertiseDebugVariable(const std::string& var_name) {
  logger_publishers_[var_name] =
    pnh_.advertise<agiros_msgs::DebugMsg>(var_name, 1);
}

void Px4Pilot::publishDebugVariable(const std::string& name,
                                    const PublishLogContainer& container) {
  if (logger_publishers_.find(name) == logger_publishers_.end()) {
    // Not found, advertise it and create it
    advertiseDebugVariable(name);
    if (container.advertise) return;
  }

  // Fill message
  agiros_msgs::DebugMsg debug_msg;
  for (unsigned int i = 0; i < container.data.size(); ++i)
    debug_msg.data.push_back(container.data(i));
  // RosTime
  debug_msg.header.stamp = ros::Time::now();
  // Publish
  logger_publishers_[name].publish(debug_msg);
}

void Px4Pilot::publishLoggerDebugMsg() {
  Logger::for_each_instance(std::bind(&Px4Pilot::publishDebugVariable, this,
                                      std::placeholders::_1,
                                      std::placeholders::_2));
}

bool Px4Pilot::getQuadrotor(Quadrotor* const quad) const {
  return pilot_.getQuadrotor(quad);
}

Pilot& Px4Pilot::getPilot() { return pilot_; }


}  // namespace agi

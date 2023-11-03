#pragma once

#include <ros/ros.h>

#include "agilib/math/types.hpp"

static constexpr auto RosTime = []() -> agi::Scalar {
  return ros::Time::now().toSec();
};

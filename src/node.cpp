#include <ros/ros.h>

#include "agihawk/px4_pilot.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "agihawk_pilot");
  agi::Px4Pilot pilot;
  
  ros::spin();
  return 0;
}

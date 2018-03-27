#include <ros/ros.h>

#include "OdomCalibration.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_calibration_node");

  OdomCalibration odom;

  ros::spin();
  return 0;
}

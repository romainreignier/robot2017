#include <ros/ros.h>

#include "PolarControl.h"

int main(int _argc, char** _argv)
{
  ros::init(_argc, _argv, "polar_control");

  PolarControl control;

  ros::spin();
  return 0;
}

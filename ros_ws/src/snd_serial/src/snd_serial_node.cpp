#include <ros/ros.h>

#include "BoardComm.h"

int main(int _argc, char** _argv)
{
  ros::init(_argc, _argv, "snd_serial_node");
  BoardComm ser;
  ser.run();
  return 0;
}

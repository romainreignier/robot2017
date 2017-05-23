#include <string>

#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

#include "snd_control/snd_hardware.h"

int main(int _argc, char** _argv)
{
  ros::init(_argc, _argv, "snd_control_node");
  ros::NodeHandle nh("~");

  double updateRate = 20.0;
  if(!nh.getParam("update_rate", updateRate))
  {
    ROS_WARN_STREAM(
      "No '~update_rate' parameter found. Default: " << updateRate);
  }

  snd_control::SndHardwareRos robot;
  controller_manager::ControllerManager cm(&robot);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Control Loop
  ros::Time prevTime = ros::Time::now();
  ros::Rate rate(updateRate);
  while(ros::ok())
  {
    const ros::Time thisTime = ros::Time::now();
    const ros::Duration period = thisTime - prevTime;
    prevTime = thisTime;

    // Retrieve encoder values
    robot.read();
    cm.update(thisTime, period);
    // Publish motor velocities
    robot.write();

    rate.sleep();
  }
  return 0;
}

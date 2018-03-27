#include <ros/ros.h>

#include "PolarControlROS.h"

int main(int _argc, char** _argv)
{
    ros::init(_argc, _argv, "polar_control_ros");
    PolarControlROS pc;

    ros::spin();
    return 0;
}

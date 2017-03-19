#include <ros/ros.h>

#include "SerialComm.h"

int main(int _argc, char** _argv)
{
    ros::init(_argc, _argv, "snd_serial_node");
    SerialComm ser;
    ser.run();
    return 0;
}

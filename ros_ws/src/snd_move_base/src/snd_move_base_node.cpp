#include <ros/ros.h>

#include "MoveBase.h"

int main(int _argc, char** _argv)
{
    ros::init(_argc, _argv, "snd_move_base");
    snd_move_base::MoveBase mb;
    ros::spin();
    return 0;
}

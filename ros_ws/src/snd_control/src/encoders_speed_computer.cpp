#include <ros/ros.h>

#include <snd_msgs/Encoders.h>
#include <snd_msgs/Motors.h>

ros::Publisher speedPub;
snd_msgs::EncodersConstPtr lastMsg;

void encodersCb(const snd_msgs::EncodersConstPtr& _msg)
{
  if(lastMsg)
  {
    const ros::Duration dt = _msg->header.stamp - lastMsg->header.stamp;
    snd_msgs::Motors speed;
    speed.header = _msg->header;
    speed.left = (_msg->left_pos - lastMsg->left_pos) / dt.toSec();
    speed.right = (_msg->right_pos - lastMsg->right_pos) / dt.toSec();
    speedPub.publish(speed);
  }
  lastMsg = _msg;
}

int main(int _argc, char** _argv)
{
    ros::init(_argc, _argv, "encoders_speed_computer");
    ROS_INFO("Ready to compute the speed from topic /encoders on /encoders_speed");
    ros::NodeHandle nh;
    ros::Subscriber encodersSub = nh.subscribe("/encoders", 1, encodersCb);
    speedPub = nh.advertise<snd_msgs::Motors>("/encoders_speed", 10);
    ros::spin();
    return 0;
}

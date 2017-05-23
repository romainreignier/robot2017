#include <iostream>

#include <ros/ros.h>

#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>

ros::Publisher armServoPub;
ros::Publisher graspServoPub;
ros::Publisher launchServoPub;
ros::Publisher ramp1ServoPub;
ros::Publisher ramp2ServoPub;
ros::Publisher pumpPub;

uint16_t kLaunchServoArm;
uint16_t kLaunchServoLaunch;
uint16_t kArmServoLow;
uint16_t kArmServoHigh;
uint16_t kGraspServoClose;
uint16_t kGraspServoOpen;
uint16_t kRamp1ServoOpen;
uint16_t kRamp1ServoClose;
uint16_t kRamp2ServoOpen;
uint16_t kRamp2ServoClose;

void joyCb(const sensor_msgs::JoyConstPtr& _msg)
{
  if(_msg->buttons[12])
  {
    // Triangle = Arm High
    std_msgs::UInt16 command;
    command.data = kArmServoHigh;
    armServoPub.publish(command);
  }
  else if(_msg->buttons[15])
  {
    // Square = Arm Low
    std_msgs::UInt16 command;
    command.data = kArmServoLow;
    armServoPub.publish(command);
  }
  else if(_msg->buttons[13])
  {
    // Circle = Gripper Open
    std_msgs::UInt16 command;
    command.data = kGraspServoOpen;
    graspServoPub.publish(command);
  }
  else if(_msg->buttons[14])
  {
    // Cross = Gripper Close
    std_msgs::UInt16 command;
    command.data = kGraspServoClose;
    graspServoPub.publish(command);
  }
  else if(_msg->buttons[4])
  {
    // Up Arrow = Ramp 1 Open
    std_msgs::UInt16 command;
    command.data = kRamp1ServoOpen;
    ramp1ServoPub.publish(command);
  }
  else if(_msg->buttons[6])
  {
    // Down Arrow = Ramp 1 Close
    std_msgs::UInt16 command;
    command.data = kRamp1ServoClose;
    ramp1ServoPub.publish(command);
  }
  else if(_msg->buttons[7])
  {
    // Left Arrow = Ramp 2 Open
    std_msgs::UInt16 command;
    command.data = kRamp2ServoOpen;
    ramp2ServoPub.publish(command);
  }
  else if(_msg->buttons[5])
  {
    // Right Arrow = Ramp 2 Close
    std_msgs::UInt16 command;
    command.data = kRamp2ServoClose;
    ramp2ServoPub.publish(command);
  }
  else if(_msg->buttons[11])
  {
    // L1 = Pump On
    std_msgs::Bool command;
    command.data = true;
    pumpPub.publish(command);
  }
  else if(_msg->buttons[9])
  {
    // L2 = Pump Off
    std_msgs::Bool command;
    command.data = false;
    pumpPub.publish(command);
  }
  else if(_msg->buttons[0])
  {
    // Select = Arm funny
    std_msgs::UInt16 command;
    command.data = kLaunchServoArm;
    launchServoPub.publish(command);
  }
  else if(_msg->buttons[3])
  {
    // Start = Launch funny!
    std_msgs::UInt16 command;
    command.data = kLaunchServoLaunch;
    launchServoPub.publish(command);
  }
}

int main(int _argc, char** _argv)
{
  ros::init(_argc, _argv, "test_joy");
  ros::NodeHandle nh("~");
  ros::Subscriber joySub = nh.subscribe("/joy", 1, &joyCb);
  armServoPub = nh.advertise<std_msgs::UInt16>("/arm_servo", 1);
  graspServoPub = nh.advertise<std_msgs::UInt16>("/grasp_servo", 1);
  launchServoPub = nh.advertise<std_msgs::UInt16>("/launch_servo", 1);
  ramp1ServoPub = nh.advertise<std_msgs::UInt16>("/ramp1_servo", 1);
  ramp2ServoPub = nh.advertise<std_msgs::UInt16>("/ramp2_servo", 1);
  pumpPub = nh.advertise<std_msgs::Bool>("/pump", 1);

  ros::NodeHandle controlNh("snd_control");
  kLaunchServoArm = controlNh.param("launch_servo/armed_value", 270);
  kLaunchServoLaunch = controlNh.param("launch_servo/launch_value", 110);
  kArmServoLow = controlNh.param("arm_servo/low_value", 385);
  kArmServoHigh = controlNh.param("arm_servo/high_value", 208);
  kGraspServoClose = controlNh.param("grasp_servo/close_value", 335);
  kGraspServoOpen = controlNh.param("grasp_servo/open_value", 380);
  kRamp1ServoOpen = controlNh.param("ramp1_servo/open_value", 280);
  kRamp1ServoClose = controlNh.param("ramp1_servo/close_value", 480);
  kRamp2ServoOpen = controlNh.param("ramp2_servo/open_value", 290);
  kRamp2ServoClose = controlNh.param("ramp2_servo/close_value", 110);

  std::cout << "USAGE:\n"
               "- Triangle: Arm HIGH\n"
               "- Square: Arm LOW\n"
               "- Circle: Gripper Open\n"
               "- Cross: Gripper Close\n"
               "- Start: Launch Funny\n"
               "- Select: Arm Funny\n"
               "- Up Arrow: Ramp1 Up\n"
               "- Down Arrow: Ramp1 Down\n"
               "- Left Arrow: Ramp2 Up\n"
               "- Right Arrow: Ramp2 Down\n";

  ros::spin();
  return 0;
}

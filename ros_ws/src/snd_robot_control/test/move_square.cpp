#include <vector>

#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <snd_msgs/MoveAngularAction.h>
#include <snd_msgs/MoveLinearAction.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_square");

  ros::NodeHandle nh;
  ros::Time::waitForValid();

  const double squareSize = nh.param("size", 500.0);

  ROS_INFO_STREAM("move_square node launched for a square of " << squareSize << " mm");

  snd_msgs::MoveLinearGoal linearGoal;
  linearGoal.distance = squareSize;

  snd_msgs::MoveAngularGoal angularGoal;
  angularGoal.angle = 90;

  actionlib::SimpleActionClient<snd_msgs::MoveLinearAction> linearAc(
    nh, "/polar_control/move_linear", true);

  ROS_INFO("Waiting for linear action server to start.");
  linearAc.waitForServer();

  actionlib::SimpleActionClient<snd_msgs::MoveAngularAction> angularAc(
    nh, "/polar_control/move_angular", true);

  ROS_INFO("Waiting for angular action server to start.");
  angularAc.waitForServer();

  for(size_t i = 0; i < 4; ++i)
  {
    ROS_INFO_STREAM("Goal distance: " << linearGoal.distance);

    linearAc.sendGoal(linearGoal);

    // wait for the action to return
    bool finished_before_timeout = linearAc.waitForResult(ros::Duration(40.0));

    if(finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = linearAc.getState();
      ROS_INFO_STREAM("Action finished: " << state.toString());
      ROS_INFO_STREAM("Goal desc: " << state.getText());
      if(state != actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_WARN("Abort");
        break;
      }
    }
    else
    {
      ROS_INFO("Action did not finish before the time out.");
    }

    ROS_INFO_STREAM("Goal angular: " << angularGoal.angle);

    angularAc.sendGoal(angularGoal);

    // wait for the action to return
    finished_before_timeout = angularAc.waitForResult(ros::Duration(40.0));

    if(finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = angularAc.getState();
      ROS_INFO_STREAM("Action finished: " << state.toString());
      ROS_INFO_STREAM("Goal desc: " << state.getText());
      if(state != actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_WARN("Abort");
        break;
      }
    }
    else
    {
      ROS_INFO("Action did not finish before the time out.");
    }
  }

  return 0;
}

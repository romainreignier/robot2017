#include <iostream>

#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <snd_msgs/MoveAngularAction.h>
#include <snd_msgs/MoveAngularStepAction.h>
#include <snd_msgs/MoveLinearAction.h>
#include <snd_msgs/MoveLinearStepAction.h>

int main(int argc, char** argv)
{
  ros::init(
    argc, argv, "polar_control_tester", ros::init_options::AnonymousName);

  ros::NodeHandle nh;
  ros::Time::waitForValid();

  if(argc == 3 && std::string(argv[1]) == "move_linear")
  {
    const float distance = std::atof(argv[2]);

    actionlib::SimpleActionClient<snd_msgs::MoveLinearAction> ac(
      nh, "/polar_control/move_linear", true);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();

    ROS_INFO_STREAM("Goal distance: " << distance);

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    snd_msgs::MoveLinearGoal goal;
    goal.distance = distance;
    ac.sendGoal(goal);

    // wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(40.0));

    if(finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO_STREAM("Action finished: " << state.toString());
      ROS_INFO_STREAM("Goal desc: " << state.getText());
    }
    else
    {
      ROS_INFO("Action did not finish before the time out.");
    }
  }
  else if(argc == 3 && std::string(argv[1]) == "move_angular")
  {
    const float angle = std::atof(argv[2]);

    actionlib::SimpleActionClient<snd_msgs::MoveAngularAction> ac(
      nh, "/polar_control/move_angular", true);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();

    ROS_INFO_STREAM("Goal angular: " << angle);

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    snd_msgs::MoveAngularGoal goal;
    goal.angle = angle;
    ac.sendGoal(goal);

    // wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(40.0));

    if(finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO_STREAM("Action finished: " << state.toString());
      ROS_INFO_STREAM("Goal desc: " << state.getText());
    }
    else
    {
      ROS_INFO("Action did not finish before the time out.");
    }
  }
  else if(argc == 3 && std::string(argv[1]) == "move_linear_step")
  {
    const float distance = std::atof(argv[2]);

    actionlib::SimpleActionClient<snd_msgs::MoveLinearStepAction> ac(
      nh, "/polar_control/move_linear_step", true);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();

    ROS_INFO_STREAM("Goal distance: " << distance);

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    snd_msgs::MoveLinearStepGoal goal;
    goal.distance = distance;
    ac.sendGoal(goal);

    // wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(40.0));

    if(finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO_STREAM("Action finished: " << state.toString());
      ROS_INFO_STREAM("Goal desc: " << state.getText());
    }
    else
    {
      ROS_INFO("Action did not finish before the time out.");
    }
  }
  else if(argc == 3 && std::string(argv[1]) == "move_angular_step")
  {
    const float angle = std::atof(argv[2]);

    actionlib::SimpleActionClient<snd_msgs::MoveAngularStepAction> ac(
      nh, "/polar_control/move_angular_step", true);

    ROS_INFO("Waiting for action server to start.");
    ac.waitForServer();

    ROS_INFO_STREAM("Goal angular: " << angle);

    ROS_INFO("Action server started, sending goal.");
    // send a goal to the action
    snd_msgs::MoveAngularStepGoal goal;
    goal.angle = angle;
    ac.sendGoal(goal);

    // wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(40.0));

    if(finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO_STREAM("Action finished: " << state.toString());
      ROS_INFO_STREAM("Goal desc: " << state.getText());
    }
    else
    {
      ROS_INFO("Action did not finish before the time out.");
    }
  }
  else
  {
    std::cout << "Test for polar_control_node\n"
              << "move_linear <distance_mm>\n"
              << "move_angular <angular_deg>\n"
              << "move_linear_step <distance_mm>\n"
              << "move_angular_step <angle_deg>\n";
  }

  return 0;
}

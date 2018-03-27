#include <vector>

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>

using MoveBaseClient =
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

int main(int _argc, char** _argv)
{
  ros::init(_argc, _argv, "move_square");
  ros::NodeHandle nh{"~"};
  ros::Time::waitForValid();

  ROS_INFO("move_square node launched");

  const double squareSize = nh.param("size", 1.0);

  // Create the goals
  std::vector<move_base_msgs::MoveBaseGoal> goals;

  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "odom";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = 0.0;
    goal.target_pose.pose.position.y = 0.0;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    goals.push_back(goal);

    goal.target_pose.pose.position.x = squareSize;
    goal.target_pose.pose.position.y = 0.0;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    goals.push_back(goal);

    goal.target_pose.pose.position.x = squareSize;
    goal.target_pose.pose.position.y = 0.0;
    goal.target_pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(M_PI / 2.0);
    goals.push_back(goal);

    goal.target_pose.pose.position.x = squareSize;
    goal.target_pose.pose.position.y = squareSize;
    goal.target_pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(M_PI / 2.0);
    goals.push_back(goal);

    goal.target_pose.pose.position.x = squareSize;
    goal.target_pose.pose.position.y = squareSize;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI);
    goals.push_back(goal);

    goal.target_pose.pose.position.x = 0.0;
    goal.target_pose.pose.position.y = squareSize;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI);
    goals.push_back(goal);

    goal.target_pose.pose.position.x = 0.0;
    goal.target_pose.pose.position.y = squareSize;
    goal.target_pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(3 * M_PI / 2.0);
    goals.push_back(goal);

    goal.target_pose.pose.position.x = 0.0;
    goal.target_pose.pose.position.y = 0.0;
    goal.target_pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(3 * M_PI / 2.0);
    goals.push_back(goal);

    goal.target_pose.pose.position.x = 0.0;
    goal.target_pose.pose.position.y = 0.0;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
    goals.push_back(goal);
  }

  MoveBaseClient ac("move_base", true);

  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer(); // will wait for infinite time

  for(const auto& goal : goals)
  {
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Arrived!");
    }
    else
    {
      ROS_WARN("The base failed to move");
      const actionlib::SimpleClientGoalState state = ac.getState();
      ROS_WARN_STREAM("Action finished: " << state.toString());
      ROS_WARN_STREAM("Goal desc: " << state.getText());
      break;
    }
  }

  return 0;
}

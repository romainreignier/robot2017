#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <move_base_msgs/MoveBaseAction.h>

class MoveBaseRelay
{
public:
    MoveBaseRelay()
        : m_nh{"~"}, m_moveBaseClient{"/move_base", true}
    {
        ros::Time::waitForValid(); // Just in case with Gazebo
        m_moveBaseSubscriber =
            m_nh.subscribe("/move_base_simple/goal", 1, &MoveBaseRelay::goalCb, this);
    }

    void goalCb(const geometry_msgs::PoseStampedConstPtr& _msg)
    {
        ROS_INFO("New goal received.");
        // m_moveBaseClient.cancelAllGoals();
        try
        {
            ROS_INFO("Send new trajectory.");
			move_base_msgs::MoveBaseGoal goal;
            goal.target_pose = *_msg;
			m_moveBaseClient.sendGoal(goal);
        }
        catch(const std::exception& _e)
        {
            ROS_INFO("Something went wrong, cancelling the goal.");
            m_moveBaseClient.cancelAllGoals();
        }
    }

private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_moveBaseSubscriber;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
        m_moveBaseClient;
};

int main(int _argc, char** _argv)
{
    ros::init(_argc, _argv, "move_base_relay");
    MoveBaseRelay mbr;
    ros::spin();
    return 0;
}

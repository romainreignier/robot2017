#include <dwb_local_planner/debug_dwb_local_planner.h>
#include <nav_2d_utils/conversions.h>

#include <nav_msgs/Odometry.h>
#include <robot_move/FollowSegment.h>
#include <ros/ros.h>

static std::shared_ptr<dwb_local_planner::DebugDWBLocalPlanner> planner;
static nav_msgs::OdometryConstPtr lastOdom;
static TFListenerPtr transformer;
static ros::Publisher velPub;

void onNewOdometry(nav_msgs::OdometryConstPtr _odom) { lastOdom = _odom; }

bool followPath(robot_move::FollowSegmentRequest& _req,
                robot_move::FollowSegmentResponse& _resp)
{
    (void)_resp;
    ros::Rate rate(20);
    ROS_INFO("Starting following path");
    planner->setPlan(_req.path);
    bool stop = false;
    while(!stop)
    {
        if(!transformer->waitForTransform("odom", "base_link", ros::Time(0),
                                          ros::Duration(0.2)))
        {
            ROS_ERROR("Can't transform from base_link to odom");
            return false;
        }
        tf::StampedTransform odomToMap;
        transformer->lookupTransform("odom", "base_link", ros::Time(0),
                                     odomToMap);
        nav_2d_msgs::Pose2DStamped robotPoseInMapFrame;
        robotPoseInMapFrame.header.stamp = odomToMap.stamp_;
        robotPoseInMapFrame.header.frame_id = odomToMap.frame_id_;
        robotPoseInMapFrame.pose.x = odomToMap.getOrigin().x();
        robotPoseInMapFrame.pose.y = odomToMap.getOrigin().y();
        robotPoseInMapFrame.pose.theta = tf::getYaw(odomToMap.getRotation());

        if(!lastOdom)
        {
            ROS_ERROR("No last odom");
            return false;
        }
        if(fabs((lastOdom->header.stamp - ros::Time::now()).toSec()) > 0.5)
        {
            ROS_ERROR("Too old odometry !");
            return false;
        }
        ROS_INFO("Compute Velocities");
        nav_2d_msgs::Twist2D vel;
        vel.x = lastOdom->twist.twist.linear.x;
        vel.theta = lastOdom->twist.twist.angular.z;
        nav_2d_msgs::Twist2DStamped res =
            planner->computeVelocityCommands(robotPoseInMapFrame, vel);

        geometry_msgs::Twist cmd = nav_2d_utils::twist2Dto3D(res.velocity);
        velPub.publish(cmd);

        stop = planner->isGoalReached(robotPoseInMapFrame, vel);
        ros::spinOnce();
        rate.sleep();
    }
    geometry_msgs::Twist zero;
    velPub.publish(zero);
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plan_node");
    planner = std::make_shared<dwb_local_planner::DebugDWBLocalPlanner>();
    ROS_INFO("Plan Node");

    transformer = std::make_shared<tf::TransformListener>();
    CostmapROSPtr costmap_ros =
        std::make_shared<costmap_2d::Costmap2DROS>("costmap", *transformer);
    planner->initialize("dwb_local_planner", transformer, costmap_ros);

    ros::NodeHandle nh;
    ROS_INFO("Starting /follow_path service");
    auto followTrajService = nh.advertiseService("/follow_path", &followPath);

    velPub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ros::Subscriber sub = nh.subscribe("odom", 1, &onNewOdometry);
    ros::spin();
}

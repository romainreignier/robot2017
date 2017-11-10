#include <ros/ros.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

int main(int _argc, char** _argv)
{
  ros::init(_argc, _argv, "map_odom_tf_broadcaster");
  ros::NodeHandle nhPrivate{"~"};

  geometry_msgs::TransformStamped staticTransform;

  // Listen to tf to get the transform between /odom and /base_link
  {
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener{tfBuffer};

    try
    {
      staticTransform = tfBuffer.lookupTransform(
        "base_link", "odom", ros::Time{0}, ros::Duration{10});
    }
    catch(const tf2::TransformException& _e)
    {
      ROS_WARN_STREAM("Could not get transform between 'base_link' and 'odom': "
                      << _e.what());
    }
  }

  tf2_ros::StaticTransformBroadcaster staticBroadcaster;

  staticTransform.header.frame_id = "map";
  staticTransform.child_frame_id = "odom";

  staticBroadcaster.sendTransform(staticTransform);
  ROS_INFO("Tf between 'map' and 'odom' published");
  ros::spin();
  return 0;
}

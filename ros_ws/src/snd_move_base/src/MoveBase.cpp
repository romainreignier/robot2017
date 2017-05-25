#include "MoveBase.h"
#include <angles/angles.h>
#include <tf/tf.h>
#include <tf2/transform_datatypes.h>

// #include <tf2_bullet/tf2_bullet.h>

namespace snd_move_base
{

MoveBase::MoveBase()
  : m_nh{"~"},
    m_actionServer{
      "/move_base", boost::bind(&MoveBase::executeCb, this, _1), false},
    m_tfListener{m_tfBuffer}, m_isBlockedByObstacle{false}
{
  // Parameters
  m_updateRate = m_nh.param("update_rate", 20);
  m_obstacleTimeout = m_nh.param("obstacle_timeout", 4.0);
  m_baseFrame = m_nh.param("base_frame", std::string{"base_link"});
  m_mapFrame = m_nh.param("map_frame", std::string{"map"});
  m_vLinMax = m_nh.param("vlin_max", 0.5);
  m_vAngMax = m_nh.param("vang_max", M_PI / 4);

  m_laserSub = m_nh.subscribe(
    "/scan",
    1,
    boost::function<void(const sensor_msgs::LaserScanConstPtr&)>(
      [&](const sensor_msgs::LaserScanConstPtr _msg) { m_laserMsg = _msg; }));

  m_proximitySub = m_nh.subscribe(
    "/proximity_sensors",
    1,
    boost::function<void(const snd_msgs::ProximitySensorsConstPtr&)>(
      [&](const snd_msgs::ProximitySensorsConstPtr _msg) {
        m_proximityMsg = _msg;
      }));

  m_cmdVelPub = m_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  // Create the Dynamic Reconfigure server
  m_dynParamServer.setCallback(
    boost::bind(&MoveBase::dynParamCb, this, _1, _2));

  m_actionServer.start();
  ROS_INFO_STREAM("MoveBase action server ready");
}

void MoveBase::executeCb(const move_base_msgs::MoveBaseGoalConstPtr& _goal)
{
  // Get goal

  // TODO: Transform goal pose in map frame
  // tf::transformPose

  geometry_msgs::PoseStamped targetPose = _goal->target_pose;
  geometry_msgs::TransformStamped targetTr =
    poseToTransform(_goal->target_pose);

  ROS_INFO_STREAM("New Action received to : " << targetPose);
  /*
  // 3 steps
  // Rotation to segment
  const float segmentAngle = angleSegment(robotPose, targetPose);
  move(0, segmentAngle);
  // Distance along segment
  geometry_msgs::PoseStamped robotPose = getRobotPose();
  const float lengthSegment = distance(robotPose, targetPose);
  move(lengthSegment, 0);
  // Rotation to targetYaw
  float targetYaw = tf::getYaw(targetPose.pose.orientation);
  move(0, targetYaw);
  */

  // Rotation to segment
  const auto robotPose = getRobotPose();
  const float segmentAngle = angleSegment(robotPose, targetPose);
  // Get Quaternion
  // tf::Quaternion segmentQuaternion;
  // segmentQuaternion.setRPY(0, 0, segmentAngle);
  const geometry_msgs::Quaternion segmentQuaternion =
    tf::createQuaternionMsgFromYaw(segmentAngle);
  geometry_msgs::PoseStamped pose1;
  pose1.pose.position = robotPose.pose.position;
  pose1.pose.orientation = segmentQuaternion;

  std::cout << "Move to point the segment " << pose1 << "\n";
  if(!move(pose1))
  {
    return;
  }
  // Distance along segment
  geometry_msgs::PoseStamped pose2;
  pose2.pose.position = targetPose.pose.position;
  pose2.pose.orientation = segmentQuaternion;
  std::cout << "Move along the segment" << pose2 << "\n";
  // target pose with segment orientation
  if(!move(pose2))
  {
    return;
  }

  // Rotation to targetYaw
  std::cout << "Rotate to target yaw" << targetPose << "\n";
  // Target Pose
  if(!move(targetPose))
  {
    return;
  }
  ROS_INFO("Finished action");
  m_actionServer.setSucceeded();
}

// void MoveBase::move(float _dist, float _angle)
bool MoveBase::move(const geometry_msgs::PoseStamped& _targetPose)
{
  ros::Rate r{m_updateRate};
  // geometry_msgs::PoseStamped startPose = getRobotPose();
  float errDistance;
  float errHeading;
  do
  {
    if(m_actionServer.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("Action preempted, cancelling the goal");
      stop();
      m_actionServer.setPreempted();
      return false;
    }

    // Get current robot pose
    geometry_msgs::PoseStamped robotPose = getRobotPose();

    // Get target pose in base_link
    geometry_msgs::TransformStamped mapInBase = m_tfBuffer.lookupTransform(
      m_baseFrame, m_mapFrame, ros::Time{0}, ros::Duration{2.0});
    btTransform trMapInBase = tf2::transformToBullet(mapInBase);

    btTransform targetTr = poseToBullet(_targetPose);

    btVector3 targetInBase = trMapInBase * targetTr.getOrigin();
    std::cout << "target in base: " << targetInBase.getX() << ", "
              << targetInBase.getY() << "\n";

    btTransform robotTr = tf2::transformToBullet(getRobotTransform());

    errDistance = std::copysign(targetTr.getOrigin().distance(robotTr.getOrigin()), targetInBase.getX());

    float angleRobotTarget = angleSegment(robotPose, _targetPose);
    std::cout << "angle robot - target: " << angleRobotTarget << "\n";

    // btQuaternion diffQuat = targetTr.getRotation() - robotTr.getRotation();
    // btScalar roll, pitch, yaw;
    // diffQuat.g

    // Compute error
    // errDistance = distance(robotPose, _targetPose);
    const float robotYaw = tf::getYaw(robotPose.pose.orientation);
    const float targetYaw = tf::getYaw(_targetPose.pose.orientation);
    errHeading = angles::normalize_angle(targetYaw - robotYaw);
    // errHeading = angles::normalize_angle(segmentAngle - robotYaw);

    // P(ID)
    geometry_msgs::Twist cmd;
    cmd.linear.x = m_linearP * errDistance;
    if(cmd.linear.x > m_vLinMax)
    {
      cmd.linear.x = m_vLinMax;
    }
    else if(cmd.linear.x < -m_vLinMax)
    {
      cmd.linear.x = -m_vLinMax;
    }
    cmd.angular.z = m_angularP * errHeading;
    if(cmd.angular.z > m_vAngMax)
    {
      cmd.angular.z = m_vAngMax;
    }
    else if(cmd.angular.z < - m_vAngMax)
    {
      cmd.angular.z = -m_vAngMax;
    }

    std::cout << "\nrobot " << robotPose << "\nerror distance: " << errDistance
              << "\nerror heading: " << errHeading
              << "\ncommand linear: " << cmd.linear.x
              << "\ncommand angular: " << cmd.angular.z << "\n";

    // Check obstacles
    // if(m_laserChecker.checkObstacle(m_laserMsg))
    /*
    if(m_proximityMsg->front)
    {
      if(!m_isBlockedByObstacle)
      {
        ROS_WARN("Obstacle!");
        m_isBlockedByObstacle = true;
        m_timeObstacleDetected = ros::Time::now();
      }
      else
      {
        if(ros::Time::now() - m_timeObstacleDetected >
           ros::Duration(m_obstacleTimeout))
        {
          ROS_WARN_STREAM("Blocked by an obstacle for more than "
                          << m_obstacleTimeout
                          << " s, aborting the goal.");
          m_actionServer.setAborted();
          stop();
          return false;
        }
      }
      stop();
    }
    else
    {
      m_isBlockedByObstacle = false;
    }
    */
    // Publish command
    m_cmdVelPub.publish(cmd);

    // Publish action feedback (current position robot)
    move_base_msgs::MoveBaseFeedback feedback;
    feedback.base_position = robotPose;
    m_actionServer.publishFeedback(feedback);

    // sleep
    r.sleep();
  } while(std::abs(errDistance) > m_distanceTolerance ||
          std::abs(errHeading) > m_headingTolerance);
  stop();
  ROS_INFO("Arrived");
  return true;
}

void MoveBase::stop()
{
  geometry_msgs::Twist msg;
  msg.linear.x = 0;
  msg.angular.z = 0;
  m_cmdVelPub.publish(msg);
}

geometry_msgs::TransformStamped MoveBase::getRobotTransform()
{
  return m_tfBuffer.lookupTransform(
    m_mapFrame, m_baseFrame, ros::Time{0}, ros::Duration{2.0});
}

geometry_msgs::PoseStamped MoveBase::getRobotPose()
{
  return transformToPose(getRobotTransform());
}

void MoveBase::dynParamCb(MoveBaseConfig& _config, uint32_t _level)
{
  m_linearP = _config.linear_p;
  m_linearI = _config.linear_i;
  m_linearD = _config.linear_d;
  m_angularP = _config.angular_p;
  m_angularI = _config.angular_i;
  m_angularD = _config.angular_d;

  m_distanceTolerance = _config.goal_distance_tolerance;
  m_headingTolerance = _config.goal_heading_tolerance;

  m_laserChecker.setParams(_config.robot_width,
                           _config.laser_distance_threshold,
                           _config.laser_min_index,
                           _config.laser_max_index);
}

float MoveBase::distance(const geometry_msgs::PoseStamped& _p1,
                         const geometry_msgs::PoseStamped& _p2)
{
  return std::hypot(_p2.pose.position.y - _p1.pose.position.y,
                    _p2.pose.position.x - _p1.pose.position.x);
}

float MoveBase::angleSegment(const geometry_msgs::PoseStamped& _p1,
                             const geometry_msgs::PoseStamped& _p2)
{
  return std::atan2(_p2.pose.position.y - _p1.pose.position.y,
                    _p2.pose.position.x - _p1.pose.position.x);
}

geometry_msgs::PoseStamped
MoveBase::transformToPose(const geometry_msgs::TransformStamped& _transform)
{
  geometry_msgs::PoseStamped pose;
  pose.header = _transform.header;
  pose.pose.position.x = _transform.transform.translation.x;
  pose.pose.position.y = _transform.transform.translation.y;
  pose.pose.position.z = _transform.transform.translation.z;
  pose.pose.orientation = _transform.transform.rotation;
  return pose;
}

geometry_msgs::TransformStamped
MoveBase::poseToTransform(const geometry_msgs::PoseStamped& _pose)
{
  geometry_msgs::TransformStamped transform;
  transform.header = _pose.header;
  transform.transform.translation.x = _pose.pose.position.x;
  transform.transform.translation.y = _pose.pose.position.y;
  transform.transform.translation.z = _pose.pose.position.z;
  transform.transform.rotation.x = _pose.pose.orientation.x;
  transform.transform.rotation.y = _pose.pose.orientation.y;
  transform.transform.rotation.z = _pose.pose.orientation.z;
  transform.transform.rotation.w = _pose.pose.orientation.w;
  return transform;
}

btTransform MoveBase::poseToBullet(const geometry_msgs::PoseStamped& _pose)
{
  return btTransform(btQuaternion(_pose.pose.orientation.x,
                                  _pose.pose.orientation.y,
                                  _pose.pose.orientation.z,
                                  _pose.pose.orientation.w),
                     btVector3(_pose.pose.position.x,
                               _pose.pose.position.y,
                               _pose.pose.position.z));
}

std::ostream& operator<<(std::ostream& _out,
                         const geometry_msgs::PoseStamped& _pose)
{
  _out << "(" << _pose.pose.position.x << ", " << _pose.pose.position.y << " | "
       << (tf::getYaw(_pose.pose.orientation) * 180 / M_PI) << ")";
  return _out;
}

} // namespace snd_move_base

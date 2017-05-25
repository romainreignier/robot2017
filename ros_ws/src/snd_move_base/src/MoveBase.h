#pragma once

#include <ostream>
#include <string>

#include <bullet/LinearMath/btTransform.h>
#include <tf2_bullet/tf2_bullet.h>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <sensor_msgs/LaserScan.h>
#include <snd_move_base/MoveBaseConfig.h>
#include <snd_msgs/ProximitySensors.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>

#include "LaserChecker.h"

namespace snd_move_base
{

class MoveBase
{
public:
  static float distance(const geometry_msgs::PoseStamped& _p1,
                        const geometry_msgs::PoseStamped& _p2);
  static float angleSegment(const geometry_msgs::PoseStamped& _p1,
                            const geometry_msgs::PoseStamped& _p2);
  static geometry_msgs::PoseStamped
  transformToPose(const geometry_msgs::TransformStamped& _transform);
  static geometry_msgs::TransformStamped
  poseToTransform(const geometry_msgs::PoseStamped& _pose);
  btTransform poseToBullet(const geometry_msgs::PoseStamped& _pose);

  MoveBase();

private:
  void executeCb(const move_base_msgs::MoveBaseGoalConstPtr& _goal);
  void stop();
  geometry_msgs::TransformStamped getRobotTransform();
  geometry_msgs::PoseStamped getRobotPose();
  void dynParamCb(MoveBaseConfig& _config, uint32_t _level);
  bool move(const geometry_msgs::PoseStamped& _targetPose);

private:
  ros::NodeHandle m_nh;
  ros::Publisher m_cmdVelPub;
  ros::Subscriber m_laserSub;
  ros::Subscriber m_proximitySub;
  LaserChecker m_laserChecker;
  sensor_msgs::LaserScanConstPtr m_laserMsg;
  snd_msgs::ProximitySensorsConstPtr m_proximityMsg;
  actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> m_actionServer;
  tf2_ros::Buffer m_tfBuffer;
  tf2_ros::TransformListener m_tfListener;
  dynamic_reconfigure::Server<MoveBaseConfig> m_dynParamServer;

  bool m_isBlockedByObstacle;
  ros::Time m_timeObstacleDetected;

  float m_updateRate;
  float m_obstacleTimeout;
  float m_linearP;
  float m_linearI;
  float m_linearD;
  float m_angularP;
  float m_angularI;
  float m_angularD;
  float m_distanceTolerance;
  float m_headingTolerance;
  float m_vLinMax;
  float m_vAngMax;

  std::string m_baseFrame;
  std::string m_mapFrame;
};

std::ostream& operator<<(std::ostream& _out,
                         const geometry_msgs::PoseStamped& _pose);
} // namespace snd_move_base

#pragma once

#include <string>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <snd_msgs/Encoders.h>
#include <snd_msgs/ProximitySensors.h>
#include <snd_msgs/Status.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt16.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <bullet/LinearMath/btTransform.h>
#include <tf2_bullet/tf2_bullet.h>

class Match
{
public:
  enum State
  {
    WAITING_STARTER_INSERTED,
    WAITING_STARTER_REMOVED,
    RUNNING,
    LAUNCH_FUNNY,
    FINISHED
  };

  enum MatchState
  {
    START1,
    MOVE1,
    STOP1
  };

  Match();
  void init();

private:
  void timerCb(const ros::TimerEvent&);
  void run();
  void stopMotors() const;
  void moveServo(const ros::Publisher& _pub, int _val);
  bool isObstacleDetected(float _threshold = 0.2);
  geometry_msgs::TransformStamped
  poseToTransform(const geometry_msgs::PoseStamped& _pose);
  geometry_msgs::PoseStamped
  transformToPose(const geometry_msgs::TransformStamped& _transform);
  geometry_msgs::PoseStamped getRobotPose();
  geometry_msgs::TransformStamped getRobotTransform();

private:
  ros::NodeHandle m_nh;
  // Publishers
  ros::Publisher m_cmdVelPub;
  ros::Publisher m_funnyServoPub;
  ros::Publisher m_armServoPub;
  ros::Publisher m_greenLedPub;
  // Subscribers
  ros::Subscriber m_statusSub;
  ros::Subscriber m_lidarSub;
  ros::Subscriber m_encodersSub;
  ros::Subscriber m_sensorsSub;
  // Messages
  snd_msgs::StatusConstPtr m_statusMsg;
  snd_msgs::EncodersConstPtr m_encodersMsg;
  sensor_msgs::LaserScanConstPtr m_laserMsg;
  snd_msgs::ProximitySensorsConstPtr m_sensorsMsg;
  // Time
  ros::Time m_startTime;
  ros::Timer m_timer;
  // tf
  tf2_ros::Buffer m_tfBuffer;
  tf2_ros::TransformListener m_tfListener;
  // Parameters
  int m_funnyServoArmed;
  int m_funnyServoLaunch;
  int m_armServoLow;
  int m_armServoHigh;
  State m_state;
  MatchState m_matchState;
  snd_msgs::Color m_color;
  snd_msgs::EncodersConstPtr m_startPos;
  double m_desiredDistance;
  int m_linearSpeed;
  double m_obstacleThreshold;
  geometry_msgs::TransformStamped m_startTransform;
  bool m_useOAS;
  std::string m_baseFrame;
  std::string m_mapFrame;
};

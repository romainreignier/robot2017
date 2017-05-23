#pragma once

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <snd_msgs/Encoders.h>
#include <snd_msgs/MotorControlMode.h>
#include <snd_msgs/Motors.h>
#include <snd_msgs/Status.h>
#include <std_msgs/UInt16.h>

class Homologation
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

  Homologation();
  void init();

private:
  void timerCb(const ros::TimerEvent&);
  void run();
  void stopMotors() const;
  void moveServo(const ros::Publisher& _pub, int _val);
  bool isObstacleDetected(float _threshold = 0.2);

private:
  ros::NodeHandle m_nh;
  // Publishers
  ros::Publisher m_motorsPub;
  ros::Publisher m_motorsModePub;
  ros::Publisher m_funnyServoPub;
  // Subscribers
  ros::Subscriber m_statusSub;
  ros::Subscriber m_lidarSub;
  ros::Subscriber m_encodersSub;
  // Messages
  snd_msgs::StatusConstPtr m_statusMsg;
  snd_msgs::EncodersConstPtr m_encodersMsg;
  sensor_msgs::LaserScanConstPtr m_laserMsg;
  // Time
  ros::Time m_startTime;
  ros::Timer m_timer;
  // Parameters
  int m_funnyServoArmed;
  int m_funnyServoLaunch;
  State m_state;
  MatchState m_matchState;
  snd_msgs::Color m_color;
  snd_msgs::EncodersConstPtr m_startPos;
  double m_desiredDistance;
  int m_pwm;
  double m_obstacleThreshold;
};

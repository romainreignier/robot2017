#pragma once

#include <string>

#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>
#include <snd_msgs/Encoders.h>
#include <snd_msgs/MoveAngularAction.h>
#include <snd_msgs/MoveAngularStepAction.h>
#include <snd_msgs/MoveLinearAction.h>
#include <snd_msgs/MoveLinearStepAction.h>
#include <snd_robot_control/PidConfig.h>

#include "../../../../firmware/common/RunningAverage/RunningAverage.h"

class PolarControl
{
public:
  PolarControl();
  template <typename T> T bound(T _in, T _min, T _max);
  int16_t boundPwm(int16_t _pwm);

protected:
  // Subscriber callback
  void encodersCb(const snd_msgs::EncodersConstPtr& _msg);

  // Dynamic reconfigure callback
  void dynParamCb(const snd_robot_control::PidConfig& _cfg, uint32_t);

  // Action servers callbacks
  void moveLinearActionCb();
  void moveLinearPreemptCb();
  void moveAngularActionCb();
  void moveAngularPreemptCb();
  void moveLinearStepActionCb();
  void moveLinearStepPreemptCb();
  void moveAngularStepActionCb();
  void moveAngularStepPreemptCb();

  // helpers
  void abortCurrentAction(const std::string& _reason);
  void succeedCurrentAction(float _distError, float _angError);
  void resetInternalVariables();
  void stop();

  // Control methods
  void controlLoop(const ros::TimerEvent&);
  void computeTraj();
  void computeTrajWithAcc();
  void asserv(const int32_t& _dLeft, const int32_t& _dRight);
  void lectureCodeur(int32_t& _dLeft, int32_t& _dRight);

protected:
  ros::NodeHandle m_nh;
  ros::Timer m_controlLoopTimer;

  ros::Publisher m_cmdPub;
  ros::Subscriber m_encodersSub;
  ros::Subscriber m_odomSub;

  snd_msgs::EncodersConstPtr m_lastEncoders;
  snd_msgs::EncodersConstPtr m_previousEncoders;

  dynamic_reconfigure::Server<snd_robot_control::PidConfig> m_dynParamServer;

  actionlib::SimpleActionServer<snd_msgs::MoveLinearAction> m_moveLinearAs;
  actionlib::SimpleActionServer<snd_msgs::MoveAngularAction> m_moveAngularAs;
  actionlib::SimpleActionServer<snd_msgs::MoveLinearStepAction>
    m_moveLinearStepAs;
  actionlib::SimpleActionServer<snd_msgs::MoveAngularStepAction>
    m_moveAngularStepAs;

  bool m_useAccel = false;

  // Variables from STM32 code
  uint32_t kPidTimerPeriodMs = 10;
  float cibleDistance;
  float cibleAngle;
  int16_t maxPwm;
  float consigneDistance;
  float consigneAngle;
  float mesureDistance;
  float mesureAngle;
  float erreurDistance;
  float erreurAngle;
  float lastErreurDistance;
  float lastErreurAngle;
  float toleranceDistance;
  float toleranceAngle;
  int16_t cptRestOnPosition;
  int16_t cptRestOnAngle;
  int16_t compensationDist;
  int16_t compensationAng;
  float kpDist;
  float kpAng;
  float kiDist;
  float kiAng;
  float kdDist;
  float kdAng;
  float iTermDist;
  float iTermAng;
  float iMinDist;
  float iMaxDist;
  float iMinAng;
  float iMaxAng;
  uint32_t finAsservIterations;
  bool finish;
  bool mustComputeTraj;
#if CMD_PWM
  int16_t leftPwm;
  int16_t rightPwm;
#else
  float leftPwm;
  float rightPwm;
#endif
  float leftSpeed;
  float rightSpeed;
  RunningAverage<int32_t, 2> leftQeiAvg;
  RunningAverage<int32_t, 2> rightQeiAvg;
  int32_t leftQeiCnt = 0;
  int32_t rightQeiCnt = 0;

  float currentLinearVelocity = 0;
  float currentAngularVelocity = 0;

  float vLinMax;
  float accLinMax;
  float vAngMax;
  float accAngMax;
  float outputMax;

  float smoothRotation;
  float linear_speed;

  static constexpr float kPi = 3.14159265358979323846f;
  int32_t encoderResolution;
  float wheelSeparationMM;
  float leftWheelRadius;
  float rightWheelRadius;
  float LEFT_TICKS_TO_MM;
  float RIGHT_TICKS_TO_MM;
};

template <typename T> T PolarControl::bound(T _in, T _min, T _max)
{
  if(_in > _max) return _max;
  if(_in < _min) return _min;
  return _in;
}

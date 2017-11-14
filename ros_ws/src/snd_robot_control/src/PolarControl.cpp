#include "PolarControl.h"

#include <angles/angles.h>

#include "snd_msgs/Motors.h"

PolarControl::PolarControl()
  : m_nh{"~"}, m_moveLinearAs{m_nh, "move_linear", false},
    m_moveAngularAs{m_nh, "move_angular", false},
    m_moveLinearStepAs{m_nh, "move_linear_step", false},
    m_moveAngularStepAs{m_nh, "move_angular_step", false}
{
  // Get parameters
  const double controlLoopPeriodMs = m_nh.param("control_loop_period_ms", 10);
  encoderResolution = m_nh.param("encoders_resolution", 2400);
  kPidTimerPeriodMs = static_cast<uint32_t>(controlLoopPeriodMs);
  wheelSeparationMM = m_nh.param("wheels_separation_mm", 230.0);
  leftWheelRadius = m_nh.param("left_wheel_radius_mm", 35.5);
  rightWheelRadius = m_nh.param("right_wheel_radius_mm", 35.5);

  LEFT_TICKS_TO_MM = (2 * kPi * leftWheelRadius) / encoderResolution;
  RIGHT_TICKS_TO_MM = (2 * kPi * rightWheelRadius) / encoderResolution;

  // Init publishers
  m_cmdPub = m_nh.advertise<snd_msgs::Motors>("/motors", 10);

  // Init subscribers
  // TODO tcpNoDelay
  m_encodersSub =
    m_nh.subscribe("/encoders", 1, &PolarControl::encodersCb, this);

  // Init Dynamic Reconfigure Server
  m_dynParamServer.setCallback(
    boost::bind(&PolarControl::dynParamCb, this, _1, _2));

  // Init MoveLinear Action Server
  m_moveLinearAs.registerGoalCallback(
    boost::bind(&PolarControl::moveLinearActionCb, this));
  m_moveLinearAs.registerPreemptCallback(
    boost::bind(&PolarControl::moveLinearPreemptCb, this));
  m_moveLinearAs.start();

  // Init MoveAngular Action Server
  m_moveAngularAs.registerGoalCallback(
    boost::bind(&PolarControl::moveAngularActionCb, this));
  m_moveAngularAs.registerPreemptCallback(
    boost::bind(&PolarControl::moveAngularPreemptCb, this));
  m_moveAngularAs.start();

  // Init MoveLinearStep Action Server
  m_moveLinearStepAs.registerGoalCallback(
    boost::bind(&PolarControl::moveLinearStepActionCb, this));
  m_moveLinearStepAs.registerPreemptCallback(
    boost::bind(&PolarControl::moveLinearStepPreemptCb, this));
  m_moveLinearStepAs.start();

  // Init MoveAngularStep Action Server
  m_moveAngularStepAs.registerGoalCallback(
    boost::bind(&PolarControl::moveAngularStepActionCb, this));
  m_moveAngularStepAs.registerPreemptCallback(
    boost::bind(&PolarControl::moveAngularStepPreemptCb, this));
  m_moveAngularStepAs.start();

  // Init control loop Timer
  m_controlLoopTimer =
    m_nh.createTimer(ros::Duration{controlLoopPeriodMs / 1000.0},
                     &PolarControl::controlLoop,
                     this);

  resetInternalVariables();
  stop();

  ROS_INFO("polar_control_node ready");
}

void PolarControl::encodersCb(const snd_msgs::EncodersConstPtr& _msg)
{
  m_lastEncoders = _msg;
  if(!m_previousEncoders)
  {
    m_previousEncoders = _msg;
  }
}

void PolarControl::dynParamCb(const snd_robot_control::PidConfig& _cfg,
                              uint32_t)
{
  kpDist = _cfg.distance_p;
  kiDist = _cfg.distance_i;
  kdDist = _cfg.distance_d;
  iMinDist = -_cfg.distance_max_i_term;
  iMaxDist = _cfg.distance_max_i_term;
  toleranceDistance = _cfg.distance_tolerance;
  vLinMax = _cfg.distance_max_speed * kPidTimerPeriodMs / 1000.0;

  kpAng = _cfg.angular_p;
  kiAng = _cfg.angular_i;
  kdAng = _cfg.angular_d;
  iMinAng = -_cfg.angular_max_i_term;
  iMaxAng = _cfg.angular_max_i_term;
  toleranceAngle = _cfg.angular_tolerance;
  // rad/s
  vAngMax = _cfg.angular_max_speed * kPidTimerPeriodMs / 1000.0;

  outputMax = _cfg.max_wheel_output;
}

void PolarControl::controlLoop(const ros::TimerEvent&)
{
  int32_t dLeft;
  int32_t dRight;
  lectureCodeur(dLeft, dRight);

  // Appele a 20Hz
  if(mustComputeTraj)
  {
    computeTraj();
  }
  if(!finish)
  {
    asserv(dLeft, dRight);
  }
}

void PolarControl::computeTraj()
{
  // Increment distance
  if(cibleDistance >= 0)
  {
    if(consigneDistance < cibleDistance)
    {
      consigneDistance += vLinMax;
    }
    else
    {
      consigneDistance = cibleDistance;
    }
  }
  else
  {
    if(consigneDistance > cibleDistance)
    {
      consigneDistance -= vLinMax;
    }
    else
    {
      consigneDistance = cibleDistance;
    }
  }
  // Increment Angle
  if(cibleAngle >= 0)
  {
    if(consigneAngle < cibleAngle)
    {
      consigneAngle += vAngMax;
    }
    else
    {
      consigneAngle = cibleAngle;
    }
  }
  else
  {
    if(consigneAngle > cibleAngle)
    {
      consigneAngle -= vAngMax;
    }
    else
    {
      consigneAngle = cibleAngle;
    }
  }
}

void PolarControl::asserv(const int32_t& _dLeft, const int32_t& _dRight)
{
  float correctionDistance = 0.0;
  float correctionAngle = 0.0;

  // Estimation deplacement
  const float dl = static_cast<float>(_dLeft) *
                   LEFT_TICKS_TO_MM; // calcul du déplacement de la roue droite
  const float dr = static_cast<float>(_dRight) *
                   RIGHT_TICKS_TO_MM; // calcul du déplacement de la roue gauche

  // calcul du déplacement du robot
  const float dD = (dr + dl) / 2;
  // calcul de la variation de l'angle alpha du robot
  const float dA = (dr - dl) / wheelSeparationMM;

  // Incrementation des mesures
  // linear_speed = ((dD / kPidTimerPeriodMs) * 1000 );
  mesureDistance += dD;
  mesureAngle += dA;
  // mesureAngle = angles::normalize_angle(mesureAngle);

  // Calcul des erreurs
  erreurDistance = consigneDistance - mesureDistance;
  erreurAngle = angles::normalize_angle(consigneAngle - mesureAngle);

  iTermAng += kiAng * erreurAngle;
  iTermAng = bound(iTermAng, iMinAng, iMaxAng);

#if CMD_PWM
  if(erreurDistance >= 0.0f)
    correctionDistance = (kpDist * erreurDistance +
                          (kdDist * (erreurDistance - lastErreurDistance))) +
                         700.0f + compensationDist;
  else
    correctionDistance = (kpDist * erreurDistance +
                          (kdDist * (erreurDistance - lastErreurDistance))) -
                         700.0f + compensationDist;

  if(erreurAngle >= 0.0f)
    correctionAngle = kpAng * erreurAngle +
                      kdAng * (erreurAngle - lastErreurAngle) + iTermAng +
                      750.0f + compensationAng;
  else
    correctionAngle = kpAng * erreurAngle +
                      kdAng * (erreurAngle - lastErreurAngle) + iTermAng -
                      750.0f + compensationAng;

  if(std::fabs(erreurDistance - lastErreurDistance) < 1e-6f)
  {
    ++cptRestOnPosition;
    iTermDist += kiDist * erreurDistance;
    iTermDist = bound(iTermDist, iMinDist, iMaxDist);
  }
  else
    compensationDist = 0;

  if(cptRestOnPosition > 3) compensationDist = iTermDist;

  if(std::fabs(erreurAngle - lastErreurAngle) < 1e-6f)
  {
    ++cptRestOnAngle;
    iTermAng += kiAng * erreurAngle;
    iTermAng = bound(iTermAng, iMinAng, iMaxAng);
  }
  else
    compensationAng = 0;

  if(cptRestOnPosition > 3) compensationAng = iTermAng;

  leftPwm = boundPwm(static_cast<int16_t>(correctionDistance -
                                          smoothRotation * correctionAngle));
  rightPwm = boundPwm(static_cast<int16_t>(correctionDistance +
                                           smoothRotation * correctionAngle));
#else
  iTermDist += kiDist * erreurDistance;
  iTermDist = bound(iTermDist, iMinDist, iMaxDist);

  correctionDistance = kpDist * erreurDistance +
                       kdDist * (erreurDistance - lastErreurDistance) +
                       iTermDist;
  correctionAngle =
    kpAng * erreurAngle + kdAng * (erreurAngle - lastErreurAngle) + iTermAng;

  leftPwm = bound(correctionDistance - correctionAngle, -outputMax, outputMax);
  rightPwm = bound(correctionDistance + correctionAngle, -outputMax, outputMax);
#endif

  lastErreurDistance = erreurDistance;
  lastErreurAngle = erreurAngle;

  snd_msgs::Motors cmdMsg;
  cmdMsg.header.stamp = ros::Time::now();
  cmdMsg.left = leftPwm;
  cmdMsg.right = rightPwm;

  // std::cout << "mesure distance: " << mesureDistance
  //           << " mesure angle: " << mesureAngle
  //           << " erreur distance: " << erreurDistance
  //           << " correction distance: " << correctionDistance
  //           << " erreur angle: " << erreurAngle
  //           << " correction angle: " << correctionAngle
  //           << " out left: " << leftPwm << " out right: " << rightPwm << '\n';

  if(std::fabs(erreurDistance) < toleranceDistance &&
     std::fabs(erreurAngle) < toleranceAngle)
  {
    finAsservIterations++;
    if(finAsservIterations > 50)
    {
      finish = true;
      cmdMsg.left = 0;
      cmdMsg.right = 0;
      succeedCurrentAction(erreurDistance, erreurAngle);
    }
  }
  m_cmdPub.publish(cmdMsg);
}

void PolarControl::lectureCodeur(int32_t& _dLeft, int32_t& _dRight)
{
  if(m_lastEncoders)
  {
    // Retrieve the values from a locked system
    _dLeft = m_lastEncoders->left - m_previousEncoders->left;
    _dRight = m_lastEncoders->right - m_previousEncoders->right;
    m_previousEncoders = m_lastEncoders;
  }
  else
  {
    _dLeft = 0;
    _dRight = 0;
  }

  // Increment the internal counters
  leftQeiCnt += _dLeft;
  rightQeiCnt += _dRight;

  // Compute the average
  leftQeiAvg.add(_dLeft);
  rightQeiAvg.add(_dRight);

  // Speeds in ticks/s
  leftSpeed = (leftQeiAvg.getAverage() * 1000.0f) / (kPidTimerPeriodMs);
  rightSpeed = (rightQeiAvg.getAverage() * 1000.0f) / (kPidTimerPeriodMs);
  linear_speed = ((leftSpeed + rightSpeed) / 2);
  smoothRotation =
    bound((-(0.9f / 1500.0f) * std::fabs(linear_speed) + 1.0f), 0.05f, 1.0f);
}

void PolarControl::moveLinearActionCb()
{
  abortCurrentAction("New MoveLinear Action received");

  ROS_INFO("MoveLinear Action received.");
  const auto goal = m_moveLinearAs.acceptNewGoal();

  if(m_moveLinearAs.isPreemptRequested())
  {
    ROS_WARN("MoveLinear action already preempted");
    m_moveLinearAs.setPreempted(snd_msgs::MoveLinearResult{},
                                "already preempted");
  }

  cibleAngle = 0;
  cibleDistance = goal->distance;
  mustComputeTraj = true;
  resetInternalVariables();
}

void PolarControl::moveLinearPreemptCb()
{
  ROS_INFO("MoveLinear Action preemption request");
  m_moveLinearAs.setPreempted(snd_msgs::MoveLinearResult{}, "Preempt request");
  stop();
}

void PolarControl::moveAngularActionCb()
{
  abortCurrentAction("New MoveAngular Action received");

  ROS_INFO("MoveAngular Action received.");
  const auto goal = m_moveAngularAs.acceptNewGoal();

  if(m_moveAngularAs.isPreemptRequested())
  {
    ROS_WARN("MoveAngular action already preempted");
    m_moveAngularAs.setPreempted(snd_msgs::MoveAngularResult{},
                                 "already preempted");
  }

  cibleAngle = angles::from_degrees(goal->angle);
  cibleDistance = 0;
  mustComputeTraj = true;
  resetInternalVariables();
}

void PolarControl::moveAngularPreemptCb()
{
  ROS_INFO("MoveAngular Action preemption request");
  m_moveAngularAs.setPreempted(snd_msgs::MoveAngularResult{},
                               "Preempt request");
  stop();
}

void PolarControl::moveLinearStepActionCb()
{
  abortCurrentAction("New MoveLinearStep Action received");

  ROS_INFO("MoveLinearStep Action received.");
  const auto goal = m_moveLinearStepAs.acceptNewGoal();

  if(m_moveLinearStepAs.isPreemptRequested())
  {
    ROS_WARN("MoveLinearStep action already preempted");
    m_moveLinearStepAs.setPreempted(snd_msgs::MoveLinearStepResult{},
                                    "already preempted");
  }

  cibleAngle = 0;
  cibleDistance = goal->distance;
  mustComputeTraj = false;
  resetInternalVariables();
}

void PolarControl::moveLinearStepPreemptCb()
{
  ROS_INFO("MoveLinearSte Action preemption request");
  m_moveLinearStepAs.setPreempted(snd_msgs::MoveLinearStepResult{},
                                  "Preempt request");
  stop();
}

void PolarControl::moveAngularStepActionCb()
{
  abortCurrentAction("New MoveAngularStep Action received");

  ROS_INFO("MoveAngularStep Action received.");
  const auto goal = m_moveAngularStepAs.acceptNewGoal();

  if(m_moveAngularStepAs.isPreemptRequested())
  {
    ROS_WARN("MoveAngularStep action already preempted");
    m_moveAngularStepAs.setPreempted(snd_msgs::MoveAngularStepResult{},
                                     "already preempted");
  }

  cibleAngle = angles::from_degrees(goal->angle);
  cibleDistance = 0;
  mustComputeTraj = false;
  resetInternalVariables();
}

void PolarControl::moveAngularStepPreemptCb()
{
  ROS_INFO("MoveAngularStep Action preemption request");
  m_moveAngularStepAs.setPreempted(snd_msgs::MoveAngularStepResult{},
                                   "Preempt request");
  stop();
}

void PolarControl::abortCurrentAction(const std::string& _reason)
{
  if(m_moveLinearAs.isActive())
  {
    ROS_WARN_STREAM("Abort MoveLinear Action: " << _reason);
    m_moveLinearAs.setAborted(snd_msgs::MoveLinearResult{}, _reason);
  }
  else if(m_moveAngularAs.isActive())
  {
    ROS_WARN_STREAM("Abort MoveAngular Action: " << _reason);
    m_moveAngularAs.setAborted(snd_msgs::MoveAngularResult{}, _reason);
  }
  else if(m_moveLinearStepAs.isActive())
  {
    ROS_WARN_STREAM("Abort MoveLinearStep Action: " << _reason);
    m_moveLinearStepAs.setAborted(snd_msgs::MoveLinearStepResult{}, _reason);
  }
  else if(m_moveAngularStepAs.isActive())
  {
    ROS_WARN_STREAM("Abort MoveAngularStep Action: " << _reason);
    m_moveAngularStepAs.setAborted(snd_msgs::MoveAngularStepResult{}, _reason);
  }
}

void PolarControl::succeedCurrentAction(float _distError, float _angError)
{
  std::stringstream ss;
  ss << "distance error: " << _distError
     << " mm angular error: " << angles::to_degrees(_angError) << " deg";

  if(m_moveLinearAs.isActive())
  {
    ROS_INFO_STREAM("Finish MoveLinear Action: " << ss.str());
    m_moveLinearAs.setSucceeded(snd_msgs::MoveLinearResult{}, ss.str());
  }
  else if(m_moveAngularAs.isActive())
  {
    ROS_INFO_STREAM("Finish MoveAngular Action: " << ss.str());
    m_moveAngularAs.setSucceeded(snd_msgs::MoveAngularResult{}, ss.str());
  }
  else if(m_moveLinearStepAs.isActive())
  {
    ROS_INFO_STREAM("Finish MoveLinearStep Action: " << ss.str());
    m_moveLinearStepAs.setSucceeded(snd_msgs::MoveLinearStepResult{}, ss.str());
  }
  else if(m_moveAngularStepAs.isActive())
  {
    ROS_INFO_STREAM("Finish MoveAngularStep Action: " << ss.str());
    m_moveAngularStepAs.setSucceeded(snd_msgs::MoveAngularStepResult{},
                                     ss.str());
  }
}

void PolarControl::resetInternalVariables()
{
  consigneDistance = cibleDistance;
  consigneAngle = cibleAngle;
  mesureDistance = 0;
  mesureAngle = 0;
  finAsservIterations = 0;
  iTermDist = 0.0;
  iTermAng = 0.0;
  finish = false;
  compensationDist = 0;
  cptRestOnPosition = 0;
  cptRestOnAngle = 0;
  erreurDistance = 0;
  lastErreurDistance = 0;
}

void PolarControl::stop()
{
  finish = true;
  mustComputeTraj = false;
  m_cmdPub.publish(snd_msgs::Motors{});
}

int16_t PolarControl::boundPwm(int16_t _pwm)
{
  if(_pwm > maxPwm) return maxPwm;
  if(_pwm < -maxPwm) return -maxPwm;
  return _pwm;
}

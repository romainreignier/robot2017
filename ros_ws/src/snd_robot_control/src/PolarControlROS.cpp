#include "PolarControlROS.h"

#include <tf2/utils.h>

#include "libPolarControl/angles.h"

PolarControlROS::PolarControlROS() : m_nh{"~"}, m_moveAs{m_nh, "move", false}
{
  // Params
  const double updateRate = m_nh.param("update_rate", 50);

  // Init Publisher
  m_cmdPub = m_nh.advertise<snd_msgs::Motors>("/motors", 10);

  // Init Subscriber
  // TODO tcpNoDelay
  m_odomSub = m_nh.subscribe("/odom", 1, &PolarControlROS::odomCb, this);

  // Init Dynamic Reconfigure Server
  m_dynParamServer.setCallback(
    boost::bind(&PolarControlROS::dynParamCb, this, _1, _2));

  // Init MoveLinear Action Server
  m_moveAs.registerGoalCallback(
    boost::bind(&PolarControlROS::moveActionCb, this));
  m_moveAs.registerPreemptCallback(
    boost::bind(&PolarControlROS::movePreemptCb, this));
  m_moveAs.start();

  // Init Timer
  m_updateTimer =
    m_nh.createTimer(ros::Rate{updateRate}, &PolarControlROS::update, this);

  ROS_INFO("PolarControlROS Node launched");
}

void PolarControlROS::odomCb(const nav_msgs::OdometryConstPtr& _msg)
{
  m_lastOdom = _msg;
}

void PolarControlROS::dynParamCb(
  const snd_robot_control::PolarControlConfig& _cfg, uint32_t)
{
  m_control.getLinearComponent().setTunings(_cfg.distance_p,
                                            _cfg.distance_i,
                                            _cfg.distance_d,
                                            _cfg.distance_max_i_term);

  m_control.getAngularComponent().setTunings(
    _cfg.angular_p, _cfg.angular_i, _cfg.angular_d, _cfg.angular_max_i_term);

  m_control.getLinearComponent().init(_cfg.distance_max_speed,
                                      _cfg.distance_max_acceleration,
                                      _cfg.distance_tolerance);

  m_control.getAngularComponent().init(_cfg.angular_max_speed,
                                       _cfg.angular_max_acceleration,
                                       _cfg.angular_tolerance);
}

void PolarControlROS::moveActionCb()
{
  const auto goal = m_moveAs.acceptNewGoal();
  ROS_INFO_STREAM(
    "New Move Action received to (" << goal->x << ", " << goal->y << " | "
                                    << angles::to_degrees(goal->theta)
                                    << ")");

  if(m_moveAs.isPreemptRequested())
  {
    ROS_WARN("Action Move already preempted");
    m_moveAs.setPreempted(snd_msgs::MoveToResult{}, "Action already preempted");
  }

  m_control.setTargetPose({goal->x, goal->y, goal->theta});
}

void PolarControlROS::movePreemptCb()
{
  ROS_WARN("Action preempted");
  // Set current position as target
  // TODO see if it is a good idea
  m_control.setTargetPose(getCurrentPose());
  m_moveAs.setPreempted(snd_msgs::MoveToResult{}, "Action preempted");
}

void PolarControlROS::update(const ros::TimerEvent&)
{
  snd_msgs::Motors msg;
  if(!m_lastOdom)
  {
    ROS_WARN_THROTTLE(1, "Odom not yet received so cannot control anything");
  }
  else
  {

    const auto& cmd = m_control.computeMotorsCommands(getCurrentPose());

    if(m_control.isGoalReached())
    {
      m_moveAs.setSucceeded(snd_msgs::MoveToResult{}, "Goal Reached");
    }

    msg.left = std::get<PolarControl::LEFT>(cmd);
    msg.right = std::get<PolarControl::RIGHT>(cmd);
  }
  msg.header.stamp = ros::Time::now();
  m_cmdPub.publish(msg);
}

Pose PolarControlROS::getCurrentPose() const
{
  Pose p;
  if(m_lastOdom)
  {
    p.x = m_lastOdom->pose.pose.position.x;
    p.y = m_lastOdom->pose.pose.position.y;
    p.theta = tf2::getYaw(m_lastOdom->pose.pose.orientation);
  }
  return p;
}

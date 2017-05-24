#include "homologation.h"

Homologation::Homologation()
  : m_nh{"~"}, m_state{WAITING_STARTER_INSERTED}, m_matchState{START1}
{
  // Parameters
  double updateRate = m_nh.param("update_rate", 20);
  m_desiredDistance = m_nh.param("distance", 1.0);
  m_pwm = m_nh.param("pwm", 1500);
  m_obstacleThreshold = m_nh.param("threshold", 0.4);
  ROS_INFO_STREAM("\nupdate_rate: " << updateRate << "\ndistance: "
                                    << m_desiredDistance
                                    << "\npwm: "
                                    << m_pwm
                                    << "\nthreshold: "
                                    << m_obstacleThreshold);
  // Get servo position
  ros::NodeHandle controlNh("snd_control");
  m_funnyServoArmed = controlNh.param("launch_servo/armed_value", 270);
  m_funnyServoLaunch = controlNh.param("launch_servo/launch_value", 110);
  m_armServoHigh = controlNh.param("arm_servo/high_value", 208);
  m_armServoLow = controlNh.param("arm_servo/low_value", 385);

  // Publishers
  m_funnyServoPub = m_nh.advertise<std_msgs::UInt16>("/launch_servo", 1);
  m_armServoPub = m_nh.advertise<std_msgs::UInt16>("/arm_servo", 1);
  m_motorsPub = m_nh.advertise<snd_msgs::Motors>("/motors_speed", 1);
  m_motorsModePub =
    m_nh.advertise<snd_msgs::MotorControlMode>("/motors_mode", 1);

  // Subscribers
  m_statusSub = m_nh.subscribe(
    "/status",
    1,
    boost::function<void(const snd_msgs::StatusConstPtr&)>(
      [&](const snd_msgs::StatusConstPtr _msg) { m_statusMsg = _msg; }));
  m_lidarSub = m_nh.subscribe(
    "/scan",
    1,
    boost::function<void(const sensor_msgs::LaserScanConstPtr&)>(
      [&](const sensor_msgs::LaserScanConstPtr _msg) { m_laserMsg = _msg; }));

  m_encodersSub = m_nh.subscribe(
    "/encoders",
    1,
    boost::function<void(const snd_msgs::EncodersConstPtr&)>(
      [&](const snd_msgs::EncodersConstPtr _msg) { m_encodersMsg = _msg; }));

  m_sensorsSub = m_nh.subscribe(
    "/proximity_sensors",
    1,
    boost::function<void(const snd_msgs::ProximitySensorsConstPtr&)>(
      [&](const snd_msgs::ProximitySensorsConstPtr _msg) { m_sensorsMsg = _msg; }));

  // Timer
  m_timer = m_nh.createTimer(
    ros::Duration(1 / updateRate), &Homologation::timerCb, this, false, false);

  // Sleep 1 sec to let the Publishers to come up
  ros::Duration(1).sleep();
}

void Homologation::init()
{
  // Make sure the timer is not running
  m_timer.stop();
  // Reset the state
  m_state = WAITING_STARTER_INSERTED;
  m_matchState = START1;

  ROS_INFO("Publish motors mode");
  // Set the motors in open-loop mode
  snd_msgs::MotorControlMode modeMsg;
  modeMsg.mode = snd_msgs::MotorControlMode::PWM;
  m_motorsModePub.publish(modeMsg);

  // Stop the motors, just to be sure
  stopMotors();

  // Lift the Arm to avoid any problem
  ROS_INFO("Lift arm");
  moveServo(m_armServoPub, m_armServoHigh);

  // Set the funny servo in ARMED position
  moveServo(m_funnyServoPub, m_funnyServoArmed);

  // Disable the Arm servo
  // moveServo(m_armServoPub, 0);
  // Start the timer
  m_timer.start();
}

void Homologation::timerCb(const ros::TimerEvent&)
{
  if(m_statusMsg && m_statusMsg->starter && m_statusMsg->eStop)
  {
    // if starter AND eStop -> reset
    ROS_INFO("starter + eStop = init");
    init();
    return;
  }
  switch(m_state)
  {
  case WAITING_STARTER_INSERTED:
    if(m_statusMsg && !m_statusMsg->starter)
    {
      ROS_INFO_THROTTLE(1, "Waiting for the starter to be inserted");
    }
    else if(m_statusMsg && m_statusMsg->starter)
    {
      ROS_INFO("Starter is inserted -> WAITING_STARTER_REMOVED");
      m_state = WAITING_STARTER_REMOVED;
    }
    break;
  case WAITING_STARTER_REMOVED:
    if(m_statusMsg && !m_statusMsg->starter)
    {
      ROS_INFO("Starter removed -> RUNNING");
      m_startTime = ros::Time::now();
      m_color = m_statusMsg->color_switch;
      m_state = RUNNING;
    }
    break;
  case RUNNING:
    if(ros::Time::now() - m_startTime > ros::Duration(90))
    {
      ROS_INFO("90sec! -> LAUNCH_FUNNY");
      stopMotors();
      m_state = LAUNCH_FUNNY;
    }
    else
    {
      run();
    }
    break;
  case LAUNCH_FUNNY:
  {
    // Note: if the robot was moving, add a delay to be sure it is stopped while
    // doing the Funny Action
    stopMotors();
    moveServo(m_funnyServoPub, m_funnyServoLaunch);
    ROS_INFO("FunnyAction launched -> FINISHED");
    m_state = FINISHED;
    break;
  }
  case FINISHED:
    // Do nothing
    if(isObstacleDetected(m_obstacleThreshold))
    {
      ROS_WARN("Obstacle");
      stopMotors();
    }
    break;
  }
}

void Homologation::run()
{
  // fonction called periodically by the timer during the match (between starter
  // removed and Funny Action)
  switch(m_matchState)
  {
  case START1:
    if(m_encodersMsg)
    {
      ROS_INFO("run: -> MOVE1");
      m_startPos = m_encodersMsg;
      m_matchState = MOVE1;
    }
    else
    {
      ROS_WARN("No encoder msg :(");
    }
    break;
  case MOVE1:
  {
    const int encoderResolution = 2400;
    const float wheelRadius = 0.026;
    const int desiredTicks = static_cast<int>(
      m_desiredDistance / (2 * M_PI * wheelRadius) * encoderResolution);
    if((m_encodersMsg->left - m_startPos->left) < desiredTicks &&
       (m_encodersMsg->right - m_startPos->right) < desiredTicks)
    {
      if(isObstacleDetected(m_obstacleThreshold))
      {
        ROS_WARN("Obstacle");
        stopMotors();
      }
      else
      {
        snd_msgs::Motors msg;
        msg.left = m_pwm;
        msg.right = m_pwm;
        m_motorsPub.publish(msg);
      }
    }
    else
    {
      ROS_INFO("run: MOVE1 -> STOP1");
      m_matchState = STOP1;
    }
    break;
  }
  case STOP1:
    if(isObstacleDetected(m_obstacleThreshold))
    {
      ROS_WARN("Obstacle");
      stopMotors();
    }
    break;
  }
}

void Homologation::stopMotors() const
{
  snd_msgs::Motors motorsMsg;
  motorsMsg.left = 0;
  motorsMsg.right = 0;
  m_motorsPub.publish(motorsMsg);
}

void Homologation::moveServo(const ros::Publisher& _pub, int _val)
{
  std_msgs::UInt16 msg;
  msg.data = _val;
  _pub.publish(msg);
}

bool Homologation::isObstacleDetected(float _threshold)
{
  if(!m_laserMsg)
  {
    ROS_WARN("No laser message recived");
    // If there is no laser message, we are blind so don't move
    return true;
  }
  if(m_sensorsMsg)
  {
    // If proximity sensor detect something, no need to check the lidar...
    if(m_sensorsMsg->front)
    {
      return true;
    }
  }
  // TODO: only loop on the index in front of the robot
  for(size_t i = 50; i < 150; ++i)
  {
    const float& d = m_laserMsg->ranges[i];
    if(d < _threshold)
    {
      return true;
    }
  }
  return false;
}

int main(int _argc, char** _argv)
{
  ros::init(_argc, _argv, "homologation_node");
  ROS_INFO("Homologation node launched");
  Homologation h;
  h.init();
  ros::spin();
  return 0;
}

#include "PolarControl.h"


PolarControl::PolarControl()
{
}

void PolarControl::setMaxOutput(float _max)
{
  m_maxOutput = _max;
}

void PolarControl::setTargetPose(const Pose& _target, const Pose& _current)
{
  m_targetPose = _target;
  m_state = State::ROTATION_ALONG_PATH;
  m_isGoalReached = false;
  m_currentDesiredPose = _current;
  m_currentActualPose = _current;
  m_linear.reset();
  m_angular.reset();
  m_angular.setActualCommand(_current.theta);
  m_angular.setDesiredCommand(_current.theta);
}

std::pair<float, float>
PolarControl::computeMotorsCommands(const Pose& _current, float _dt)
{
  float linearError = 0;
  float angularError = 0;

  // If angular error < threshold, compute linear component also
  if(std::abs(m_targetPose.theta - m_angular.getDesiredCommand()) < m_endAngularMovementThreshold)
  {
      m_linear.computeDesiredCommand();
  }

  // Compute the errors
  /*
  switch(m_state)
  {
  case State::ROTATION_ALONG_PATH:
  {
    linearError = 0;
    const float pathAngle = angle(m_targetPose, _currentPose);
    angularError =
      angles::shortest_angular_distance(_currentPose.theta, pathAngle);
    break;
  }
  case State::TRANSLATION_ALONG_PATH:
  {
    linearError = distance(m_targetPose, _currentPose);
    angularError = angle(m_targetPose, _currentPose);
    break;
  }
  case State::ROTATION_ALONG_TARGET:
  {
    // NOTE: see if is better to set 0 or actual error
    linearError = distance(m_targetPose, _currentPose);
    angularError =
      angles::shortest_angular_distance(_currentPose.theta, m_targetPose.theta);
    break;
  }
  }
  */


  // Check the errors
  if(std::abs(linearError) < m_linear.getTolerance() &&
     std::abs(angularError) < m_angular.getTolerance())
  {
    m_endControlIterations++;
    if(m_endControlIterations >= m_numberIterationsEndControl)
    {
      switch(m_state)
      {
      case State::ROTATION_ALONG_PATH:
        m_state = State::TRANSLATION_ALONG_PATH;
        break;
      case State::TRANSLATION_ALONG_PATH:
        m_state = State::ROTATION_ALONG_TARGET;
        break;
      case State::ROTATION_ALONG_TARGET:
        m_isGoalReached = true;
        // Reset everything for the next step
        m_linear.reset();
        m_angular.reset();
        linearError = 0;
        angularError = 0;
        m_endControlIterations = 0;
        break;
      }
    }
  }

  // Compute the commands
  const float linearCommand = m_linear.computeCommand(linearError, _dt);
  const float angularCommand = m_angular.computeCommand(angularError, _dt);

  // Compute the commands per motors
  float leftCommand = linearCommand - angularCommand;
  float rightCommand = linearCommand + angularCommand;

  // Bound the motors commands
  leftCommand = bound(leftCommand, -m_maxOutput, m_maxOutput);
  rightCommand = bound(rightCommand, -m_maxOutput, m_maxOutput);

  // Return the commands
  return {leftCommand, rightCommand};
}

float PolarControl::bound(float _in, float _min, float _max)
{
  if(_in > _max) return _max;
  if(_in < _min) return _min;
  return _in;
}

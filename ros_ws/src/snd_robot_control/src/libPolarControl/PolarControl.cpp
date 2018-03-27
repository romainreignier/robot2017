#include "PolarControl.h"

#include "angles.h"

PolarControl::PolarControl()
{
}

void PolarControl::setMaxOutput(float _max)
{
  m_maxOutput = _max;
}

void PolarControl::setTargetPose(const Pose& _target)
{
  m_targetPose = _target;
  m_state = State::ROTATION_ALONG_PATH;
  m_linear.reset();
  m_angular.reset();
}

std::pair<float, float>
PolarControl::computeMotorsCommands(const Pose& _currentPose)
{
  float linearError = 0;
  float angularError = 0;

  // Compute the errors
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
        m_state = State::ROTATION_ALONG_PATH;
        break;
      }

      // Reset everything for the next step
      m_linear.reset();
      m_angular.reset();
      linearError = 0;
      angularError = 0;
      m_endControlIterations = 0;
    }
  }

  // Compute the commands
  const float linearCommand = m_linear.computeCommand(linearError);
  const float angularCommand = m_angular.computeCommand(angularError);

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

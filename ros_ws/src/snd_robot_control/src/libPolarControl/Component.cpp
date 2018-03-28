#include "Component.h"

#include <algorithm>

Component::Component()
{
}

void Component::init(float _maxVelocity, float _maxAcceleration,
                     float _targetTolerance)
{
  m_maxVelocity = _maxVelocity;
  m_maxAcceleration = _maxAcceleration;
  m_targetTolerance = _targetTolerance;
}

void Component::setTunings(float _kp, float _ki, float _kd, float _maxITerm)
{
  m_kp = _kp;
  m_ki = _ki;
  m_kd = _kd;
  m_maxITerm = _maxITerm;
  // Reset PID
  m_iTerm = 0;
  m_lastError = 0;
}

float Component::computeCommand(float _error, float _dt)
{
  computeActualError(_error, _dt);

  float cmd = m_lastCmd + m_currentError * (m_kp + m_ki * _dt + m_kd / _dt) +
              m_lastError * (-m_kp - 2 * m_kd / _dt) +
              m_lastError2 * (m_kd / _dt);

  /*
  m_iTerm += m_ki * _error;

  if(m_iTerm > m_maxITerm
  {
      m_iTerm = m_maxITerm;
  }
  else
  {
      m_iTerm = -m_maxITerm;
  }

  const float cmd = m_kp * _error + m_kd * (_error - m_lastError) + m_iTerm;

  */
  m_lastError2 = m_lastError;
  m_lastError = m_currentError;
  m_lastCmd = cmd;

  return cmd;
}

void Component::computeActualError(float _error, float _dt)
{
  // Increment the velocity
  const float creneauVel = m_maxVelocity - m_currentVelocity;
  float step = 0;
  if(creneauVel >= 0)
  {
    const float maxStep = m_maxAcceleration * _dt;
    step = std::min(maxStep, creneauVel);
  }
  else
  {
    const float maxStep = -m_maxDeceleration * _dt;
    step = std::max(maxStep, creneauVel);
  }
  m_currentVelocity += step;

  // Increment the position
  const float creneauPos = _error - m_currentError;
  if(creneauPos >= 0)
  {
    const float maxStep = m_currentVelocity * _dt;
    step = std::min(maxStep, creneauPos);
  }
  else
  {
    const float maxStep = -m_currentVelocity * _dt;
    step = std::max(maxStep, creneauPos);
  }
  m_currentError += step;
  /*
  if(_error >= 0)
  {
    if(m_currentVelocity < m_maxVelocity)
    {
      m_currentVelocity += m_maxAcceleration * _dt;
    }
    else
    {
      m_currentVelocity = m_maxVelocity;
    }
  }
  else
  {
    if(m_currentVelocity > -m_maxVelocity)
    {
      m_currentVelocity -= m_maxAcceleration * _dt;
    }
    else
    {
      m_currentVelocity = -m_maxVelocity;
    }
  }
  // Increment distance
  if(_error >= 0)
  {
    if(m_currentError < _error)
    {
      m_currentError += m_currentVelocity * _dt;
    }
    else
    {
      m_currentError = _error;
    }
  }
  else
  {
    if(m_currentError > _error)
    {
      m_currentError -= m_currentVelocity * _dt;
    }
    else
    {
      m_currentError = _error;
    }
  }
  */
}

float Component::computeDesiredCommand()
{
    float ret = 0;
    const float stopDistance = computeStopDistance(m_currentDesiredVelocity, m_maxDeceleration);
    if(std::abs(m_currentActualCommand - m_currentDesiredCommand) > stopDistance)
    {
        // On accelere

    }
    else
    {
        // On decelere

    }
    return ret;
}

float Component::computeStopDistance(float _velocity, float _acceleration)
{
    return (_velocity * _velocity) / (2 * _acceleration);
}

void Component::reset()
{
  m_iTerm = 0;
  m_lastError = 0;
  m_lastError2 = 0;
  m_currentError = 0;
  m_currentVelocity = 0;
  m_currentActualCommand = 0;
  m_currentDesiredCommand = 0;
}

void Component::setActualCommand(float _cmd)
{
  m_currentActualCommand = _cmd;
}

void Component::setDesiredCommand(float _cmd)
{
  m_currentDesiredCommand = _cmd;
}

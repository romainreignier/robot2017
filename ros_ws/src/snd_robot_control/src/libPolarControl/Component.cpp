#include "Component.h"

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

float Component::computeCommand(float _error)
{
  computeActualError(_error);

  float dt = 0.01; // s
  float cmd = m_lastCmd + m_currentError * (m_kp + m_ki * dt + m_kd / dt) +
              m_lastError * (-m_kp - 2 * m_kd / dt) +
              m_lastError2 * (m_kd / dt);

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

  return cmd;
}

void Component::computeActualError(float _error)
{
  // Increment the velocity
  if(_error >= 0)
  {
    if(m_currentVelocity < m_maxVelocity)
    {
      m_currentVelocity += m_maxAcceleration;
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
      m_currentVelocity -= m_maxAcceleration;
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
      m_currentError += m_currentVelocity;
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
      m_currentError -= m_currentVelocity;
    }
    else
    {
      m_currentError = _error;
    }
  }
}

void Component::reset()
{
  m_iTerm = 0;
  m_lastError = 0;
  m_lastError2 = 0;
  m_currentError = 0;
  m_currentVelocity = 0;
}

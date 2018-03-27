#pragma once

class Component
{
public:
  Component();
  void init(float _maxVelocity, float _maxAcceleration, float _targetTolerance);
  void setTunings(float _kp, float _ki, float _kd, float _maxITerm);
  float computeCommand(float _error);
  void computeActualError(float _error);
  void reset();

  // Getters
  float getTolerance() const { return m_targetTolerance; }

protected:
  // Parameters
  float m_maxVelocity = 1;
  float m_maxAcceleration = 1;
  float m_targetTolerance = 0;
  float m_kp = 0;
  float m_ki = 0;
  float m_kd = 0;
  float m_maxITerm = 0;
  // Internal variables
  float m_iTerm = 0;
  float m_lastError = 0;
  float m_lastError2 = 0;
  float m_lastCmd = 0;
  float m_currentVelocity = 0;
  float m_currentError;
};

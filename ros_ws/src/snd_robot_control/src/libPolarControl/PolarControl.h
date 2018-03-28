#pragma once

#include <tuple>

#include "Component.h"
#include "Pose.h"
#include "angles.h"

class PolarControl
{
public:
  enum
  {
    LEFT,
    RIGHT
  };

  enum class State
  {
    ROTATION_ALONG_PATH,
    TRANSLATION_ALONG_PATH,
    ROTATION_ALONG_TARGET,
  };

  PolarControl();
  // Init
  void setMaxOutput(float _max);

  void setTargetPose(const Pose& _target, const Pose& _current);

  std::pair<float, float> computeMotorsCommands(const Pose& _current,
                                                float _dt);
  bool isGoalReached() const { return m_isGoalReached; }

  // Getters
  Component& getLinearComponent() { return m_linear; }
  Component& getAngularComponent() { return m_angular; }

protected:
  float bound(float _in, float _min, float _max);

protected:
  State m_state = State::ROTATION_ALONG_PATH;
  Pose m_targetPose;
  Pose m_currentActualPose;
  Pose m_currentDesiredPose;
  Component m_linear;
  Component m_angular;
  float m_maxOutput = 1;
  float m_endAngularMovementThreshold = angles::from_degrees(1.0f);
  int m_endControlIterations = 0;
  int m_numberIterationsEndControl = 100;
  bool m_isGoalReached;
};

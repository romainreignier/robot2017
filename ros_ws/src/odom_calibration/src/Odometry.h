#pragma once

#include <cstdint>

struct Pose
{
  float x;
  float y;
  float theta;
};

class Odometry
{
public:
  Odometry();
  Pose computeOdom(float currentLeftTicks, float currentRightTicks);
  Pose computeOdomFlo(float currentLeftTicks, float currentRightTicks);
  void reset();

  float m_lastLeftTicks = 0;
  float m_lastRightTicks = 0;

  // From STM32
  static constexpr float kPi = 3.14159265358979323846f;
  float wheelSeparationMM = 111.725f;
  static constexpr int32_t encoderResolution = 2400;
  float leftWheelRadius = 26.125f;
  float rightWheelRadius = 26.0359f; // 26.075
  float LEFT_TICKS_TO_MM = (2 * kPi * leftWheelRadius) / encoderResolution;
  float RIGHT_TICKS_TO_MM = (2 * kPi * rightWheelRadius) / encoderResolution;

  float G_X_mm = 0;
  float G_Y_mm = 0;
  float G_Theta_rad = 0;
  float dD = 0;
  float dA = 0;
  float drr = 0;
  float drg = 0;
};

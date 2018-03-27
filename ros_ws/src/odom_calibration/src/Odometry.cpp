#include "Odometry.h"

#include <angles/angles.h>

Odometry::Odometry()
{
  reset();
}

Pose Odometry::computeOdom(float currentLeftTicks, float currentRightTicks)
{
  const float dLeft = currentLeftTicks - m_lastLeftTicks;
  const float dRight = currentRightTicks - m_lastRightTicks;

  // Save old values
  m_lastLeftTicks = currentLeftTicks;
  m_lastRightTicks = currentRightTicks;

  // Estimation deplacement
  const float dl = static_cast<float>(dLeft) *
                   LEFT_TICKS_TO_MM; // calcul du déplacement de la roue droite
  const float dr = static_cast<float>(dRight) *
                   RIGHT_TICKS_TO_MM; // calcul du déplacement de la roue gauche

  // calcul du déplacement du robot
  const float dD = (dr + dl) / 2;
  // calcul de la variation de l'angle alpha du robot
  const float dA = (dr - dl) / wheelSeparationMM;

  G_X_mm += dD * cos(G_Theta_rad + dA / 2.0);
  G_Y_mm += dD * sin(G_Theta_rad + dA / 2.0);
  /*G_X_mm+= dD * cos(G_Theta_rad + dA/2.0);
  G_Y_mm+= dD * sin(G_Theta_rad + dA/2.0);
  */
  G_Theta_rad += dA;
  G_Theta_rad = angles::normalize_angle(G_Theta_rad);

  return Pose{G_X_mm, G_Y_mm, G_Theta_rad};
}

void Odometry::reset()
{
  G_X_mm = 0;
  G_Y_mm = 0;
  G_Theta_rad = 0;
  dD = 0;
  dA = 0;
  drr = 0;
  drg = 0;
  m_lastLeftTicks = 0;
  m_lastRightTicks = 0;

  LEFT_TICKS_TO_MM = (2 * kPi * leftWheelRadius) / encoderResolution;
  RIGHT_TICKS_TO_MM = (2 * kPi * rightWheelRadius) / encoderResolution;
}

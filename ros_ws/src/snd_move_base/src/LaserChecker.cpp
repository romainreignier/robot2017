#include "LaserChecker.h"

#include <cmath>
#include <iostream>

namespace snd_move_base
{

LaserChecker::LaserChecker()
    : m_isInitialized{false}, m_isLUTInitialized{false}
{}

// 59 - 145 ?
void LaserChecker::setParams(float _robotWidth, float _distThreshold,
                             size_t _minIdx, size_t _maxIdx)
{
  m_robotWidth = _robotWidth;
  m_distThreshold = _distThreshold;
  m_minIdx = _minIdx;
  m_maxIdx = _maxIdx;
  m_isInitialized = true;
}

bool LaserChecker::checkObstacle(const sensor_msgs::LaserScanConstPtr& _msg)
{
  if(!m_isInitialized)
  {
      std::cout << "LaserChecker not Initialized!\n";
      return true;
  }
  if(!m_isLUTInitialized)
  {
    computeLUT(_msg);
  }
  // Only iterate on the interesting points
  for(size_t i = m_minIdx; i < std::min(m_maxIdx, _msg->ranges.size()); ++i)
  {
    const float x = _msg->ranges[i] * m_sinLUT[i];
    const float y = _msg->ranges[i] * m_cosLUT[i];
    // Dist < threshold and point within the robot width + 0.05 m margin
    if(x < m_distThreshold && (std::abs(y) + 0.05) < (m_robotWidth / 2.0))
    {
      return true;
    }
  }
  return false;
}

void LaserChecker::computeLUT(const sensor_msgs::LaserScanConstPtr& _msg)
{
  m_sinLUT.resize(_msg->ranges.size());
  m_cosLUT.resize(_msg->ranges.size());
  m_thetaLUT.resize(_msg->ranges.size());
  for(size_t i = 0; i < _msg->ranges.size(); ++i)
  {
    m_thetaLUT[i] = _msg->angle_min + i * _msg->angle_increment;
    m_sinLUT[i] = sin(m_thetaLUT[i]);
    m_cosLUT[i] = cos(m_thetaLUT[i]);
  }
  m_isLUTInitialized = true;
}

} // namespace snd_move_base

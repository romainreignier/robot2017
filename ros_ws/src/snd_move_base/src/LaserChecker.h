#pragma once

#include <vector>

#include <sensor_msgs/LaserScan.h>

namespace snd_move_base
{
class LaserChecker
{
public:
    LaserChecker();
    void setParams(float _robotWidth, float _distThreshold, size_t _minIdx, size_t _maxIdx);
    bool checkObstacle(const sensor_msgs::LaserScanConstPtr& _msg);
    void computeLUT(const sensor_msgs::LaserScanConstPtr& _msg);

private:
    float m_robotWidth;
    float m_distThreshold;
    size_t m_minIdx;
    size_t m_maxIdx;
    bool m_isInitialized;
    bool m_isLUTInitialized;
    std::vector<float> m_sinLUT;
    std::vector<float> m_cosLUT;
    std::vector<float> m_thetaLUT;
};
} // namespace snd_move_base

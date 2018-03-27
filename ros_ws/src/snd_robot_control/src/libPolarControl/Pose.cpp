#include "Pose.h"

float distance(const Pose &_p1, const Pose &_p2)
{
   return std::hypot(_p2.x - _p1.x, _p2.y - _p1.y);
}

float angle(const Pose &_p1, const Pose &_p2)
{
   return std::atan2(_p2.y - _p1.y, _p2.x - _p1.x);
}

#pragma once

#include <cmath>

struct Pose
{
  float x = 0;
  float y = 0;
  float theta = 0;
};

float distance(const Pose& _p1, const Pose& _p2);
float angle(const Pose& _p1, const Pose& _p2);

#pragma once

#include <algorithm>
#include <cmath>

namespace angles
{

/*!
 * \brief Convert degrees to radians
 */
template <typename T> static inline T from_degrees(T degrees)
{
  return degrees * M_PI / 180.0;
}

/*!
 * \brief Convert radians to degrees
 */
template <typename T> static inline T to_degrees(T radians)
{
  return radians * 180.0 / M_PI;
}

/*!
 * \brief normalize_angle_positive
 *
 *        Normalizes the angle to be 0 to 2*M_PI
 *        It takes and returns radians.
 */
template <typename T> static inline T normalize_angle_positive(T angle)
{
  return fmod(fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
}

/*!
 * \brief normalize
 *
 * Normalizes the angle to be -M_PI circle to +M_PI circle
 * It takes and returns radians.
 *
 */
template <typename T> static inline T normalize_angle(T angle)
{
  T a = normalize_angle_positive(angle);
  if(a > M_PI) a -= 2.0 * M_PI;
  return a;
}

/*!
 * \function
 * \brief shortest_angular_distance
 *
 * Given 2 angles, this returns the shortest angular
 * difference.  The inputs and ouputs are of course radians.
 *
 * The result
 * would always be -pi <= result <= pi.  Adding the result
 * to "from" will always get you an equivelent angle to "to".
 */
template <typename T> static inline T shortest_angular_distance(T from, T to)
{
  return normalize_angle(to - from);
}

/*!
 * \function
 *
 * \brief returns the angle in [-2*M_PI, 2*M_PI]  going the other way along the
 * unit circle.
 * \param angle The angle to which you want to turn in the range [-2*M_PI,
 * 2*M_PI]
 * E.g. two_pi_complement(-M_PI/4) returns 7_M_PI/4
 * two_pi_complement(M_PI/4) returns -7*M_PI/4
 *
 */
template <typename T> static inline T two_pi_complement(T angle)
{
  // check input conditions
  if(angle > 2 * M_PI || angle < -2.0 * M_PI) angle = fmod(angle, 2.0 * M_PI);
  if(angle < 0)
    return (2 * M_PI + angle);
  else if(angle > 0)
    return (-2 * M_PI + angle);

  return (2 * M_PI);
}

/*!
 * \function
 *
 * \brief This function is only intended for internal use and not intended for
 * external use. If you do use it, read the documentation very carefully.
 * Returns the min and max amount (in radians) that can be moved from "from"
 * angle to "left_limit" and "right_limit".
 * \return returns false if "from" angle does not lie in the interval
 * [left_limit,right_limit]
 * \param from - "from" angle - must lie in [-M_PI, M_PI)
 * \param left_limit - left limit of valid interval for angular position - must
 * lie in [-M_PI, M_PI], left and right limits are specified on the unit circle
 * w.r.t to a reference pointing inwards
 * \param right_limit - right limit of valid interval for angular position -
 * must lie in [-M_PI, M_PI], left and right limits are specified on the unit
 * circle w.r.t to a reference pointing inwards
 * \param result_min_delta - minimum (delta) angle (in radians) that can be
 * moved from "from" position before hitting the joint stop
 * \param result_max_delta - maximum (delta) angle (in radians) that can be
 * movedd from "from" position before hitting the joint stop
 */
template <typename T>
static bool find_min_max_delta(T from, T left_limit, T right_limit,
                               T& result_min_delta, T& result_max_delta)
{
  T delta[4];

  delta[0] = shortest_angular_distance(from, left_limit);
  delta[1] = shortest_angular_distance(from, right_limit);

  delta[2] = two_pi_complement(delta[0]);
  delta[3] = two_pi_complement(delta[1]);

  if(delta[0] == 0)
  {
    result_min_delta = delta[0];
    result_max_delta = std::max<T>(delta[1], delta[3]);
    return true;
  }

  if(delta[1] == 0)
  {
    result_max_delta = delta[1];
    result_min_delta = std::min<T>(delta[0], delta[2]);
    return true;
  }

  T delta_min = delta[0];
  T delta_min_2pi = delta[2];
  if(delta[2] < delta_min)
  {
    delta_min = delta[2];
    delta_min_2pi = delta[0];
  }

  T delta_max = delta[1];
  T delta_max_2pi = delta[3];
  if(delta[3] > delta_max)
  {
    delta_max = delta[3];
    delta_max_2pi = delta[1];
  }

  //    printf("%f %f %f %f\n",delta_min,delta_min_2pi,delta_max,delta_max_2pi);
  if((delta_min <= delta_max_2pi) || (delta_max >= delta_min_2pi))
  {
    result_min_delta = delta_max_2pi;
    result_max_delta = delta_min_2pi;
    if(left_limit == -M_PI && right_limit == M_PI)
      return true;
    else
      return false;
  }
  result_min_delta = delta_min;
  result_max_delta = delta_max;
  return true;
}

/*!
 * \function
 *
 * \brief Returns the delta from "from_angle" to "to_angle" making sure it does
 * not violate limits specified by left_limit and right_limit.
 * The valid interval of angular positions is [left_limit,right_limit]. E.g.,
 * [-0.25,0.25] is a 0.5 radians wide interval that contains 0.
 * But [0.25,-0.25] is a 2*M_PI-0.5 wide interval that contains M_PI (but not
 * 0).
 * The value of shortest_angle is the angular difference between "from" and "to"
 * that lies within the defined valid interval.
 * E.g. shortest_angular_distance_with_limits(-0.5,0.5,0.25,-0.25,ss) evaluates
 * ss to 2*M_PI-1.0 and returns true while
 * shortest_angular_distance_with_limits(-0.5,0.5,-0.25,0.25,ss) returns false
 * since -0.5 and 0.5 do not lie in the interval [-0.25,0.25]
 *
 * \return true if "from" and "to" positions are within the limit interval,
 * false otherwise
 * \param from - "from" angle
 * \param to - "to" angle
 * \param left_limit - left limit of valid interval for angular position, left
 * and right limits are specified on the unit circle w.r.t to a reference
 * pointing inwards
 * \param right_limit - right limit of valid interval for angular position, left
 * and right limits are specified on the unit circle w.r.t to a reference
 * pointing inwards
 * \param shortest_angle - result of the shortest angle calculation
 */
template <typename T>
static inline bool
shortest_angular_distance_with_limits(T from, T to, T left_limit, T right_limit,
                                      T& shortest_angle)
{

  T min_delta = -2 * M_PI;
  T max_delta = 2 * M_PI;
  T min_delta_to = -2 * M_PI;
  T max_delta_to = 2 * M_PI;
  bool flag =
    find_min_max_delta(from, left_limit, right_limit, min_delta, max_delta);
  T delta = shortest_angular_distance(from, to);
  T delta_mod_2pi = two_pi_complement(delta);

  if(flag) // from position is within the limits
  {
    if(delta >= min_delta && delta <= max_delta)
    {
      shortest_angle = delta;
      return true;
    }
    else if(delta_mod_2pi >= min_delta && delta_mod_2pi <= max_delta)
    {
      shortest_angle = delta_mod_2pi;
      return true;
    }
    else // to position is outside the limits
    {
      find_min_max_delta(
        to, left_limit, right_limit, min_delta_to, max_delta_to);
      if(fabs(min_delta_to) < fabs(max_delta_to))
        shortest_angle = std::max<T>(delta, delta_mod_2pi);
      else if(fabs(min_delta_to) > fabs(max_delta_to))
        shortest_angle = std::min<T>(delta, delta_mod_2pi);
      else
      {
        if(fabs(delta) < fabs(delta_mod_2pi))
          shortest_angle = delta;
        else
          shortest_angle = delta_mod_2pi;
      }
      return false;
    }
  }
  else // from position is outside the limits
  {
    find_min_max_delta(to, left_limit, right_limit, min_delta_to, max_delta_to);

    if(fabs(min_delta) < fabs(max_delta))
      shortest_angle = std::min<T>(delta, delta_mod_2pi);
    else if(fabs(min_delta) > fabs(max_delta))
      shortest_angle = std::max<T>(delta, delta_mod_2pi);
    else
    {
      if(fabs(delta) < fabs(delta_mod_2pi))
        shortest_angle = delta;
      else
        shortest_angle = delta_mod_2pi;
    }
    return false;
  }

  shortest_angle = delta;
  return false;
}
}

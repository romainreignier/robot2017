#include "LibLut.h"

#include <algorithm>
#include <stdexcept>

LibLut::LibLut(const std::vector<std::pair<double, double>>& _table)
  : m_table(_table)
{
  if(m_table.size() <= 2)
  {
    throw std::runtime_error("LUT : invalid number of values");
  }
  std::sort(m_table.begin(), m_table.end());
}

double LibLut::InterpolateLinear(double _value)
{
  // from https://stackoverflow.com/a/11675205
  // Assumes that "table" is sorted by .first
  // Check if _x is out of bound
  // TODO: see which behavior to have when out of bonds.
  if(_value > m_table.back().first) return m_table.back().first;
  if(_value < m_table.front().first) return m_table.front().first;
  auto it =
    std::lower_bound(m_table.begin(),
                     m_table.end(),
                     std::make_pair(_value, static_cast<double>(-INFINITY)));
  // Corner case
  if(it == m_table.begin()) return it->second;
  auto it2 = it;
  --it2;
  return it2->second +
         (it->second - it2->second) * (_value - it2->first) /
           (it->first - it2->first);
}

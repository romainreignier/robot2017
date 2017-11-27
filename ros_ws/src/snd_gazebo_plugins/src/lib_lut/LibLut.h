#pragma once

#include <utility>
#include <vector>

class LibLut
{
public:
  /*!
   * \brief NewLUT
   * \param _table (ie. [[-10, 1], [-9, 2], [-8, 3], ...])
   */
  LibLut(const std::vector<std::pair<double, double>>& _table);

  /*!
   * \brief InterpolateLinear
   * \param _value
   * \return interpolated value at input
   */
  double InterpolateLinear(double _value);

private:
  std::vector<std::pair<double, double>> m_table;
};
